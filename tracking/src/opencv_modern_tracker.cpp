#include "opencv_modern_tracker.hpp"
#include <opencv2/dnn.hpp>
#include <cstdio>
#include <sys/stat.h>

static bool file_exists(const std::string& p) {
    struct stat st {};
    return ::stat(p.c_str(), &st) == 0 && S_ISREG(st.st_mode);
}

OpenCvModernTracker::OpenCvModernTracker(OpenCvModernAlgo algo) : algo_(algo) {}

const char* OpenCvModernTracker::name() const {
    switch (algo_) {
        case OpenCvModernAlgo::DaSiamRPN: return "dasiamrpn";
        case OpenCvModernAlgo::GOTURN: return "goturn";
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 8)
        case OpenCvModernAlgo::Vit: return "vit";
#endif
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
        case OpenCvModernAlgo::Nano: return "ocvnano";
#endif
    }
    return "opencv_modern";
}

void OpenCvModernTracker::reset() {
    tr_.release();
    initialized_ = false;
    box_ = cv::Rect();
    last_score_ = 0.f;
}

bool OpenCvModernTracker::load(const std::string& model_dir) {
    model_dir_ = model_dir;
    auto exists = [](const std::string& p) { return file_exists(p); };

    try {
        if (algo_ == OpenCvModernAlgo::DaSiamRPN) {
            cv::TrackerDaSiamRPN::Params p;
            // Prefer zoo names, then short names from OpenCV samples.
            const std::string a = model_dir + "/dasiamrpn_model.onnx";
            const std::string b = model_dir + "/object_tracking_dasiamrpn_model_2021nov.onnx";
            const std::string c1 = model_dir + "/dasiamrpn_kernel_cls1.onnx";
            const std::string c1b = model_dir + "/object_tracking_dasiamrpn_kernel_cls1_2021nov.onnx";
            const std::string r1 = model_dir + "/dasiamrpn_kernel_r1.onnx";
            const std::string r1b = model_dir + "/object_tracking_dasiamrpn_kernel_r1_2021nov.onnx";
            p.model = exists(a) ? a : b;
            p.kernel_cls1 = exists(c1) ? c1 : c1b;
            p.kernel_r1 = exists(r1) ? r1 : r1b;
            if (!exists(p.model) || !exists(p.kernel_cls1) || !exists(p.kernel_r1)) {
                std::fprintf(stderr,
                    "[DaSiamRPN] missing ONNX in %s — run deploy/fetch_compare_models.sh dasiamrpn\n",
                    model_dir.c_str());
                return false;
            }
            p.backend = cv::dnn::DNN_BACKEND_OPENCV;
            p.target = cv::dnn::DNN_TARGET_CPU;
            tr_ = cv::TrackerDaSiamRPN::create(p);
        } else if (algo_ == OpenCvModernAlgo::GOTURN) {
            cv::TrackerGOTURN::Params p;
            p.modelTxt = model_dir + "/goturn.prototxt";
            p.modelBin = model_dir + "/goturn.caffemodel";
            if (!exists(p.modelTxt) || !exists(p.modelBin)) {
                std::fprintf(stderr,
                    "[GOTURN] missing %s / %s — run deploy/fetch_compare_models.sh goturn\n",
                    p.modelTxt.c_str(), p.modelBin.c_str());
                return false;
            }
            tr_ = cv::TrackerGOTURN::create(p);
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 8)
        } else if (algo_ == OpenCvModernAlgo::Vit) {
            cv::TrackerVit::Params p;
            const std::string a = model_dir + "/object_tracking_vittrack_2023sep.onnx";
            const std::string b = model_dir + "/vitTracker.onnx";
            p.net = exists(a) ? a : b;
            if (!exists(p.net)) {
                std::fprintf(stderr,
                    "[VitTrack] missing ONNX in %s — run deploy/fetch_compare_models.sh vit\n",
                    model_dir.c_str());
                return false;
            }
            p.backend = cv::dnn::DNN_BACKEND_OPENCV;
            p.target = cv::dnn::DNN_TARGET_CPU;
            tr_ = cv::TrackerVit::create(p);
#endif
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
        } else if (algo_ == OpenCvModernAlgo::Nano) {
            cv::TrackerNano::Params p;
            p.backbone = model_dir + "/nanotrack_backbone.onnx";
            p.neckhead = model_dir + "/nanotrack_head.onnx";
            if (!exists(p.backbone) || !exists(p.neckhead)) {
                // Fall back to nanotrackv3 layout
                p.backbone = model_dir + "/../nanotrackv3/nanotrack_backbone.onnx";
                p.neckhead = model_dir + "/../nanotrackv3/nanotrack_head.onnx";
            }
            if (!exists(p.backbone) || !exists(p.neckhead)) {
                std::fprintf(stderr, "[OpenCV Nano] missing backbone/head ONNX in %s\n", model_dir.c_str());
                return false;
            }
            p.backend = cv::dnn::DNN_BACKEND_OPENCV;
            p.target = cv::dnn::DNN_TARGET_CPU;
            tr_ = cv::TrackerNano::create(p);
#endif
        }
    } catch (const cv::Exception& e) {
        std::fprintf(stderr, "[%s] create failed: %s\n", name(), e.what());
        return false;
    }
    if (!tr_) {
        std::fprintf(stderr, "[%s] create returned null\n", name());
        return false;
    }
    std::fprintf(stderr, "[%s] models OK: %s\n", name(), model_dir.c_str());
    return true;
}

void OpenCvModernTracker::init(const cv::Mat& frame, const cv::Rect& bbox) {
    initialized_ = false;
    last_score_ = 0.f;
    if (!tr_ || frame.empty() || bbox.width < 4 || bbox.height < 4) return;
    box_ = bbox;
    tr_->init(frame, box_);
    initialized_ = true;
    last_score_ = 1.f;
    std::fprintf(stderr, "[%s] init %d,%d %dx%d\n", name(), bbox.x, bbox.y, bbox.width, bbox.height);
}

bool OpenCvModernTracker::track(const cv::Mat& frame, TrackOut& out) {
    if (!initialized_ || !tr_ || frame.empty()) return false;
    const bool ok = tr_->update(frame, box_);
    float score = ok ? 0.7f : 0.05f;
    if (algo_ == OpenCvModernAlgo::DaSiamRPN) {
        auto* p = dynamic_cast<cv::TrackerDaSiamRPN*>(tr_.get());
        if (p) score = p->getTrackingScore();
    }
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 8)
    if (algo_ == OpenCvModernAlgo::Vit) {
        auto* p = dynamic_cast<cv::TrackerVit*>(tr_.get());
        if (p) score = p->getTrackingScore();
    }
#endif
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
    if (algo_ == OpenCvModernAlgo::Nano) {
        auto* p = dynamic_cast<cv::TrackerNano*>(tr_.get());
        if (p) score = p->getTrackingScore();
    }
#endif
    last_score_ = score;
    out.score = score;
    out.peak_margin = ok ? 0.2f : 0.f;
    if (box_.width < 2 || box_.height < 2) return false;
    out.cx = std::max(0.f, std::min(1.f, float(box_.x + box_.width * 0.5) / float(frame.cols)));
    out.cy = std::max(0.f, std::min(1.f, float(box_.y + box_.height * 0.5) / float(frame.rows)));
    out.w = std::max(0.01f, float(box_.width) / float(frame.cols));
    out.h = std::max(0.01f, float(box_.height) / float(frame.rows));
    return ok;
}
