#include "opencv_legacy_tracker.hpp"
#include <algorithm>
#include <cstdio>

OpenCvLegacyTracker::OpenCvLegacyTracker(OpenCvAlgo algo) : algo_(algo) {}

const char* OpenCvLegacyTracker::name() const {
    switch (algo_) {
        case OpenCvAlgo::CSRT: return "csrt";
        case OpenCvAlgo::KCF: return "kcf";
        case OpenCvAlgo::MOSSE: return "mosse";
        case OpenCvAlgo::MIL: return "mil";
        case OpenCvAlgo::MedianFlow: return "medianflow";
        case OpenCvAlgo::TLD: return "tld";
    }
    return "opencv";
}

bool OpenCvLegacyTracker::load(const std::string&) {
    // Classical OpenCV trackers need no weight files.
    return true;
}

void OpenCvLegacyTracker::reset() {
    tr_.release();
    initialized_ = false;
    box_ = cv::Rect2d();
}

void OpenCvLegacyTracker::init(const cv::Mat& frame, const cv::Rect& bbox) {
    reset();
    if (frame.empty() || bbox.width < 4 || bbox.height < 4) return;
    im_w_ = frame.cols;
    im_h_ = frame.rows;
    switch (algo_) {
        case OpenCvAlgo::CSRT: tr_ = cv::legacy::TrackerCSRT::create(); break;
        case OpenCvAlgo::KCF: tr_ = cv::legacy::TrackerKCF::create(); break;
        case OpenCvAlgo::MOSSE: tr_ = cv::legacy::TrackerMOSSE::create(); break;
        case OpenCvAlgo::MIL: tr_ = cv::legacy::TrackerMIL::create(); break;
        case OpenCvAlgo::MedianFlow: tr_ = cv::legacy::TrackerMedianFlow::create(); break;
        case OpenCvAlgo::TLD: tr_ = cv::legacy::TrackerTLD::create(); break;
    }
    if (!tr_) {
        std::fprintf(stderr, "[OpenCV/%s] create failed\n", name());
        return;
    }
    box_ = bbox;
    tr_->init(frame, box_);
    initialized_ = true;
    std::fprintf(stderr, "[OpenCV/%s] init %d,%d %dx%d\n",
                 name(), bbox.x, bbox.y, bbox.width, bbox.height);
}

bool OpenCvLegacyTracker::track(const cv::Mat& frame, TrackOut& out) {
    if (!initialized_ || !tr_ || frame.empty()) return false;
    const bool ok = tr_->update(frame, box_);
    if (!ok || box_.width < 2 || box_.height < 2) {
        out.score = 0.05f;
        out.peak_margin = 0.f;
        // Keep last box so UI does not jump to zero.
    } else {
        out.score = 0.75f;
        out.peak_margin = 0.2f;
    }
    const float cx = static_cast<float>(box_.x + box_.width * 0.5) / static_cast<float>(frame.cols);
    const float cy = static_cast<float>(box_.y + box_.height * 0.5) / static_cast<float>(frame.rows);
    out.cx = std::max(0.f, std::min(1.f, cx));
    out.cy = std::max(0.f, std::min(1.f, cy));
    out.w = std::max(0.01f, static_cast<float>(box_.width) / static_cast<float>(frame.cols));
    out.h = std::max(0.01f, static_cast<float>(box_.height) / static_cast<float>(frame.rows));
    return ok;
}
