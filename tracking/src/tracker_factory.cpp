#include "itracker.hpp"
#include "nanotrack.hpp"
#include "nanotrack_onnx.hpp"
#include "lighttrack.hpp"
#include "opencv_legacy_tracker.hpp"
#include "opencv_modern_tracker.hpp"
#include "cf_ensemble_tracker.hpp"
#include "tctrack_lite.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <memory>
#include <string>

namespace {

std::string lower(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

class NanoTrackAdapter : public ITracker {
public:
    explicit NanoTrackAdapter(bool v3) : v3_(v3) {}
    const char* name() const override { return v3_ ? "nanov3-ncnn" : "nano"; }
    bool load(const std::string& model_dir) override {
        if (v3_) tracker_.apply_preset_v3();
        return tracker_.load_models(model_dir);
    }
    void init(const cv::Mat& frame, const cv::Rect& bbox) override { tracker_.init(frame, bbox); }
    bool track(const cv::Mat& frame, TrackOut& out) override {
        if (!tracker_.is_initialized() || frame.empty()) return false;
        tracker_.track(frame);
        out.cx = tracker_.state.target_pos.x / static_cast<float>(frame.cols);
        out.cy = tracker_.state.target_pos.y / static_cast<float>(frame.rows);
        out.w = tracker_.state.target_sz.x / static_cast<float>(frame.cols);
        out.h = tracker_.state.target_sz.y / static_cast<float>(frame.rows);
        out.score = tracker_.state.cls_score_max;
        out.peak_margin = tracker_.state.peak_margin;
        return true;
    }
    bool is_initialized() const override { return tracker_.is_initialized(); }
    void reset() override { tracker_.release(); }

private:
    bool v3_;
    NanoTrack tracker_;
};

class LightTrackAdapter : public ITracker {
public:
    const char* name() const override { return "light"; }
    bool load(const std::string& model_dir) override { return tracker_.load_models(model_dir); }
    void init(const cv::Mat& frame, const cv::Rect& bbox) override { tracker_.init(frame, bbox); }
    bool track(const cv::Mat& frame, TrackOut& out) override {
        if (!tracker_.is_initialized() || frame.empty()) return false;
        tracker_.track(frame);
        out.cx = tracker_.state.target_pos.x / static_cast<float>(frame.cols);
        out.cy = tracker_.state.target_pos.y / static_cast<float>(frame.rows);
        out.w = tracker_.state.target_sz.x / static_cast<float>(frame.cols);
        out.h = tracker_.state.target_sz.y / static_cast<float>(frame.rows);
        out.score = tracker_.state.cls_score_max;
        out.peak_margin = tracker_.state.peak_margin;
        return true;
    }
    bool is_initialized() const override { return tracker_.is_initialized(); }
    void reset() override { tracker_.release(); }

private:
    LightTrack tracker_;
};

class StubTracker : public ITracker {
public:
    StubTracker(const char* n, const char* hint) : name_(n), hint_(hint) {}
    const char* name() const override { return name_; }
    bool load(const std::string&) override {
        std::fprintf(stderr, "[%s] NOT AVAILABLE: %s\n", name_, hint_);
        return false;
    }
    void init(const cv::Mat&, const cv::Rect&) override {}
    bool track(const cv::Mat&, TrackOut&) override { return false; }
    bool is_initialized() const override { return false; }
    void reset() override {}

private:
    const char* name_;
    const char* hint_;
};

}  // namespace

std::unique_ptr<ITracker> create_tracker(const std::string& backend_in) {
    const std::string b = lower(backend_in);
    if (b == "nanov3" || b == "nano_v3" || b == "v3")
        return std::make_unique<NanoTrackOnnx>();
    if (b == "nano" || b == "nanotrack" || b == "nanov1" || b == "v1")
        return std::make_unique<NanoTrackAdapter>(false);
    if (b == "light" || b == "lighttrack")
        return std::make_unique<LightTrackAdapter>();

    // OpenCV classical
    if (b == "csrt") return std::make_unique<OpenCvLegacyTracker>(OpenCvAlgo::CSRT);
    if (b == "kcf") return std::make_unique<OpenCvLegacyTracker>(OpenCvAlgo::KCF);
    if (b == "mosse") return std::make_unique<OpenCvLegacyTracker>(OpenCvAlgo::MOSSE);
    if (b == "mil") return std::make_unique<OpenCvLegacyTracker>(OpenCvAlgo::MIL);
    if (b == "medianflow" || b == "mf") return std::make_unique<OpenCvLegacyTracker>(OpenCvAlgo::MedianFlow);
    if (b == "tld") return std::make_unique<OpenCvLegacyTracker>(OpenCvAlgo::TLD);

    // CFTrack-style ensemble (no external weights)
    if (b == "cftrack" || b == "cf" || b == "cfensemble")
        return std::make_unique<CfEnsembleTracker>();

    // OpenCV modern DNN
    if (b == "dasiamrpn" || b == "dasiam" || b == "siamrpn")
        return std::make_unique<OpenCvModernTracker>(OpenCvModernAlgo::DaSiamRPN);
    if (b == "goturn")
        return std::make_unique<OpenCvModernTracker>(OpenCvModernAlgo::GOTURN);

    // TCTrack-lite (NanoTrack V3 + online template refresh)
    if (b == "tctrack" || b == "tctrack++" || b == "tctrackpp" || b == "tctracklite")
        return std::make_unique<TcTrackLite>();

#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 8)
    // VitTrack = OSTrack-family (opencv zoo); also used as MixFormer stand-in on CPU
    if (b == "vit" || b == "vittrack" || b == "ostrack" || b == "ostrack256")
        return std::make_unique<OpenCvModernTracker>(OpenCvModernAlgo::Vit);
    if (b == "mixformer" || b == "mixformerv2" || b == "mixformerv2s")
        return std::make_unique<OpenCvModernTracker>(OpenCvModernAlgo::Vit);
#else
    if (b == "vit" || b == "vittrack" || b == "ostrack" || b == "ostrack256")
        return std::make_unique<StubTracker>(
            "ostrack",
            "Needs OpenCV >= 4.8 (TrackerVit). Board has older OpenCV — use TRACKER=dasiamrpn|nanov3|tctrack|cftrack.");
    if (b == "mixformer" || b == "mixformerv2" || b == "mixformerv2s")
        return std::make_unique<StubTracker>(
            "mixformer",
            "Full MixFormerV2-S ONNX needs export + OpenCV>=4.8 VitTrack path. Use TRACKER=dasiamrpn|nanov3|tctrack meanwhile.");
#endif

#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
    if (b == "ocvnano" || b == "opencv_nano")
        return std::make_unique<OpenCvModernTracker>(OpenCvModernAlgo::Nano);
#endif

    std::fprintf(stderr,
        "[multitrack] unknown backend '%s'\n"
        "  known: nano nanov3 light csrt kcf mosse mil medianflow tld\n"
        "         dasiamrpn goturn cftrack tctrack [vit/ostrack/mixformer if OpenCV>=4.8]\n",
        backend_in.c_str());
    return nullptr;
}
