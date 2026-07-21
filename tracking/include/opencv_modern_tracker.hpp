#pragma once

#include "itracker.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include <string>

// OpenCV modern DNN trackers available on OpenCV 4.6+: DaSiamRPN, GOTURN.
// TrackerVit / TrackerNano need OpenCV >= 4.7/4.8 — optional via CV_VERSION.
enum class OpenCvModernAlgo {
    DaSiamRPN,
    GOTURN,
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 8)
    Vit,
#endif
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
    Nano,
#endif
};

class OpenCvModernTracker : public ITracker {
public:
    explicit OpenCvModernTracker(OpenCvModernAlgo algo);
    const char* name() const override;
    bool load(const std::string& model_dir) override;
    void init(const cv::Mat& frame, const cv::Rect& bbox) override;
    bool track(const cv::Mat& frame, TrackOut& out) override;
    bool is_initialized() const override { return initialized_; }
    void reset() override;

private:
    OpenCvModernAlgo algo_;
    cv::Ptr<cv::Tracker> tr_;
    bool initialized_ = false;
    cv::Rect box_;
    std::string model_dir_;
    float last_score_ = 0.f;
};
