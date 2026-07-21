#pragma once
#include "itracker.hpp"
#include <opencv2/tracking.hpp>
#include <opencv2/tracking/tracking_legacy.hpp>
#include <string>

enum class OpenCvAlgo { CSRT, KCF, MOSSE, MIL, MedianFlow, TLD };

class OpenCvLegacyTracker : public ITracker {
public:
    explicit OpenCvLegacyTracker(OpenCvAlgo algo);
    const char* name() const override;
    bool load(const std::string& model_dir) override;
    void init(const cv::Mat& frame, const cv::Rect& bbox) override;
    bool track(const cv::Mat& frame, TrackOut& out) override;
    bool is_initialized() const override { return initialized_; }
    void reset() override;

private:
    OpenCvAlgo algo_;
    cv::Ptr<cv::legacy::Tracker> tr_;
    bool initialized_ = false;
    int im_w_ = 0, im_h_ = 0;
    cv::Rect2d box_;
};
