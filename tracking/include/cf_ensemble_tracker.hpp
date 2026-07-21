#pragma once

#include "itracker.hpp"
#include <opencv2/tracking.hpp>
#include <opencv2/tracking/tracking_legacy.hpp>

// Correlation-filter ensemble (CFTrack-style without proprietary weights):
// primary KCF + CSRT rescue.
class CfEnsembleTracker : public ITracker {
public:
    const char* name() const override { return "cftrack"; }
    bool load(const std::string&) override { return true; }
    void init(const cv::Mat& frame, const cv::Rect& bbox) override;
    bool track(const cv::Mat& frame, TrackOut& out) override;
    bool is_initialized() const override { return initialized_; }
    void reset() override;

private:
    void recreate(const cv::Mat& frame, const cv::Rect2d& box);
    cv::Ptr<cv::legacy::Tracker> primary_;
    cv::Ptr<cv::legacy::Tracker> rescue_;
    bool initialized_ = false;
    cv::Rect2d box_;
    int fail_streak_ = 0;
    int frames_ = 0;
};
