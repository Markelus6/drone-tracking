#pragma once

#include <opencv2/core.hpp>
#include <memory>
#include <string>

// Common single-object tracker surface used by multitrack_fc.
struct TrackOut {
    float cx = 0.f;   // normalized [0,1]
    float cy = 0.f;
    float w = 0.f;
    float h = 0.f;
    float score = 0.f;
    float peak_margin = 0.f;
};

class ITracker {
public:
    virtual ~ITracker() = default;
    virtual const char* name() const = 0;
    virtual bool load(const std::string& model_dir) = 0;
    virtual void init(const cv::Mat& frame, const cv::Rect& bbox) = 0;
    virtual bool track(const cv::Mat& frame, TrackOut& out) = 0;
    virtual bool is_initialized() const = 0;
    virtual void reset() = 0;
};

std::unique_ptr<ITracker> create_tracker(const std::string& backend);
