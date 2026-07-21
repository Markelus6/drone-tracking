#pragma once
#include "itracker.hpp"
#include "nanotrack_onnx.hpp"
#include <algorithm>
#include <cstdlib>
#include <cstdio>
#include <cmath>

// Temporal-context lite tracker for boards without full TCTrack weights:
// NanoTrack V3 + periodic online template refresh (TCTrack-inspired).
class TcTrackLite : public ITracker {
public:
    const char* name() const override { return "tctrack"; }
    bool load(const std::string& model_dir) override {
        refresh_every_ = 45;
        if (const char* e = std::getenv("TCTRACK_REFRESH")) {
            refresh_every_ = std::max(10, std::atoi(e));
        }
        return inner_.load(model_dir);
    }
    void init(const cv::Mat& frame, const cv::Rect& bbox) override {
        frames_ = 0;
        last_bb_ = bbox;
        inner_.init(frame, bbox);
        std::fprintf(stderr, "[tctrack] NanoTrackV3 + template refresh every %d frames\n", refresh_every_);
    }
    bool track(const cv::Mat& frame, TrackOut& out) override {
        if (!inner_.is_initialized()) return false;
        const bool ok = inner_.track(frame, out);
        frames_++;
        if (!ok) return false;
        last_bb_ = cv::Rect(
            int(std::lround((out.cx - out.w * 0.5f) * frame.cols)),
            int(std::lround((out.cy - out.h * 0.5f) * frame.rows)),
            std::max(4, int(std::lround(out.w * frame.cols))),
            std::max(4, int(std::lround(out.h * frame.rows))));
        last_bb_ &= cv::Rect(0, 0, frame.cols, frame.rows);
        if (frames_ > 0 && (frames_ % refresh_every_) == 0 && last_bb_.width >= 24 && last_bb_.height >= 24
            && out.score > 0.35f) {
            inner_.init(frame, last_bb_);
            std::fprintf(stderr, "[tctrack] online template refresh @ frame %d score=%.2f\n",
                         frames_, out.score);
        }
        return true;
    }
    bool is_initialized() const override { return inner_.is_initialized(); }
    void reset() override {
        frames_ = 0;
        inner_.reset();
    }

private:
    NanoTrackOnnx inner_;
    cv::Rect last_bb_;
    int frames_ = 0;
    int refresh_every_ = 45;
};
