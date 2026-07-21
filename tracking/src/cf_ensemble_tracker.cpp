#include "cf_ensemble_tracker.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>

void CfEnsembleTracker::reset() {
    primary_.release();
    rescue_.release();
    initialized_ = false;
    box_ = cv::Rect2d();
    fail_streak_ = 0;
    frames_ = 0;
}

void CfEnsembleTracker::recreate(const cv::Mat& frame, const cv::Rect2d& box) {
    primary_ = cv::legacy::TrackerKCF::create();
    rescue_ = cv::legacy::TrackerCSRT::create();
    if (primary_) primary_->init(frame, box);
    if (rescue_) rescue_->init(frame, box);
}

void CfEnsembleTracker::init(const cv::Mat& frame, const cv::Rect& bbox) {
    reset();
    if (frame.empty() || bbox.width < 4 || bbox.height < 4) return;
    box_ = bbox;
    recreate(frame, box_);
    initialized_ = primary_ || rescue_;
    std::fprintf(stderr, "[cftrack] init %d,%d %dx%d (KCF+CSRT ensemble)\n",
                 bbox.x, bbox.y, bbox.width, bbox.height);
}

bool CfEnsembleTracker::track(const cv::Mat& frame, TrackOut& out) {
    if (!initialized_ || frame.empty()) return false;
    frames_++;
    cv::Rect2d b1 = box_, b2 = box_;
    bool ok1 = primary_ && primary_->update(frame, b1);
    bool ok2 = rescue_ && rescue_->update(frame, b2);

    float score = 0.1f;
    if (ok1 && ok2) {
        // Prefer KCF when boxes agree; CSRT when they diverge (occlusion / scale).
        const float cx1 = float(b1.x + b1.width * 0.5), cy1 = float(b1.y + b1.height * 0.5);
        const float cx2 = float(b2.x + b2.width * 0.5), cy2 = float(b2.y + b2.height * 0.5);
        const float dist = std::hypot(cx1 - cx2, cy1 - cy2);
        const float ref = std::max(8.0, 0.35 * std::max(box_.width, box_.height));
        if (dist < ref) {
            box_ = b1;
            score = 0.85f;
        } else {
            box_ = b2;  // CSRT more robust under distractors
            score = 0.65f;
        }
        fail_streak_ = 0;
    } else if (ok2) {
        box_ = b2;
        score = 0.55f;
        fail_streak_ = 0;
    } else if (ok1) {
        box_ = b1;
        score = 0.5f;
        fail_streak_ = 0;
    } else {
        fail_streak_++;
        score = 0.05f;
        if (fail_streak_ == 8) {
            // Re-seed both filters from last box (CFTrack-like recovery).
            recreate(frame, box_);
            std::fprintf(stderr, "[cftrack] reinit after fail streak\n");
        }
    }

    // Soft size clamp vs runaway.
    box_.width = std::clamp(box_.width, 8.0, double(frame.cols));
    box_.height = std::clamp(box_.height, 8.0, double(frame.rows));
    box_.x = std::clamp(box_.x, 0.0, double(frame.cols - box_.width));
    box_.y = std::clamp(box_.y, 0.0, double(frame.rows - box_.height));

    out.score = score;
    out.peak_margin = score > 0.4f ? 0.2f : 0.f;
    out.cx = float(box_.x + box_.width * 0.5) / float(frame.cols);
    out.cy = float(box_.y + box_.height * 0.5) / float(frame.rows);
    out.w = float(box_.width) / float(frame.cols);
    out.h = float(box_.height) / float(frame.rows);
    return ok1 || ok2;
}
