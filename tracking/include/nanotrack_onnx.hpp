#pragma once
#include "itracker.hpp"
#include "nanotrack.hpp"  // reuse NanoTrackConfig helpers conceptually
#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

// NanoTrack V3 via OpenCV DNN + official ONNX (no NCNN convert needed).
class NanoTrackOnnx : public ITracker {
public:
    const char* name() const override { return "nanov3"; }
    bool load(const std::string& model_dir) override;
    void init(const cv::Mat& frame, const cv::Rect& bbox) override;
    bool track(const cv::Mat& frame, TrackOut& out) override;
    bool is_initialized() const override { return initialized_; }
    void reset() override;

private:
    cv::Mat get_subwindow(const cv::Mat& im, cv::Point2f pos, int model_sz, int original_sz, cv::Scalar ave);
    void create_window();
    void create_grids();
    void update(const cv::Mat& x_crop, cv::Point2f& tp, cv::Point2f& tsz, float scale_z, float& score);

    cv::dnn::Net backbone_;
    cv::dnn::Net head_;
    cv::Mat zf_;  // template feature blob NCHW float
    bool initialized_ = false;
    int score_sz_ = 16;
    int im_w_ = 0, im_h_ = 0;
    int track_frames_ = 0;
    cv::Scalar channel_ave_;
    cv::Point target_pos_{0, 0};
    cv::Point2f target_pos_f_{0.f, 0.f};
    cv::Point2f target_sz_{0, 0};
    cv::Point2f initial_sz_{0, 0};
    float cls_score_ = 0.f;
    float peak_margin_ = 0.f;

    // V3 hyperparams (configv3.yaml)
    float penalty_k_ = 0.138f;
    float window_influence_ = 0.455f;
    float lr_ = 0.348f;
    float context_amount_ = 0.5f;
    int exemplar_size_ = 127;
    int instance_size_ = 255;
    int total_stride_ = 16;

    std::vector<float> grid_x_, grid_y_, window_;
};
