
#pragma once

#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <string>
#include <vector>

struct NanoTrackBBox {
    float cx = 0.f, cy = 0.f, w = 0.f, h = 0.f, score = 0.f;
};

struct NanoTrackConfig {
    int stride = 16;
    float penalty_k = 0.15f;
    float window_influence = 0.475f;
    float lr = 0.38f;
    int exemplar_size = 127;
    int instance_size = 255;
    int total_stride = 16;
    int score_size = 16;
    float context_amount = 0.5f;
    float template_update_rate = 0.01f;
    float template_update_threshold = 0.5f;
    int template_update_interval = 30;
};

struct NanoTrackState {
    int im_h = 0, im_w = 0;
    cv::Scalar channel_ave{0, 0, 0};
    cv::Point target_pos{0, 0};
    cv::Point2f target_sz{0.f, 0.f};
    float cls_score_max = 0.f;
};

class NanoTrack {
public:
    NanoTrack() = default;
    ~NanoTrack();

    bool load_models(const std::string& model_dir);
    void release();
    void init(const cv::Mat& img, const cv::Rect& bbox);
    void track(const cv::Mat& im);
    bool is_initialized() const { return initialized_; }

    NanoTrackState state;
    NanoTrackConfig cfg;

private:
    void create_window();
    void create_grids();
    cv::Mat get_subwindow_tracking(cv::Mat im, cv::Point2f pos, int model_sz, int original_sz, cv::Scalar channel_ave);
    void update(const cv::Mat& x_crop, cv::Point& target_pos, cv::Point2f& target_sz, float scale_z, float& cls_score_max);
    void update_template(const cv::Mat& img);

    std::vector<float> grid_to_search_x, grid_to_search_y, window;

    struct Impl;
    Impl* impl_ = nullptr;
    bool initialized_ = false;
    int score_sz_ = 16;
    int track_frames_ = 0;
    cv::Point2f initial_target_sz_{0.f, 0.f};

    cv::KalmanFilter kf_;
    bool kf_initialized_ = false;
    void init_kalman(float cx, float cy);
    cv::Point2f predict_kalman();
    void update_kalman(float cx, float cy);
};
