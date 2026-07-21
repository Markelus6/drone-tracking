#include "lighttrack.hpp"
#include <ncnn/net.h>
#include <ncnn/mat.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <cstdio>
#include <algorithm>

struct LightTrack::Impl {
    ncnn::Net init_net, update_net;
    ncnn::Mat zf;
};

static float fast_sigmoid(float x) { return 1.0f / (1.0f + std::exp(-x)); }
static float sz_whFun(cv::Point2f wh) { float pad = (wh.x + wh.y) * 0.5f; return std::sqrt((wh.x + pad) * (wh.y + pad)); }

static std::vector<float> sz_change_fun(const std::vector<float>& w, const std::vector<float>& h, float sz) {
    int n = (int)w.size();
    std::vector<float> pad(n), out(n);
    for (int i = 0; i < n; i++) pad[i] = (w[i] + h[i]) * 0.5f;
    for (int i = 0; i < n; i++) { float t = std::sqrt((w[i] + pad[i]) * (h[i] + pad[i])) / sz; out[i] = std::max(t, 1.0f / t); }
    return out;
}

static std::vector<float> ratio_change_fun(const std::vector<float>& w, const std::vector<float>& h, cv::Point2f tsz) {
    int n = (int)w.size();
    float ratio = tsz.x / (tsz.y + 1e-8f);
    std::vector<float> out(n);
    for (int i = 0; i < n; i++) { float t = ratio / (w[i] / (h[i] + 1e-8f) + 1e-8f); out[i] = std::max(t, 1.0f / t); }
    return out;
}

LightTrack::~LightTrack() { release(); }

bool LightTrack::load_models(const std::string& model_dir) {
    impl_ = new Impl();
    std::string ip = model_dir + "/lighttrack_init.param";
    std::string ib = model_dir + "/lighttrack_init.bin";
    std::string up = model_dir + "/lighttrack_update.param";
    std::string ub = model_dir + "/lighttrack_update.bin";
    if (impl_->init_net.load_param(ip.c_str()) != 0) { fprintf(stderr, "[LightTrack/ncnn] init param fail\n"); return false; }
    if (impl_->init_net.load_model(ib.c_str()) != 0) { fprintf(stderr, "[LightTrack/ncnn] init bin fail\n"); return false; }
    if (impl_->update_net.load_param(up.c_str()) != 0) { fprintf(stderr, "[LightTrack/ncnn] update param fail\n"); return false; }
    if (impl_->update_net.load_model(ub.c_str()) != 0) { fprintf(stderr, "[LightTrack/ncnn] update bin fail\n"); return false; }
    create_grids();
    create_window();
    fprintf(stderr, "[LightTrack/ncnn] Models OK: %s score_sz=%d\n", model_dir.c_str(), score_sz_);
    return true;
}

void LightTrack::release() { if (impl_) { delete impl_; impl_ = nullptr; } initialized_ = false; kf_initialized_ = false; track_frames_ = 0; }

cv::Mat LightTrack::get_subwindow_tracking(cv::Mat im, cv::Point2f pos, int model_sz, int original_sz, cv::Scalar channel_ave) {
    float c = (float)(original_sz + 1) / 2;
    int cxmin = std::round(pos.x - c), cxmax = cxmin + original_sz - 1;
    int cymin = std::round(pos.y - c), cymax = cymin + original_sz - 1;
    int lp = std::max(0, -cxmin), tp = std::max(0, -cymin);
    int rp = std::max(0, cxmax - im.cols + 1), bp = std::max(0, cymax - im.rows + 1);
    cxmin += lp; cxmax += lp; cymin += tp; cymax += tp;
    cv::Mat patch;
    if (tp || lp || rp || bp) {
        cv::Mat padded;
        cv::copyMakeBorder(im, padded, tp, bp, lp, rp, cv::BORDER_CONSTANT, channel_ave);
        patch = padded(cv::Rect(cxmin, cymin, cxmax - cxmin + 1, cymax - cymin + 1));
    } else {
        patch = im(cv::Rect(cxmin, cymin, cxmax - cxmin + 1, cymax - cymin + 1));
    }
    cv::Mat resized;
    cv::resize(patch, resized, cv::Size(model_sz, model_sz));
    return resized;
}

void LightTrack::init_kalman(float cx, float cy) {
    kf_.init(6, 2, 0);
    kf_.transitionMatrix = (cv::Mat_<float>(6, 6) <<
        1, 0, 1, 0, 0.5f, 0,
        0, 1, 0, 1, 0, 0.5f,
        0, 0, 1, 0, 1, 0,
        0, 0, 0, 1, 0, 1,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1);
    cv::setIdentity(kf_.measurementMatrix);
    cv::setIdentity(kf_.processNoiseCov, cv::Scalar::all(0.03f));
    cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(0.5f));
    cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(1.0f));
    kf_.statePre = (cv::Mat_<float>(6, 1) << cx, cy, 0, 0, 0, 0);
    kf_.statePost = kf_.statePre.clone();
    kf_initialized_ = true;
}

cv::Point2f LightTrack::predict_kalman() {
    cv::Mat pred = kf_.predict();
    return cv::Point2f(pred.at<float>(0), pred.at<float>(1));
}

void LightTrack::update_kalman(float cx, float cy) {
    cv::Mat meas = (cv::Mat_<float>(2, 1) << cx, cy);
    kf_.correct(meas);
}

void LightTrack::init(const cv::Mat& img, const cv::Rect& bbox) {
    if (!impl_ || img.empty()) return;
    cv::Point tp;
    cv::Point2f tsz;
    tp.x = bbox.x + bbox.width / 2;
    tp.y = bbox.y + bbox.height / 2;
    tsz.x = (float)bbox.width;
    tsz.y = (float)bbox.height;

    float wc_z = tsz.x + cfg.context_amount * (tsz.x + tsz.y);
    float hc_z = tsz.y + cfg.context_amount * (tsz.x + tsz.y);
    float s_z = std::round(std::sqrt(wc_z * hc_z));
    cv::Scalar avg = cv::mean(img);
    cv::Mat z_crop = get_subwindow_tracking(img, cv::Point2f(tp), cfg.exemplar_size, (int)s_z, avg);

    // ImageNet normalization: mean=[0.485,0.456,0.406], std=[0.229,0.224,0.225]
    float mean_vals[3] = {0.406f * 255.0f, 0.456f * 255.0f, 0.485f * 255.0f};
    float norm_vals[3] = {1.0f / (0.225f * 255.0f), 1.0f / (0.224f * 255.0f), 1.0f / (0.229f * 255.0f)};
    ncnn::Mat in = ncnn::Mat::from_pixels(z_crop.data, ncnn::Mat::PIXEL_BGR2RGB, z_crop.cols, z_crop.rows);
    in.substract_mean_normalize(mean_vals, norm_vals);

    ncnn::Extractor ex = impl_->init_net.create_extractor();
    ex.input("input1", in);
    ex.extract("output.1", impl_->zf);

    state.channel_ave = avg;
    state.im_h = img.rows;
    state.im_w = img.cols;
    state.target_pos = tp;
    state.target_sz = tsz;
    initial_target_sz_ = tsz;
    initialized_ = true;
    track_frames_ = 0;
    kf_initialized_ = false;
    fprintf(stderr, "[LightTrack/ncnn] init %d,%d %dx%d cx=%.3f cy=%.3f\n",
            bbox.x, bbox.y, bbox.width, bbox.height, tp.x / (float)img.cols, tp.y / (float)img.rows);
}

void LightTrack::update(const cv::Mat& x_crop, cv::Point& tp, cv::Point2f& tsz, float scale_z, float& cls_score_max) {
    if (!impl_) return;

    // ImageNet normalization
    float mean_vals[3] = {0.406f * 255.0f, 0.456f * 255.0f, 0.485f * 255.0f};
    float norm_vals[3] = {1.0f / (0.225f * 255.0f), 1.0f / (0.224f * 255.0f), 1.0f / (0.229f * 255.0f)};
    ncnn::Mat in = ncnn::Mat::from_pixels(x_crop.data, ncnn::Mat::PIXEL_BGR2RGB, x_crop.cols, x_crop.rows);
    in.substract_mean_normalize(mean_vals, norm_vals);

    ncnn::Mat cls_out, bbox_out;
    { ncnn::Extractor ex = impl_->update_net.create_extractor();
      ex.input("input1", impl_->zf);
      ex.input("input2", in);
      ex.extract("output.1", cls_out);
      ex.extract("output.2", bbox_out); }

    const int plane = score_sz_ * score_sz_;
    std::vector<float> cls_sigmoid(plane);
    for (int i = 0; i < plane; i++) cls_sigmoid[i] = fast_sigmoid(cls_out[i]);

    std::vector<float> px1(plane), py1(plane), px2(plane), py2(plane);
    for (int i = 0; i < plane; i++) {
        px1[i] = grid_to_search_x[i] - bbox_out[0 * plane + i];
        py1[i] = grid_to_search_y[i] - bbox_out[1 * plane + i];
        px2[i] = grid_to_search_x[i] + bbox_out[2 * plane + i];
        py2[i] = grid_to_search_y[i] + bbox_out[3 * plane + i];
    }

    std::vector<float> pw(plane), ph(plane);
    for (int i = 0; i < plane; i++) { pw[i] = px2[i] - px1[i]; ph[i] = py2[i] - py1[i]; }

    float sz_wh = sz_whFun(tsz);
    auto s_c = sz_change_fun(pw, ph, sz_wh);
    auto r_c = ratio_change_fun(pw, ph, tsz);
    std::vector<float> penalty(plane);
    for (int i = 0; i < plane; i++) penalty[i] = std::exp(-1.0f * (s_c[i] * r_c[i] - 1.0f) * cfg.penalty_k);

    int r_max = 0, c_max = 0;
    float maxScore = 0;
    float secondScore = 0;
    std::vector<float> pscore(plane);
    for (int i = 0; i < plane; i++) {
        pscore[i] = (penalty[i] * cls_sigmoid[i]) * (1.0f - cfg.window_influence) + window[i] * cfg.window_influence;
        if (pscore[i] > maxScore) {
            secondScore = maxScore;
            maxScore = pscore[i];
            r_max = i / score_sz_;
            c_max = i - r_max * score_sz_;
        } else if (pscore[i] > secondScore) {
            secondScore = pscore[i];
        }
    }

    int best = r_max * score_sz_ + c_max;
    float cls_mean = 0.0f;
    for (float value : cls_sigmoid) cls_mean += value;
    cls_mean /= std::max(1, plane);
    float cls_var = 0.0f;
    for (float value : cls_sigmoid) {
        const float delta = value - cls_mean;
        cls_var += delta * delta;
    }
    cls_var /= std::max(1, plane);
    const float psr = (cls_sigmoid[best] - cls_mean) / (std::sqrt(cls_var) + 1e-6f);
    const float response_quality = std::clamp((psr - 1.5f) / 5.0f, 0.0f, 1.0f);
    const float raw_score = cls_sigmoid[best];
    const float peak_margin = maxScore - secondScore;
    float effective_score = raw_score * (0.55f + 0.45f * response_quality);
    if (peak_margin < cfg.min_peak_margin) {
        effective_score *= 0.85f;
    }

    float pred_xs = (px1[best] + px2[best]) / 2.0f;
    float pred_ys = (py1[best] + py2[best]) / 2.0f;
    float pred_w = px2[best] - px1[best];
    float pred_h = py2[best] - py1[best];

    float diff_xs = (pred_xs - cfg.instance_size / 2.0f) / scale_z;
    float diff_ys = (pred_ys - cfg.instance_size / 2.0f) / scale_z;
    pred_w /= scale_z;
    pred_h /= scale_z;
    tsz.x /= scale_z;
    tsz.y /= scale_z;

    float lr = penalty[best] * cls_sigmoid[best] * cfg.lr;

    float raw_cx = tp.x + diff_xs;
    float raw_cy = tp.y + diff_ys;

    const float dx = raw_cx - tp.x;
    const float dy = raw_cy - tp.y;
    const float move = std::sqrt(dx * dx + dy * dy);
    float pos_alpha = 0.0f;
    float max_step_factor = 0.20f;
    if (effective_score >= 0.50f) {
        pos_alpha = peak_margin >= cfg.min_peak_margin ? 0.90f : 0.55f;
        max_step_factor = peak_margin >= cfg.min_peak_margin ? 1.10f : 0.55f;
    } else if (effective_score >= 0.30f) {
        pos_alpha = 0.45f;
        max_step_factor = 0.55f;
    } else if (effective_score >= cfg.min_move_score) {
        pos_alpha = 0.20f;
        max_step_factor = 0.30f;
    }
    const float max_step = std::max(3.0f, max_step_factor * std::max(tsz.x, tsz.y));
    const float step_scale = move > max_step ? max_step / move : 1.0f;
    tp.x = (int)std::round(tp.x + dx * step_scale * pos_alpha);
    tp.y = (int)std::round(tp.y + dy * step_scale * pos_alpha);

    if (effective_score >= 0.28f) {
        const float size_alpha = std::min(0.08f, lr);
        float next_w = tsz.x * (1.0f - size_alpha) + pred_w * size_alpha;
        float next_h = tsz.y * (1.0f - size_alpha) + pred_h * size_alpha;
        const float min_w = std::max(10.0f, initial_target_sz_.x * 0.55f);
        const float max_w = std::max(min_w, initial_target_sz_.x * 2.50f);
        const float min_h = std::max(10.0f, initial_target_sz_.y * 0.55f);
        const float max_h = std::max(min_h, initial_target_sz_.y * 2.50f);
        tsz.x = std::clamp(next_w, min_w, max_w);
        tsz.y = std::clamp(next_h, min_h, max_h);
    }

    state.peak_margin = peak_margin;
    state.response_psr = psr;
    cls_score_max = raw_score;
}

void LightTrack::track(const cv::Mat& im) {
    if (!initialized_ || im.empty() || !impl_) return;
    track_frames_++;

    cv::Point tp = state.target_pos;
    cv::Point2f tsz = state.target_sz;

    float hc_z = tsz.y + cfg.context_amount * (tsz.x + tsz.y);
    float wc_z = tsz.x + cfg.context_amount * (tsz.x + tsz.y);
    float s_z = std::sqrt(wc_z * hc_z);
    float scale_z = (float)cfg.exemplar_size / s_z;
    float d_search = (float)(cfg.instance_size - cfg.exemplar_size) / 2.0f;
    float pad = d_search / scale_z;
    float s_x = s_z + 2.0f * pad;

    cv::Mat x_crop = get_subwindow_tracking(im, cv::Point2f(tp), cfg.instance_size, (int)std::round(s_x), state.channel_ave);
    tsz.x *= scale_z;
    tsz.y *= scale_z;

    float cls_score_max;
    update(x_crop, tp, tsz, scale_z, cls_score_max);

    tp.x = std::max(0, std::min(state.im_w, tp.x));
    tp.y = std::max(0, std::min(state.im_h, tp.y));
    tsz.x = (float)std::max(10, std::min(state.im_w, (int)tsz.x));
    tsz.y = (float)std::max(10, std::min(state.im_h, (int)tsz.y));

    state.target_pos = tp;
    state.target_sz = tsz;
    state.cls_score_max = cls_score_max;
}

void LightTrack::create_window() {
    int sz = cfg.score_size;
    std::vector<float> hanning(sz);
    window.resize(sz * sz);
    for (int i = 0; i < sz; i++) hanning[i] = 0.5f - 0.5f * std::cos(2.0f * 3.14159265f * i / (sz - 1));
    for (int i = 0; i < sz; i++) for (int j = 0; j < sz; j++) window[i * sz + j] = hanning[i] * hanning[j];
}

void LightTrack::create_grids() {
    int sz = cfg.score_size;
    grid_to_search_x.resize(sz * sz);
    grid_to_search_y.resize(sz * sz);
    for (int i = 0; i < sz; i++) for (int j = 0; j < sz; j++) {
        grid_to_search_x[i * sz + j] = (float)(j * cfg.total_stride);
        grid_to_search_y[i * sz + j] = (float)(i * cfg.total_stride);
    }
}
