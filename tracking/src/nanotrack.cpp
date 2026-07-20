

#include "nanotrack.hpp"
#include <ncnn/net.h>
#include <ncnn/mat.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <cstdio>
#include <algorithm>

struct NanoTrack::Impl {
    ncnn::Net backbone_t, backbone_s, head;
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

NanoTrack::~NanoTrack() { release(); }

bool NanoTrack::load_models(const std::string& model_dir) {
    impl_ = new Impl();
    std::string bp = model_dir + "/nanotrack_backbone_sim-opt.param";
    std::string bb = model_dir + "/nanotrack_backbone_sim-opt.bin";
    std::string hp = model_dir + "/nanotrack_head_sim-opt.param";
    std::string hb = model_dir + "/nanotrack_head_sim-opt.bin";
    if (impl_->backbone_t.load_param(bp.c_str()) != 0) { fprintf(stderr, "[ncnn] backbone param fail\n"); return false; }
    if (impl_->backbone_t.load_model(bb.c_str()) != 0) { fprintf(stderr, "[ncnn] backbone bin fail\n"); return false; }
    if (impl_->backbone_s.load_param(bp.c_str()) != 0) { fprintf(stderr, "[ncnn] backbone2 param fail\n"); return false; }
    if (impl_->backbone_s.load_model(bb.c_str()) != 0) { fprintf(stderr, "[ncnn] backbone2 bin fail\n"); return false; }
    if (impl_->head.load_param(hp.c_str()) != 0) { fprintf(stderr, "[ncnn] head param fail\n"); return false; }
    if (impl_->head.load_model(hb.c_str()) != 0) { fprintf(stderr, "[ncnn] head bin fail\n"); return false; }
    create_grids();
    create_window();
    fprintf(stderr, "[NanoTrack/ncnn] Models OK: %s score_sz=%d\n", model_dir.c_str(), score_sz_);
    return true;
}

void NanoTrack::release() { if (impl_) { delete impl_; impl_ = nullptr; } initialized_ = false; kf_initialized_ = false; track_frames_ = 0; }

cv::Mat NanoTrack::get_subwindow_tracking(cv::Mat im, cv::Point2f pos, int model_sz, int original_sz, cv::Scalar channel_ave) {
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

void NanoTrack::init_kalman(float cx, float cy) {
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

cv::Point2f NanoTrack::predict_kalman() {
    cv::Mat pred = kf_.predict();
    return cv::Point2f(pred.at<float>(0), pred.at<float>(1));
}

void NanoTrack::update_kalman(float cx, float cy) {
    cv::Mat meas = (cv::Mat_<float>(2, 1) << cx, cy);
    kf_.correct(meas);
}

void NanoTrack::init(const cv::Mat& img, const cv::Rect& bbox) {
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
    ncnn::Mat in = ncnn::Mat::from_pixels(z_crop.data, ncnn::Mat::PIXEL_BGR2RGB, z_crop.cols, z_crop.rows);
    ncnn::Extractor ex = impl_->backbone_t.create_extractor();
    ex.input("input", in);
    ex.extract("output", impl_->zf);

    state.channel_ave = avg;
    state.im_h = img.rows;
    state.im_w = img.cols;
    state.target_pos = tp;
    state.target_sz = tsz;
    initial_target_sz_ = tsz;
    initialized_ = true;
    track_frames_ = 0;
    kf_initialized_ = false;
    fprintf(stderr, "[NanoTrack/ncnn] init %d,%d %dx%d cx=%.3f cy=%.3f\n",
            bbox.x, bbox.y, bbox.width, bbox.height, tp.x / (float)img.cols, tp.y / (float)img.rows);
}


void NanoTrack::update_template(const cv::Mat& img) {
    if (!impl_ || track_frames_ % 60 != 0 || track_frames_ < 60) return;
    // Re-extract template features from current frame at target position
    float c = (std::sqrt((state.target_sz.x + cfg.context_amount * (state.target_sz.x + state.target_sz.y)) *
                         (state.target_sz.y + cfg.context_amount * (state.target_sz.x + state.target_sz.y)))) + 0.5f;
    int c_int = static_cast<int>(c);
    int cx = state.target_pos.x, cy = state.target_pos.y;
    int img_w = img.cols, img_h = img.rows;
    int x1 = std::max(0, cx - c_int), y1 = std::max(0, cy - c_int);
    int x2 = std::min(img_w, cx + c_int), y2 = std::min(img_h, cy + c_int);

    cv::Mat patch;
    if (x2 > x1 && y2 > y1) {
        patch = img(cv::Rect(x1, y1, x2 - x1, y2 - y1));
    } else {
        patch = img;
    }
    cv::Mat resized;
    cv::resize(patch, resized, cv::Size(cfg.exemplar_size, cfg.exemplar_size));

    ncnn::Mat in = ncnn::Mat::from_pixels(resized.data, ncnn::Mat::PIXEL_BGR2RGB, resized.cols, resized.rows);
    ncnn::Mat new_zf;
    ncnn::Extractor ex = impl_->backbone_t.create_extractor();
    ex.input("input", in);
    ex.extract("output", new_zf);

    // Blend: 90% old template + 10% new (slow adaptation)
    float alpha = 0.1f;
    float* old_ptr = static_cast<float*>(impl_->zf);
    float* new_ptr = static_cast<float*>(new_zf);
    int total = impl_->zf.w * impl_->zf.h * impl_->zf.c;
    for (int i = 0; i < total; i++) {
        old_ptr[i] = old_ptr[i] * (1.0f - alpha) + new_ptr[i] * alpha;
    }
}

void NanoTrack::update(const cv::Mat& x_crop, cv::Point& tp, cv::Point2f& tsz, float scale_z, float& cls_score_max) {
    if (!impl_) return;
    ncnn::Mat in = ncnn::Mat::from_pixels(x_crop.data, ncnn::Mat::PIXEL_BGR2RGB, x_crop.cols, x_crop.rows);
    ncnn::Mat xf;
    { ncnn::Extractor ex = impl_->backbone_s.create_extractor(); ex.input("input", in); ex.extract("output", xf); }
    ncnn::Mat cls_out, bbox_out;
    { ncnn::Extractor ex = impl_->head.create_extractor(); ex.input("input1", impl_->zf); ex.input("input2", xf); ex.extract("output1", cls_out); ex.extract("output2", bbox_out); }

    const int plane = score_sz_ * score_sz_;
    std::vector<float> cls_sigmoid(plane);
    for (int i = 0; i < plane; i++) {
        // NanoTrack head has two logits: background and target. Using sigmoid
        // on the target logit alone made background patches score ~1.0.
        const float bg = cls_out[0 * plane + i];
        const float fg = cls_out[1 * plane + i];
        cls_sigmoid[i] = 1.0f / (1.0f + std::exp(std::clamp(bg - fg, -40.0f, 40.0f)));
    }

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
    std::vector<float> pscore(plane);
    for (int i = 0; i < plane; i++) {
        pscore[i] = (penalty[i] * cls_sigmoid[i]) * (1.0f - cfg.window_influence) + window[i] * cfg.window_influence;
        if (pscore[i] > maxScore) { maxScore = pscore[i]; r_max = i / score_sz_; c_max = i - r_max * score_sz_; }
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
    const float effective_score = cls_sigmoid[best] * response_quality;

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
    // Follow confident motion quickly. The old 0.20-size cap combined with a
    // 0.35 blend and another output EMA only moved ~7% of one bbox per update,
    // so a target could leave the search crop before the tracker caught up.
    float pos_alpha = 0.0f;
    float max_step_factor = 0.35f;
    if (effective_score >= 0.50f) {
        pos_alpha = 0.95f;
        max_step_factor = 1.25f;
    } else if (effective_score >= 0.30f) {
        pos_alpha = 0.78f;
        max_step_factor = 0.90f;
    } else if (effective_score >= 0.15f) {
        pos_alpha = 0.50f;
        max_step_factor = 0.50f;
    }
    const float max_step = std::max(3.0f, max_step_factor * std::max(tsz.x, tsz.y));
    const float step_scale = move > max_step ? max_step / move : 1.0f;
    tp.x = (int)std::round(tp.x + dx * step_scale * pos_alpha);
    tp.y = (int)std::round(tp.y + dy * step_scale * pos_alpha);

    const float size_alpha = std::min(0.06f, lr);
    float next_w = tsz.x * (1.0f - size_alpha) + pred_w * size_alpha;
    float next_h = tsz.y * (1.0f - size_alpha) + pred_h * size_alpha;
    const float min_w = std::max(10.0f, initial_target_sz_.x * 0.65f);
    const float max_w = std::max(min_w, initial_target_sz_.x * 1.75f);
    const float min_h = std::max(10.0f, initial_target_sz_.y * 0.65f);
    const float max_h = std::max(min_h, initial_target_sz_.y * 1.75f);
    tsz.x = std::clamp(next_w, min_w, max_w);
    tsz.y = std::clamp(next_h, min_h, max_h);

    cls_score_max = effective_score;
}

void NanoTrack::track(const cv::Mat& im) {
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

void NanoTrack::create_window() {
    int sz = cfg.score_size;
    std::vector<float> hanning(sz);
    window.resize(sz * sz);
    for (int i = 0; i < sz; i++) hanning[i] = 0.5f - 0.5f * std::cos(2.0f * 3.14159265f * i / (sz - 1));
    for (int i = 0; i < sz; i++) for (int j = 0; j < sz; j++) window[i * sz + j] = hanning[i] * hanning[j];
}

void NanoTrack::create_grids() {
    int sz = cfg.score_size;
    grid_to_search_x.resize(sz * sz);
    grid_to_search_y.resize(sz * sz);
    for (int i = 0; i < sz; i++) for (int j = 0; j < sz; j++) {
        grid_to_search_x[i * sz + j] = (float)(j * cfg.total_stride);
        grid_to_search_y[i * sz + j] = (float)(i * cfg.total_stride);
    }
}
