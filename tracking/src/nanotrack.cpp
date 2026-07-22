

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

void NanoTrack::apply_preset_v3() {
    // Official NanoTrackV3 (configv3.yaml)
    cfg.penalty_k = 0.138f;
    cfg.window_influence = 0.455f;
    cfg.lr = 0.348f;
    fprintf(stderr, "[NanoTrack] preset=v3 penalty_k=%.3f window=%.3f lr=%.3f\n",
            cfg.penalty_k, cfg.window_influence, cfg.lr);
}

bool NanoTrack::load_models(const std::string& model_dir) {
    impl_ = new Impl();
    // Prefer V3 names, then V1 sim-opt names (current deploy).
    const char* backbone_candidates[][2] = {
        {"/nanotrack_backbone.param", "/nanotrack_backbone.bin"},
        {"/nanotrack_backbone_sim-opt.param", "/nanotrack_backbone_sim-opt.bin"},
        {"/nanotrack_backbone_sim.param", "/nanotrack_backbone_sim.bin"},
    };
    const char* head_candidates[][2] = {
        {"/nanotrack_head.param", "/nanotrack_head.bin"},
        {"/nanotrack_head_sim-opt.param", "/nanotrack_head_sim-opt.bin"},
        {"/nanotrack_head_sim.param", "/nanotrack_head_sim.bin"},
    };

    std::string bp, bb, hp, hb;
    auto try_pair = [&](const char* pairs[][2], int n, std::string& p, std::string& b) -> bool {
        for (int i = 0; i < n; ++i) {
            const std::string pp = model_dir + pairs[i][0];
            const std::string bb2 = model_dir + pairs[i][1];
            FILE* f = std::fopen(pp.c_str(), "rb");
            if (!f) continue;
            std::fclose(f);
            p = pp; b = bb2;
            return true;
        }
        return false;
    };
    if (!try_pair(backbone_candidates, 3, bp, bb) || !try_pair(head_candidates, 3, hp, hb)) {
        fprintf(stderr, "[ncnn] no nanotrack backbone/head in %s\n", model_dir.c_str());
        return false;
    }

    if (impl_->backbone_t.load_param(bp.c_str()) != 0) { fprintf(stderr, "[ncnn] backbone param fail: %s\n", bp.c_str()); return false; }
    if (impl_->backbone_t.load_model(bb.c_str()) != 0) { fprintf(stderr, "[ncnn] backbone bin fail: %s\n", bb.c_str()); return false; }
    if (impl_->backbone_s.load_param(bp.c_str()) != 0) { fprintf(stderr, "[ncnn] backbone2 param fail\n"); return false; }
    if (impl_->backbone_s.load_model(bb.c_str()) != 0) { fprintf(stderr, "[ncnn] backbone2 bin fail\n"); return false; }
    if (impl_->head.load_param(hp.c_str()) != 0) { fprintf(stderr, "[ncnn] head param fail: %s\n", hp.c_str()); return false; }
    if (impl_->head.load_model(hb.c_str()) != 0) { fprintf(stderr, "[ncnn] head bin fail: %s\n", hb.c_str()); return false; }
    create_grids();
    create_window();
    fprintf(stderr, "[NanoTrack/ncnn] Models OK: %s\n  backbone=%s\n  head=%s\n  score_sz=%d\n",
            model_dir.c_str(), bp.c_str(), hp.c_str(), score_sz_);
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
    if (!impl_ || !initialized_ || img.empty()) return;
    if (state.cls_score_max < cfg.template_update_threshold) return;
    if (state.peak_margin < cfg.min_peak_margin) return;
    if (track_frames_ < cfg.template_update_interval) return;
    if (track_frames_ % cfg.template_update_interval != 0) return;

    // Same exemplar crop as init — never blend from a wrong/ambiguous lock.
    float wc_z = state.target_sz.x + cfg.context_amount * (state.target_sz.x + state.target_sz.y);
    float hc_z = state.target_sz.y + cfg.context_amount * (state.target_sz.x + state.target_sz.y);
    float s_z = std::round(std::sqrt(wc_z * hc_z));
    cv::Mat z_crop = get_subwindow_tracking(
        img, cv::Point2f(state.target_pos), cfg.exemplar_size, (int)s_z, state.channel_ave);
    ncnn::Mat in = ncnn::Mat::from_pixels(z_crop.data, ncnn::Mat::PIXEL_BGR2RGB, z_crop.cols, z_crop.rows);
    ncnn::Mat new_zf;
    ncnn::Extractor ex = impl_->backbone_t.create_extractor();
    ex.input("input", in);
    ex.extract("output", new_zf);

    const float alpha = std::clamp(cfg.template_update_rate, 0.01f, 0.25f);
    float* old_ptr = static_cast<float*>(impl_->zf);
    float* new_ptr = static_cast<float*>(new_zf);
    const int total = impl_->zf.w * impl_->zf.h * impl_->zf.c;
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
    // Soft quality blend — never crush score to near-zero (that caused instant LOST).
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

    // Follow primarily by score; weak peak margin only slows the step.
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

    // Size: only adapt when reasonably confident.
    if (effective_score >= 0.28f) {
        const float size_alpha = std::min(0.08f, lr);
        float next_w = tsz.x * (1.0f - size_alpha) + pred_w * size_alpha;
        float next_h = tsz.y * (1.0f - size_alpha) + pred_h * size_alpha;
        const float min_w = std::max(10.0f, initial_target_sz_.x * 0.55f);
        const float max_w = std::max(min_w, initial_target_sz_.x * 2.20f);
        const float min_h = std::max(10.0f, initial_target_sz_.y * 0.55f);
        const float max_h = std::max(min_h, initial_target_sz_.y * 2.20f);
        tsz.x = std::clamp(next_w, min_w, max_w);
        tsz.y = std::clamp(next_h, min_h, max_h);
    }

    state.peak_margin = peak_margin;
    state.response_psr = psr;
    cls_score_max = raw_score;
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

    // Safe online template refresh (skipped automatically if score/margin weak).
    update_template(im);
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
