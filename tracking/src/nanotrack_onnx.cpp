#include "nanotrack_onnx.hpp"
#include <cmath>
#include <cstdio>
#include <algorithm>

static float sz_whFun(float w, float h) {
    float pad = (w + h) * 0.5f;
    return std::sqrt((w + pad) * (h + pad));
}

void NanoTrackOnnx::reset() {
    initialized_ = false;
    zf_.release();
    cls_score_ = 0;
    peak_margin_ = 0;
    track_frames_ = 0;
}

bool NanoTrackOnnx::load(const std::string& model_dir) {
    const std::string bb = model_dir + "/nanotrack_backbone.onnx";
    const std::string hh = model_dir + "/nanotrack_head.onnx";
    try {
        backbone_ = cv::dnn::readNetFromONNX(bb);
        head_ = cv::dnn::readNetFromONNX(hh);
    } catch (const cv::Exception& e) {
        std::fprintf(stderr, "[NanoTrackOnnx] load fail: %s\n", e.what());
        return false;
    }
    if (backbone_.empty() || head_.empty()) {
        std::fprintf(stderr, "[NanoTrackOnnx] empty nets in %s\n", model_dir.c_str());
        return false;
    }
    backbone_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    backbone_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    head_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    head_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    create_grids();
    create_window();
    std::fprintf(stderr, "[NanoTrackOnnx] Models OK (V3 ONNX): %s\n", model_dir.c_str());
    return true;
}

void NanoTrackOnnx::create_grids() {
    // Match OpenCV TrackerNano::generateGrids
    const int sz = score_sz_;
    const int sz2 = sz / 2;
    const float half_inst = 0.5f * static_cast<float>(instance_size_);
    grid_x_.resize(sz * sz);
    grid_y_.resize(sz * sz);
    for (int y = 0; y < sz; ++y) {
        for (int x = 0; x < sz; ++x) {
            grid_x_[y * sz + x] = static_cast<float>(x - sz2) * total_stride_ + half_inst;
            grid_y_[y * sz + x] = static_cast<float>(y - sz2) * total_stride_ + half_inst;
        }
    }
}

void NanoTrackOnnx::create_window() {
    const int sz = score_sz_;
    std::vector<float> hanning(sz);
    for (int i = 0; i < sz; ++i)
        hanning[i] = 0.5f - 0.5f * std::cos(2.f * 3.14159265f * i / (sz - 1));
    window_.resize(sz * sz);
    for (int y = 0; y < sz; ++y)
        for (int x = 0; x < sz; ++x)
            window_[y * sz + x] = hanning[y] * hanning[x];
}

cv::Mat NanoTrackOnnx::get_subwindow(const cv::Mat& im, cv::Point2f pos, int model_sz, int original_sz, cv::Scalar ave) {
    // OpenCV TrackerNano uses (int)pos - c (truncate toward zero), not round.
    const int c = (original_sz + 1) / 2;
    int cxmin = static_cast<int>(pos.x) - c;
    int cxmax = cxmin + original_sz - 1;
    int cymin = static_cast<int>(pos.y) - c;
    int cymax = cymin + original_sz - 1;
    int lp = std::max(0, -cxmin), tp = std::max(0, -cymin);
    int rp = std::max(0, cxmax - im.cols + 1), bp = std::max(0, cymax - im.rows + 1);
    cxmin += lp; cxmax += lp; cymin += tp; cymax += tp;
    cv::Mat patch;
    if (tp || lp || rp || bp) {
        cv::Mat padded;
        cv::copyMakeBorder(im, padded, tp, bp, lp, rp, cv::BORDER_CONSTANT, ave);
        patch = padded(cv::Rect(cxmin, cymin, cxmax - cxmin + 1, cymax - cymin + 1));
    } else {
        patch = im(cv::Rect(cxmin, cymin, cxmax - cxmin + 1, cymax - cymin + 1));
    }
    cv::Mat resized;
    cv::resize(patch, resized, cv::Size(model_sz, model_sz));
    return resized;
}

void NanoTrackOnnx::init(const cv::Mat& frame, const cv::Rect& bbox) {
    reset();
    if (frame.empty() || bbox.width < 4 || bbox.height < 4) return;
    im_w_ = frame.cols;
    im_h_ = frame.rows;
    target_pos_f_ = cv::Point2f(bbox.x + bbox.width * 0.5f, bbox.y + bbox.height * 0.5f);
    target_pos_ = cv::Point(static_cast<int>(std::lround(target_pos_f_.x)),
                            static_cast<int>(std::lround(target_pos_f_.y)));
    target_sz_ = cv::Point2f(static_cast<float>(bbox.width), static_cast<float>(bbox.height));
    initial_sz_ = target_sz_;
    channel_ave_ = cv::mean(frame);

    float sum = target_sz_.x + target_sz_.y;
    float wc_z = target_sz_.x + context_amount_ * sum;
    float hc_z = target_sz_.y + context_amount_ * sum;
    int s_z = static_cast<int>(std::sqrt(wc_z * hc_z));
    cv::Mat z_crop = get_subwindow(frame, target_pos_f_, exemplar_size_, s_z, channel_ave_);
    cv::Mat blob = cv::dnn::blobFromImage(z_crop, 1.0, cv::Size(), cv::Scalar(), true, false);
    backbone_.setInput(blob);
    zf_ = backbone_.forward().clone();
    // Keep template tensor as head input1 (OpenCV sets it once in init).
    head_.setInput(zf_, "input1");
    initialized_ = true;
    track_frames_ = 0;
    std::fprintf(stderr, "[NanoTrackOnnx] init %d,%d %dx%d  cx=%.1f cy=%.1f\n",
                 bbox.x, bbox.y, bbox.width, bbox.height, target_pos_f_.x, target_pos_f_.y);
}

void NanoTrackOnnx::update(const cv::Mat& x_crop, cv::Point2f& tp, cv::Point2f& tsz, float scale_z, float& score) {
    cv::Mat blob = cv::dnn::blobFromImage(x_crop, 1.0, cv::Size(), cv::Scalar(), true, false);
    backbone_.setInput(blob);
    cv::Mat xf = backbone_.forward();

    head_.setInput(zf_, "input1");
    head_.setInput(xf, "input2");
    std::vector<cv::String> out_names = {"output1", "output2"};
    std::vector<cv::Mat> outs;
    try {
        head_.forward(outs, out_names);
    } catch (const cv::Exception& e) {
        std::fprintf(stderr, "[NanoTrackOnnx] head forward fail: %s\n", e.what());
        score = 0.f;
        peak_margin_ = 0.f;
        return;
    }
    if (outs.size() < 2) {
        score = 0.f;
        peak_margin_ = 0.f;
        return;
    }
    cv::Mat cls = outs[0];
    cv::Mat bbox = outs[1];
    if (cls.dims < 4 || bbox.dims < 4) {
        score = 0.f;
        return;
    }
    const int H = cls.size[2];
    const int W = cls.size[3];
    const int plane = H * W;
    if (H != score_sz_ || W != score_sz_ || static_cast<int>(grid_x_.size()) != plane) {
        score_sz_ = H;
        create_grids();
        create_window();
    }

    const float* cls_ptr = cls.ptr<float>();
    const float* bb_ptr = bbox.ptr<float>();
    auto at_cls = [&](int c, int i) -> float { return cls_ptr[c * plane + i]; };
    auto at_bb = [&](int c, int i) -> float { return bb_ptr[c * plane + i]; };

    // Softmax over bg/fg (OpenCV TrackerNano), keep fg probability.
    std::vector<float> cls_fg(plane), pscore(plane), penalty_v(plane);
    for (int i = 0; i < plane; ++i) {
        float a = at_cls(0, i);
        float b = at_cls(1, i);
        float m = std::max(a, b);
        float ea = std::exp(a - m);
        float eb = std::exp(b - m);
        cls_fg[i] = eb / (ea + eb);
    }

    const float sz_wh = sz_whFun(tsz.x, tsz.y);
    float maxScore = 0, secondScore = 0;
    int best = 0;
    for (int i = 0; i < plane; ++i) {
        float px1 = grid_x_[i] - at_bb(0, i);
        float py1 = grid_y_[i] - at_bb(1, i);
        float px2 = grid_x_[i] + at_bb(2, i);
        float py2 = grid_y_[i] + at_bb(3, i);
        float pw = px2 - px1, ph = py2 - py1;
        float s = sz_whFun(pw, ph) / (sz_wh + 1e-6f);
        float s_c = std::max(s, 1.f / s);
        float ratio = tsz.x / (tsz.y + 1e-8f);
        float r = ratio / (pw / (ph + 1e-8f) + 1e-8f);
        float r_c = std::max(r, 1.f / r);
        float pen = std::exp(-(r_c * s_c - 1.f) * penalty_k_);
        penalty_v[i] = pen;
        pscore[i] = pen * cls_fg[i] * (1.f - window_influence_) + window_[i] * window_influence_;
        if (pscore[i] > maxScore) {
            secondScore = maxScore;
            maxScore = pscore[i];
            best = i;
        } else if (pscore[i] > secondScore) {
            secondScore = pscore[i];
        }
    }

    peak_margin_ = maxScore - secondScore;
    const float raw = cls_fg[best];

    float px1 = grid_x_[best] - at_bb(0, best);
    float py1 = grid_y_[best] - at_bb(1, best);
    float px2 = grid_x_[best] + at_bb(2, best);
    float py2 = grid_y_[best] + at_bb(3, best);
    float pred_xs = (px1 + px2) * 0.5f;
    float pred_ys = (py1 + py2) * 0.5f;
    float pred_w = (px2 - px1) / scale_z;
    float pred_h = (py2 - py1) / scale_z;
    float half = 0.5f * static_cast<float>(instance_size_);
    float diff_xs = (pred_xs - half) / scale_z;
    float diff_ys = (pred_ys - half) / scale_z;
    tsz.x /= scale_z;
    tsz.y /= scale_z;

    float lr = penalty_v[best] * raw * lr_;

    // First frames: stay close to capture box (model peak is quantized to stride).
    float pos_scale = 1.f;
    if (track_frames_ < 3) pos_scale = 0.05f;
    else if (track_frames_ < 10) pos_scale = 0.4f;

    float dx = diff_xs * pos_scale;
    float dy = diff_ys * pos_scale;
    const float max_step = std::max(8.f, 0.5f * std::max(initial_sz_.x, initial_sz_.y));
    const float move = std::sqrt(dx * dx + dy * dy);
    if (move > max_step && move > 1e-3f) {
        dx *= max_step / move;
        dy *= max_step / move;
    }
    tp.x += dx;
    tp.y += dy;

    float size_lr = lr;
    if (track_frames_ < 8) size_lr *= 0.2f;
    size_lr = std::clamp(size_lr, 0.02f, 0.35f);
    float nw = tsz.x * (1.f - size_lr) + pred_w * size_lr;
    float nh = tsz.y * (1.f - size_lr) + pred_h * size_lr;
    tsz.x = std::clamp(nw, std::max(10.f, initial_sz_.x * 0.5f), initial_sz_.x * 2.5f);
    tsz.y = std::clamp(nh, std::max(10.f, initial_sz_.y * 0.5f), initial_sz_.y * 2.5f);

    score = raw;
}

bool NanoTrackOnnx::track(const cv::Mat& frame, TrackOut& out) {
    if (!initialized_ || frame.empty()) return false;
    track_frames_++;
    cv::Point2f tp = target_pos_f_;
    cv::Point2f tsz = target_sz_;

    float sum = tsz.x + tsz.y;
    float hc_z = tsz.y + context_amount_ * sum;
    float wc_z = tsz.x + context_amount_ * sum;
    float s_z = std::sqrt(wc_z * hc_z);
    float scale_z = static_cast<float>(exemplar_size_) / s_z;
    // OpenCV: sx = sz * (instanceSize / exemplarSize)
    float s_x = s_z * (static_cast<float>(instance_size_) / static_cast<float>(exemplar_size_));
    cv::Mat x_crop = get_subwindow(frame, tp, instance_size_, static_cast<int>(s_x), channel_ave_);
    tsz.x *= scale_z;
    tsz.y *= scale_z;

    float score = 0.f;
    update(x_crop, tp, tsz, scale_z, score);

    tp.x = std::clamp(tp.x, 0.f, static_cast<float>(im_w_));
    tp.y = std::clamp(tp.y, 0.f, static_cast<float>(im_h_));
    tsz.x = std::clamp(tsz.x, 10.f, static_cast<float>(im_w_));
    tsz.y = std::clamp(tsz.y, 10.f, static_cast<float>(im_h_));
    target_pos_f_ = tp;
    target_pos_ = cv::Point(static_cast<int>(std::lround(tp.x)), static_cast<int>(std::lround(tp.y)));
    target_sz_ = tsz;
    cls_score_ = score;

    out.cx = tp.x / static_cast<float>(frame.cols);
    out.cy = tp.y / static_cast<float>(frame.rows);
    out.w = tsz.x / static_cast<float>(frame.cols);
    out.h = tsz.y / static_cast<float>(frame.rows);
    out.score = cls_score_;
    out.peak_margin = peak_margin_;
    return true;
}
