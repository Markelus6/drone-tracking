// lighttrack_fc — LightTrack NCNN on orchestrator SHM → UDP bbox + MJPEG :5005.
// Standalone module — init via UDP :12349 {"cmd":"init","bbox_norm":[cx,cy,w,h]}.
// Manual capture Web UI on :5006.

#include "lighttrack.hpp"
#include "camera_orchestrator.hpp"

#include <opencv2/opencv.hpp>

#include <csignal>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <fcntl.h>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

static std::atomic<bool> g_running{true};
static std::mutex g_cmd_mtx;
static std::atomic<bool> g_need_init{false};
static float g_init_cx = 0.5f, g_init_cy = 0.5f, g_init_w = 0.2f, g_init_h = 0.2f;
static std::atomic<bool> g_tracking{false};
static std::atomic<bool> g_need_full_reset{false};

static void on_sig(int) { g_running = false; }

static int lost_frames_threshold() {
    if (const char* e = std::getenv("LIGHTTRACK_LOST_FRAMES")) return std::max(1, std::atoi(e));
    return 45;
}

static float lost_score_threshold() {
    if (const char* e = std::getenv("LIGHTTRACK_LOST_SCORE")) return std::max(0.01f, (float)std::atof(e));
    return 0.10f;
}

static std::string shm_name_from_dev(const std::string& dev) {
    std::string n = dev;
    for (char& c : n) if (c == '/') c = '_';
    return "/drone_cam" + n;
}

// Persistent SHM Reader
static std::string g_shm_name;
static void* g_shm_ptr = nullptr;
static size_t g_shm_size = 0;

static bool shm_open_persistent(const std::string& shm_name) {
    g_shm_name = shm_name;
    int fd = shm_open(shm_name.c_str(), O_RDONLY, 0666);
    if (fd < 0) return false;
    struct stat sb;
    if (fstat(fd, &sb) < 0) { close(fd); return false; }
    g_shm_size = static_cast<size_t>(sb.st_size);
    g_shm_ptr = mmap(nullptr, g_shm_size, PROT_READ, MAP_SHARED, fd, 0);
    close(fd);
    if (g_shm_ptr == MAP_FAILED) { g_shm_ptr = nullptr; return false; }
    std::fprintf(stderr, "[LightTrack] SHM persistent mmap OK: %s (%zu KB)\n", shm_name.c_str(), g_shm_size / 1024);
    return true;
}

static void shm_close_persistent() {
    if (g_shm_ptr && g_shm_ptr != MAP_FAILED) munmap(g_shm_ptr, g_shm_size);
    g_shm_ptr = nullptr;
}

static bool shm_read(cv::Mat& frame, uint64_t* frame_id = nullptr) {
    if (!g_shm_ptr) return false;
    const ShmHeader* hdr = static_cast<const ShmHeader*>(g_shm_ptr);
    __sync_synchronize();
    if (hdr->ready && hdr->width > 0 && hdr->height > 0) {
        if (frame_id) *frame_id = hdr->frame_id;
        const uint8_t* data = static_cast<const uint8_t*>(g_shm_ptr) + sizeof(ShmHeader);
        cv::Mat(hdr->height, hdr->width,
                hdr->channels == 1 ? CV_8UC1 : CV_8UC3,
                const_cast<uint8_t*>(data)).copyTo(frame);
        return true;
    }
    frame = cv::Mat();
    return false;
}

static uint64_t shm_peek_frame_id() {
    if (!g_shm_ptr) return 0;
    const ShmHeader* hdr = static_cast<const ShmHeader*>(g_shm_ptr);
    __sync_synchronize();
    return (hdr->ready && hdr->width > 0) ? hdr->frame_id : 0;
}

// MJPEG :5005
static std::mutex g_mjpeg_mtx;
static std::vector<uchar> g_mjpeg_jpeg;
static std::atomic<uint64_t> g_mjpeg_seq{0};
static std::atomic<int> g_mjpeg_clients{0};
static int g_mjpeg_port = 5005;
static int g_mjpeg_max_clients = 8;

static bool mjpeg_send_all(int fd, const char* data, size_t len) {
    size_t off = 0;
    while (off < len) {
        ssize_t n = send(fd, data + off, len - off, MSG_NOSIGNAL);
        if (n <= 0) return false;
        off += static_cast<size_t>(n);
    }
    return true;
}

static void mjpeg_handle_client(int client_sock) {
    g_mjpeg_clients.fetch_add(1, std::memory_order_relaxed);
    const char* hdr =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        "Cache-Control: no-cache, no-store\r\n"
        "Pragma: no-cache\r\n"
        "Connection: close\r\n\r\n";
    if (!mjpeg_send_all(client_sock, hdr, strlen(hdr))) {
        close(client_sock);
        g_mjpeg_clients.fetch_sub(1, std::memory_order_relaxed);
        return;
    }
    uint64_t last_seq = 0;
    while (g_running) {
        uint64_t cur = g_mjpeg_seq.load(std::memory_order_acquire);
        if (cur == last_seq) { usleep(5000); continue; }
        last_seq = cur;
        std::vector<uchar> jpeg;
        { std::lock_guard<std::mutex> lk(g_mjpeg_mtx); jpeg = g_mjpeg_jpeg; }
        if (jpeg.empty()) continue;
        char part[128];
        int n = snprintf(part, sizeof(part),
            "\r\n--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n", jpeg.size());
        if (!mjpeg_send_all(client_sock, part, n)) break;
        if (!mjpeg_send_all(client_sock, reinterpret_cast<const char*>(jpeg.data()), jpeg.size())) break;
    }
    close(client_sock);
    g_mjpeg_clients.fetch_sub(1, std::memory_order_relaxed);
}

static void mjpeg_accept_thread() {
    int srv = socket(AF_INET, SOCK_STREAM, 0);
    if (srv < 0) return;
    int opt = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    struct sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = INADDR_ANY;
    a.sin_port = htons(static_cast<uint16_t>(g_mjpeg_port));
    if (bind(srv, reinterpret_cast<struct sockaddr*>(&a), sizeof(a)) < 0) {
        std::fprintf(stderr, "[LightTrack] MJPEG bind :%d failed\n", g_mjpeg_port);
        close(srv);
        return;
    }
    listen(srv, 4);
    std::printf("[LightTrack] MJPEG :%d ready\n", g_mjpeg_port);
    while (g_running) {
        struct timeval tv{0, 500000};
        setsockopt(srv, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        int cfd = accept(srv, nullptr, nullptr);
        if (cfd < 0) continue;
        if (g_mjpeg_max_clients > 0 && g_mjpeg_clients.load() >= g_mjpeg_max_clients) {
            const char* busy = "HTTP/1.1 503 Busy\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nBusy\n";
            send(cfd, busy, strlen(busy), MSG_NOSIGNAL);
            close(cfd);
            continue;
        }
        std::thread(mjpeg_handle_client, cfd).detach();
    }
    close(srv);
}

// Double-buffered JPEG encode
static std::mutex g_viz_mtx;
static cv::Mat g_viz_frame;
static std::atomic<bool> g_viz_new{false};
static int g_stream_max_w = 480;  // preview width; tracker stays full-res

static int env_int(const char* a, const char* b, int defv, int lo, int hi) {
    if (const char* e = std::getenv(a)) return std::max(lo, std::min(hi, std::atoi(e)));
    if (b) if (const char* e = std::getenv(b)) return std::max(lo, std::min(hi, std::atoi(e)));
    return defv;
}

static void mjpeg_encoder_thread() {
    // ~30 FPS preview. Load cut via smaller encode size + quality, not by dropping FPS.
    const int period_ms = env_int("LIGHTTRACK_MJPEG_PERIOD_MS", "VISION_MJPEG_PERIOD_MS", 33, 16, 200);
    const int jpeg_q = env_int("LIGHTTRACK_MJPEG_QUALITY", "VISION_MJPEG_QUALITY", 40, 20, 95);
    using clock = std::chrono::steady_clock;
    auto last_encode = clock::now();
    std::fprintf(stderr, "[LightTrack] MJPEG encode period=%dms q=%d stream_max_w=%d\n",
                 period_ms, jpeg_q, g_stream_max_w);
    while (g_running) {
        if (!g_viz_new.load(std::memory_order_acquire)) { usleep(1000); continue; }
        auto now = clock::now();
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_encode).count();
        if (elapsed < period_ms) { usleep(1000); continue; }
        cv::Mat local;
        { std::lock_guard<std::mutex> lk(g_viz_mtx); local = std::move(g_viz_frame); g_viz_new.store(false); }
        if (local.empty()) continue;
        std::vector<uchar> buf;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_q};
        cv::imencode(".jpg", local, buf, params);
        { std::lock_guard<std::mutex> lk(g_mjpeg_mtx); g_mjpeg_jpeg = std::move(buf); }
        g_mjpeg_seq.fetch_add(1, std::memory_order_release);
        last_encode = clock::now();
    }
}

// Tracking overlay drawing
static void draw_grid(cv::Mat& img, float track_cx = -1, float track_cy = -1, float track_h = 0, float track_score = 0) {
    int w = img.cols, h = img.rows;
    float dz = 0.12f;
    int cx = w / 2, cy = h / 2;
    cv::Scalar green(0, 200, 0), orange(0, 140, 255), white(255, 255, 255);

    cv::rectangle(img, cv::Point((int)((0.5f - dz) * w), (int)((0.5f - dz) * h)),
                  cv::Point((int)((0.5f + dz) * w), (int)((0.5f + dz) * h)), green, 1, cv::LINE_AA);

    if (track_cx >= 0 && track_h > 0.01f) {
        float err_x = (track_cx - 0.5f) * 2.0f;
        float err_y = (track_cy - 0.5f) * 2.0f;
        int yaw_pwm = 1500 + (int)(err_x * 350);
        yaw_pwm = std::max(1300, std::min(1700, yaw_pwm));

        int bx = (int)(track_cx * w);
        cv::arrowedLine(img, cv::Point(cx, cy), cv::Point(bx, cy),
                        (std::abs(err_x) < dz) ? green : orange, 3, cv::LINE_AA, 0, 0.4f);
        cv::arrowedLine(img, cv::Point(cx, cy), cv::Point(cx, (int)(track_cy * h)),
                        cv::Scalar(255, 255, 0), 2, cv::LINE_AA, 0, 0.3f);

        const char* dir = (err_x < -dz) ? "YAW>>" : (err_x > dz) ? "<<YAW" : "CENTER";
        char buf[64];
        snprintf(buf, sizeof(buf), "%s  PWM:%d", dir, yaw_pwm);
        cv::putText(img, buf, cv::Point(5, 14), cv::FONT_HERSHEY_SIMPLEX, 0.4, orange, 1, cv::LINE_AA);
        snprintf(buf, sizeof(buf), "ex=%.2f ey=%.2f h=%.2f sc=%.2f", err_x, err_y, track_h, track_score);
        cv::putText(img, buf, cv::Point(5, h - 6), cv::FONT_HERSHEY_SIMPLEX, 0.35, white, 1, cv::LINE_AA);
    } else {
        cv::putText(img, "NO TARGET", cv::Point(cx - 30, 14), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(100, 100, 100), 1, cv::LINE_AA);
    }
}

static void draw_tracking(cv::Mat& img, float cx, float cy, float nw, float nh, float score) {
    if (img.empty() || nh <= 0.01f) return;
    if (nw <= 0.01f) nw = nh;
    const int W = img.cols, H = img.rows;
    const float bw = nw * static_cast<float>(W);
    const float bh = nh * static_cast<float>(H);
    int x = static_cast<int>(cx * W - bw * 0.5f);
    int y = static_cast<int>(cy * H - bh * 0.5f);
    int w = static_cast<int>(bw);
    int hh = static_cast<int>(bh);
    x = std::max(0, std::min(x, W - 1));
    y = std::max(0, std::min(y, H - 1));
    w = std::max(1, std::min(w, W - x));
    hh = std::max(1, std::min(hh, H - y));
    cv::Rect bb(x, y, w, hh);
    const cv::Scalar green(0, 255, 0);
    const cv::Scalar cyan(255, 255, 0);
    cv::rectangle(img, bb, green, 3, cv::LINE_AA);
    int tl = std::min(bb.width, bb.height) / 5;
    cv::line(img, cv::Point(bb.x, bb.y), cv::Point(bb.x + tl, bb.y), cyan, 2, cv::LINE_AA);
    cv::line(img, cv::Point(bb.x, bb.y), cv::Point(bb.x, bb.y + tl), cyan, 2, cv::LINE_AA);
    cv::line(img, cv::Point(bb.x + bb.width, bb.y), cv::Point(bb.x + bb.width - tl, bb.y), cyan, 2, cv::LINE_AA);
    cv::line(img, cv::Point(bb.x + bb.width, bb.y), cv::Point(bb.x + bb.width, bb.y + tl), cyan, 2, cv::LINE_AA);
    char lbl[64];
    snprintf(lbl, sizeof(lbl), "LightTrack %.2f", score);
    cv::putText(img, lbl, cv::Point(bb.x, std::max(20, bb.y - 6)), cv::FONT_HERSHEY_SIMPLEX, 0.6, green, 2, cv::LINE_AA);
    int cxp = bb.x + bb.width / 2;
    int cyp = bb.y + bb.height / 2;
    cv::drawMarker(img, cv::Point(cxp, cyp), cyan, cv::MARKER_CROSS, 20, 2, cv::LINE_AA);
    cv::circle(img, cv::Point(cxp, cyp), 5, green, -1, cv::LINE_AA);
}

// CMD thread (:12348)
static void cmd_thread(int port) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) return;
    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    struct sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = INADDR_ANY;
    a.sin_port = htons(static_cast<uint16_t>(port));
    if (bind(sock, reinterpret_cast<struct sockaddr*>(&a), sizeof(a)) < 0) {
        std::fprintf(stderr, "[LightTrack] cmd bind :%d failed\n", port);
        close(sock);
        return;
    }
    std::printf("[LightTrack] Commands UDP :%d (init/reset)\n", port);
    char buf[512];
    while (g_running) {
        struct timeval tv{0, 200000};
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        ssize_t n = recv(sock, buf, sizeof(buf) - 1, 0);
        if (n <= 0) continue;
        buf[n] = '\0';
        std::lock_guard<std::mutex> lk(g_cmd_mtx);
        if (std::strstr(buf, "reset") || std::strstr(buf, "stop")) {
            g_tracking = false;
            g_need_init = false;
            g_need_full_reset = true;
        } else if (std::strstr(buf, "init")) {
            const bool force = std::strstr(buf, "force") != nullptr;
            if (g_tracking.load() && !force) continue;
            const char* p = std::strstr(buf, "bbox_norm");
            if (p) {
                float cx = 0.5f, cy = 0.5f, w = 0.2f, h = 0.2f;
                const char* b = std::strchr(p, '[');
                if (b) std::sscanf(b, "[%f,%f,%f,%f]", &cx, &cy, &w, &h);
                if (w <= 0.01f) w = h;
                g_init_cx = cx; g_init_cy = cy; g_init_w = w; g_init_h = h;
                g_tracking = false;
                g_need_init = true;
            }
        }
    }
    close(sock);
}

// Manual Capture Web UI (:5006)
static void manual_capture_thread() {
    int port = 5006;
    int srv = socket(AF_INET, SOCK_STREAM, 0);
    if (srv < 0) { fprintf(stderr, "[LightTrack/MC] socket fail\n"); return; }
    int opt = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    struct sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = INADDR_ANY;
    a.sin_port = htons(port);
    if (bind(srv, (struct sockaddr*)&a, sizeof(a)) < 0) {
        fprintf(stderr, "[LightTrack/MC] bind :%d fail\n", port);
        close(srv);
        return;
    }
    listen(srv, 4);
    fprintf(stderr, "[LightTrack/MC] Web UI :%d ready\n", port);

    const char* page = R"rawliteral(
<!DOCTYPE html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>LightTrack Capture</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#111;color:#0f0;font-family:monospace;display:flex;flex-direction:column;align-items:center;gap:8px;padding:8px}
h1{font-size:16px}
#wrap{position:relative;display:inline-block;border:1px solid #0f0}
#wrap img{display:block;width:100%}
#wrap canvas{position:absolute;top:0;left:0;width:100%;height:100%;cursor:crosshair}
#status{font-size:12px;color:#0f0}
button{background:#0f0;color:#111;border:none;padding:6px 16px;font-size:13px;font-weight:bold;border-radius:4px;cursor:pointer;font-family:inherit}
button:hover{background:#6f6}
</style></head><body>
<h1>LightTrack Manual Capture</h1>
<div id="wrap">
  <img id="video" src="" onerror="this.src='data:,'">
  <canvas id="canvas" width="640" height="480"></canvas>
</div>
<div id="status">Draw rectangle around object to track</div>
<script>
const img = document.getElementById('video');
img.src = 'http://' + location.hostname + ':5005/';
const cvs = document.getElementById('canvas');
const ctx = cvs.getContext('2d');
let startX, startY, isDrawing = false;

img.onload = () => { cvs.width = img.naturalWidth || 640; cvs.height = img.naturalHeight || 480; };

cvs.addEventListener('mousedown', e => {
    const r = cvs.getBoundingClientRect();
    startX = (e.clientX - r.left) / r.width * cvs.width;
    startY = (e.clientY - r.top) / r.height * cvs.height;
    isDrawing = true;
    ctx.clearRect(0, 0, cvs.width, cvs.height);
});

cvs.addEventListener('mousemove', e => {
    if (!isDrawing) return;
    const r = cvs.getBoundingClientRect();
    const ex = (e.clientX - r.left) / r.width * cvs.width;
    const ey = (e.clientY - r.top) / r.height * cvs.height;
    ctx.clearRect(0, 0, cvs.width, cvs.height);
    ctx.strokeStyle = '#0f0'; ctx.lineWidth = 2;
    ctx.strokeRect(startX, startY, ex - startX, ey - startY);
});

cvs.addEventListener('mouseup', e => {
    if (!isDrawing) return;
    isDrawing = false;
    const r = cvs.getBoundingClientRect();
    const ex = (e.clientX - r.left) / r.width * cvs.width;
    const ey = (e.clientY - r.top) / r.height * cvs.height;
    const cx = ((startX + ex) / 2) / cvs.width;
    const cy = ((startY + ey) / 2) / cvs.height;
    const w = Math.abs(ex - startX) / cvs.width;
    const h = Math.abs(ey - startY) / cvs.height;
    if (w < 0.03 || h < 0.03) {
        document.getElementById('status').textContent =
            'Draw a box (click-drag), not a click';
        ctx.clearRect(0, 0, cvs.width, cvs.height);
        return;
    }
    document.getElementById('status').textContent =
        'Sending: cx=' + cx.toFixed(3) + ' cy=' + cy.toFixed(3) +
        ' w=' + w.toFixed(3) + ' h=' + h.toFixed(3);
    fetch('/api/capture?cx=' + cx + '&cy=' + cy + '&w=' + w + '&h=' + h)
        .then(r => r.text()).then(t => {
            document.getElementById('status').textContent = t;
            ctx.clearRect(0, 0, cvs.width, cvs.height);
        });
});
</script></body></html>
)rawliteral";

    while (g_running) {
        struct timeval tv{0, 500000};
        setsockopt(srv, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        int cfd = accept(srv, nullptr, nullptr);
        if (cfd < 0) continue;

        char req[2048];
        ssize_t n = recv(cfd, req, sizeof(req) - 1, 0);
        if (n <= 0) { close(cfd); continue; }
        req[n] = '\0';

        if (strstr(req, "GET /api/capture?")) {
            const char* p = strstr(req, "cx=");
            float cx = 0.5f, cy = 0.5f, w = 0.2f, h = 0.2f;
            if (p) std::sscanf(p, "cx=%f&cy=%f&w=%f&h=%f", &cx, &cy, &w, &h);
            if (w < 0.03f) w = 0.08f;
            if (h < 0.03f) h = 0.08f;

            int cmd_sock = socket(AF_INET, SOCK_DGRAM, 0);
            struct sockaddr_in cmd_addr{};
            cmd_addr.sin_family = AF_INET;
            cmd_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
            cmd_addr.sin_port = htons(12349);
            char msg[256];
            std::snprintf(msg, sizeof(msg), "reset");
            sendto(cmd_sock, msg, std::strlen(msg), 0, (struct sockaddr*)&cmd_addr, sizeof(cmd_addr));
            usleep(50000);
            std::snprintf(msg, sizeof(msg), "init bbox_norm[%f,%f,%f,%f]", cx, cy, w, h);
            sendto(cmd_sock, msg, std::strlen(msg), 0, (struct sockaddr*)&cmd_addr, sizeof(cmd_addr));
            close(cmd_sock);

            fprintf(stderr, "[LightTrack/MC] cx=%.3f cy=%.3f w=%.3f h=%.3f\n", cx, cy, w, h);

            char reply[256];
            int rlen = std::snprintf(reply, sizeof(reply), "Captured! cx=%.3f cy=%.3f w=%.3f h=%.3f", cx, cy, w, h);
            std::string resp = "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\n" + std::string(reply);
            send(cfd, resp.c_str(), resp.size(), 0);
            close(cfd);
            continue;
        }

        std::string resp = "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nConnection: close\r\n\r\n";
        resp += page;
        send(cfd, resp.c_str(), resp.size(), 0);
        close(cfd);
    }
    close(srv);
}

int main(int argc, char** argv) {
    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);

    std::string camera = "/dev/cam_usb2";
    std::string model_dir = "models/lighttrack";
    std::string telem_host = "127.0.0.1";
    int telem_port = 12345;
    int cmd_port = 12349;
    std::string cam_name = "front";
    int frame_stride = 1;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--camera" && i + 1 < argc) camera = argv[++i];
        else if (a == "--models" && i + 1 < argc) model_dir = argv[++i];
        else if (a == "--telem-host" && i + 1 < argc) telem_host = argv[++i];
        else if (a == "--telem-port" && i + 1 < argc) telem_port = std::atoi(argv[++i]);
        else if (a == "--cmd-port" && i + 1 < argc) cmd_port = std::atoi(argv[++i]);
        else if (a == "--cam-name" && i + 1 < argc) cam_name = argv[++i];
        else if (a == "--viz-port" && i + 1 < argc) g_mjpeg_port = std::atoi(argv[++i]);
    }
    if (const char* e = std::getenv("LIGHTTRACK_MODELS_DIR")) model_dir = e;
    if (const char* e = std::getenv("LIGHTTRACK_CMD_PORT")) cmd_port = std::atoi(e);
    if (const char* e = std::getenv("VISION_TELEMETRY_HOST")) telem_host = e;
    if (const char* e = std::getenv("VISION_TELEMETRY_PORT")) telem_port = std::atoi(e);
    if (const char* e = std::getenv("LIGHTTRACK_CAM_NAME")) cam_name = e;
    if (const char* e = std::getenv("LIGHTTRACK_VIZ_PORT")) g_mjpeg_port = std::atoi(e);
    if (const char* e = std::getenv("LIGHTTRACK_FRAME_STRIDE")) frame_stride = std::max(1, std::atoi(e));
    g_stream_max_w = env_int("LIGHTTRACK_STREAM_MAX_W", "VISION_STREAM_MAX_W", 480, 0, 1920);

    LightTrack tracker;
    if (!tracker.load_models(model_dir)) {
        std::fprintf(stderr, "[LightTrack] model load failed from %s\n", model_dir.c_str());
        return 1;
    }
    std::printf("[LightTrack] Models OK: %s  cam=%s  telem=%s:%d  cmd=:%d  viz=:%d  stride=%d\n",
                model_dir.c_str(), camera.c_str(), telem_host.c_str(), telem_port, cmd_port, g_mjpeg_port, frame_stride);

    const std::string shm = shm_name_from_dev(camera);
    if (!shm_open_persistent(shm)) {
        std::fprintf(stderr, "[LightTrack] SHM failed %s — start orch_daemon first\n", shm.c_str());
        return 1;
    }

    cv::Mat probe;
    if (!shm_read(probe)) {
        std::fprintf(stderr, "[LightTrack] SHM empty %s\n", shm.c_str());
        shm_close_persistent();
        return 1;
    }

    int telem_sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in telem_addr{};
    telem_addr.sin_family = AF_INET;
    telem_addr.sin_port = htons(static_cast<uint16_t>(telem_port));
    inet_pton(AF_INET, telem_host.c_str(), &telem_addr.sin_addr);

    std::thread(cmd_thread, cmd_port).detach();
    std::thread(mjpeg_accept_thread).detach();
    std::thread(mjpeg_encoder_thread).detach();
    std::thread(manual_capture_thread).detach();

    int frame_id = 0;
    int lost_streak = 0;
    uint64_t last_shm_frame_id = 0;
    std::chrono::steady_clock::time_point init_hold_until{};
    float smooth_cx = 0.f, smooth_cy = 0.f, smooth_w = 0.f, smooth_h = 0.f, smooth_score = 0.f;
    bool smooth_init = false;
    float init_lock_w = 0.f, init_lock_h = 0.f;
    int init_size_hold_frames = 0;
    auto last_hb = std::chrono::steady_clock::now();
    static const int VEL_HIST = 5;
    float vel_hist_cx[VEL_HIST] = {0}, vel_hist_cy[VEL_HIST] = {0};
    int vel_idx = 0, vel_count = 0;
    float last_vx = 0, last_vy = 0;
    float last_ax = 0, last_ay = 0;
    float cur_speed = 0;
    int stable_frames = 0;
    float frozen_h = 0;
    bool size_frozen = false;
    float reacq_cx = 0, reacq_cy = 0, reacq_w = 0, reacq_h = 0;
    int reacq_attempts = 0;
    static const int MAX_REACQ = 8;
    int stat_frames_sec = 0;
    int stat_track_calls = 0;
    double stat_track_ms_sum = 0;
    double stat_track_ms_max = 0;
    float stat_fps = 0;
    float stat_track_ms_avg = 0;
    float stat_track_ms_max_v = 0;

    while (g_running) {
        uint64_t shm_fid = shm_peek_frame_id();
        if (shm_fid == 0) { usleep(5000); continue; }
        if (shm_fid == last_shm_frame_id && !g_need_init.load()) { usleep(1000); continue; }

        cv::Mat frame;
        if (!shm_read(frame, &shm_fid)) { usleep(1000); continue; }
        last_shm_frame_id = shm_fid;

        bool should_track = (frame_id % frame_stride == 0);

        if (g_need_full_reset.exchange(false)) {
            g_tracking = false;
            reacq_h = 0;
            reacq_attempts = MAX_REACQ;
            lost_streak = 0;
            smooth_init = false;
            smooth_cx = smooth_cy = smooth_w = smooth_h = smooth_score = 0;
            vel_count = 0;
            vel_idx = 0;
            last_vx = last_vy = last_ax = last_ay = 0;
            stable_frames = 0;
            size_frozen = false;
            frozen_h = 0;
            cur_speed = 0;
            fprintf(stderr, "[LightTrack] FULL RESET\n");
        }

        if (g_need_init.exchange(false)) {
            const int w = frame.cols, h = frame.rows;
            const float nh = std::max(0.025f, std::min(0.55f, g_init_h));
            const float nw = std::max(0.025f, std::min(0.55f, g_init_w > 0.01f ? g_init_w : nh));
            const float bh = nh * static_cast<float>(h);
            const float bw = nw * static_cast<float>(w);
            cv::Rect bb(static_cast<int>(g_init_cx * w - bw * 0.5f),
                        static_cast<int>(g_init_cy * h - bh * 0.5f),
                        static_cast<int>(bw), static_cast<int>(bh));
            bb &= cv::Rect(0, 0, w, h);
            if (bb.width < 24 || bb.height < 24) {
                g_tracking = false;
                std::fprintf(stderr, "[LightTrack] init rejected: bbox too small (%dx%d)\n", bb.width, bb.height);
                continue;
            }
            tracker.init(frame, bb);
            if (tracker.is_initialized()) {
                g_tracking = true;
                lost_streak = 0;
                stable_frames = 0;
                size_frozen = false;
                frozen_h = 0;
                vel_count = 0;
                vel_idx = 0;
                init_lock_w = nw;
                init_lock_h = nh;
                init_size_hold_frames = []() {
                    if (const char* e = std::getenv("LIGHTTRACK_INIT_SIZE_HOLD_FRAMES")) {
                        return std::max(0, std::atoi(e));
                    }
                    return 8;
                }();
                smooth_cx = g_init_cx;
                smooth_cy = g_init_cy;
                smooth_w = nw;
                smooth_h = nh;
                smooth_score = 1.0f;
                smooth_init = true;
                const int hold_ms = []() {
                    if (const char* e = std::getenv("LIGHTTRACK_HOLD_MS")) return std::max(0, std::atoi(e));
                    return 200;
                }();
                init_hold_until = std::chrono::steady_clock::now() + std::chrono::milliseconds(hold_ms);
                reacq_h = 0;
                // Fall through so MJPEG keeps updating during hold.
            } else {
                g_tracking = false;
                std::fprintf(stderr, "[LightTrack] init failed %d,%d %dx%d\n", bb.x, bb.y, bb.width, bb.height);
            }
        }

        if (!g_tracking && reacq_h > 0.01f && should_track) {
            if (reacq_cx > 0.001f && reacq_cy > 0.001f) {
                const float rw = reacq_w > 0.01f ? reacq_w : reacq_h;
                char hold[320];
                std::snprintf(hold, sizeof(hold),
                    "{\"cam\":\"%s\",\"tracker\":\"lighttrack\",\"bbox_norm\":[%.4f,%.4f,%.4f,%.4f],"
                    "\"class_id\":0,\"conf\":%.3f}",
                    cam_name.c_str(), reacq_cx, reacq_cy, rw, reacq_h, 0.15f);
                sendto(telem_sock, hold, std::strlen(hold), 0,
                       reinterpret_cast<struct sockaddr*>(&telem_addr), sizeof(telem_addr));
            }
        } else if (should_track && g_tracking && tracker.is_initialized()) {
            const auto now_tr = std::chrono::steady_clock::now();
            if (now_tr >= init_hold_until) {
            auto t0 = std::chrono::steady_clock::now();
            tracker.track(frame);
            auto t1 = std::chrono::steady_clock::now();
            double track_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
            stat_track_calls++;
            stat_track_ms_sum += track_ms;
            if (track_ms > stat_track_ms_max) stat_track_ms_max = track_ms;

            float cx = tracker.state.target_pos.x / static_cast<float>(frame.cols);
            float cy = tracker.state.target_pos.y / static_cast<float>(frame.rows);
            float tw = tracker.state.target_sz.x / static_cast<float>(frame.cols);
            const float th = tracker.state.target_sz.y / static_cast<float>(frame.rows);
            if (tw <= 0.01f) tw = th;
            const bool size_locked = init_size_hold_frames > 0;
            if (size_locked) {
                init_size_hold_frames--;
                tw = init_lock_w > 0.01f ? init_lock_w : tw;
                const float hold_h = init_lock_h > 0.01f ? init_lock_h : th;
                if (!smooth_init) {
                    smooth_cx = cx;
                    smooth_cy = cy;
                    smooth_w = tw;
                    smooth_h = hold_h;
                    smooth_init = true;
                } else {
                    smooth_w = tw;
                    smooth_h = hold_h;
                }
            }
            const float sc = tracker.state.cls_score_max;
            const float peak_margin = tracker.state.peak_margin;
            const float lost_sc = lost_score_threshold();
            const int lost_n = lost_frames_threshold();
            const bool weak = (sc < lost_sc);

            if (weak) {
                lost_streak++;
                if (lost_streak >= lost_n) {
                    reacq_cx = smooth_cx > 0.001f ? smooth_cx : cx;
                    reacq_cy = smooth_cy > 0.001f ? smooth_cy : cy;
                    reacq_w = smooth_w > 0.01f ? smooth_w : tw;
                    reacq_h = smooth_h > 0.01f ? smooth_h : th;
                    g_tracking = false;
                    smooth_init = false;
                    lost_streak = 0;
                    fprintf(stderr, "[LightTrack] LOST: score=%.3f margin=%.3f; waiting for validated init\n",
                            sc, peak_margin);
                    char msg[128];
                    std::snprintf(msg, sizeof(msg), "{\"cam\":\"%s\",\"tracker\":\"lighttrack\",\"lost\":true}", cam_name.c_str());
                    sendto(telem_sock, msg, std::strlen(msg), 0, reinterpret_cast<struct sockaddr*>(&telem_addr), sizeof(telem_addr));
                } else if (smooth_init) {
                    cx = smooth_cx;
                    cy = smooth_cy;
                }
            } else {
                lost_streak = std::max(0, lost_streak - 3);
            }

            if (g_tracking) {
            float alpha = 0.15f;
            if (sc >= 0.45f) {
                const float output_motion = smooth_init
                    ? std::hypot(cx - smooth_cx, cy - smooth_cy) : 0.0f;
                alpha = output_motion > 0.02f ? 0.70f : 0.45f;
            } else if (sc >= 0.30f) {
                alpha = 0.30f;
            } else if (weak) {
                alpha = 0.0f;
            }
            if (smooth_init) {
                smooth_cx = smooth_cx * (1.0f - alpha) + cx * alpha;
                smooth_cy = smooth_cy * (1.0f - alpha) + cy * alpha;
            } else {
                smooth_cx = cx;
                smooth_cy = cy;
            }
            cx = smooth_cx;
            cy = smooth_cy;

            smooth_score = weak ? smooth_score * 0.9f + sc * 0.1f : sc;
            smooth_init = true;
            if (!size_locked && !weak && sc >= 0.30f) {
                smooth_w = smooth_w > 0.01f ? smooth_w * 0.88f + tw * 0.12f : tw;
                smooth_h = smooth_h > 0.01f ? smooth_h * 0.88f + th * 0.12f : th;
            }
            if (smooth_w <= 0.01f) smooth_w = smooth_h;
            if (init_lock_w > 0.01f) {
                smooth_w = std::clamp(smooth_w, init_lock_w * 0.55f, init_lock_w * 2.2f);
            }
            if (init_lock_h > 0.01f) {
                smooth_h = std::clamp(smooth_h, init_lock_h * 0.55f, init_lock_h * 2.2f);
            }

            const float pub_conf = weak ? std::min(sc, 0.20f) : sc;
            char msg[320];
            std::snprintf(msg, sizeof(msg),
                "{\"cam\":\"%s\",\"tracker\":\"lighttrack\",\"bbox_norm\":[%.4f,%.4f,%.4f,%.4f],"
                "\"class_id\":0,\"conf\":%.3f,\"peak_margin\":%.3f}",
                cam_name.c_str(), cx, cy, smooth_w, smooth_h, pub_conf, peak_margin);
            sendto(telem_sock, msg, std::strlen(msg), 0,
                   reinterpret_cast<struct sockaddr*>(&telem_addr), sizeof(telem_addr));
            } // g_tracking
            } // init_hold done
        }

        {
            extern std::atomic<int> g_mjpeg_clients;
            if (g_mjpeg_clients.load() > 0) {
                cv::Mat viz;
                if (g_stream_max_w > 0 && frame.cols > g_stream_max_w) {
                    const int nw = g_stream_max_w;
                    const int nh = std::max(1, (int)std::lround(
                        frame.rows * (double)nw / (double)frame.cols));
                    cv::resize(frame, viz, cv::Size(nw, nh), 0, 0, cv::INTER_LINEAR);
                } else {
                    viz = frame.clone();
                }
                if (g_tracking.load() && smooth_init) {
                    draw_grid(viz, smooth_cx, smooth_cy, smooth_h, smooth_score);
                    draw_tracking(viz, smooth_cx, smooth_cy, smooth_w, smooth_h, smooth_score);
                } else {
                    draw_grid(viz);
                }
                char st[128];
                snprintf(st, sizeof(st), "FPS:%.0f  track:%.1fms avg %.1fmax",
                         stat_fps, stat_track_ms_avg, stat_track_ms_max_v);
                int baseline = 0;
                cv::Size sz = cv::getTextSize(st, cv::FONT_HERSHEY_SIMPLEX, 0.45, 1, &baseline);
                cv::rectangle(viz, cv::Point(4, viz.rows - 26), cv::Point(8 + sz.width, viz.rows - 6), cv::Scalar(0, 0, 0), -1);
                cv::putText(viz, st, cv::Point(6, viz.rows - 10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
                std::lock_guard<std::mutex> lk(g_viz_mtx);
                g_viz_frame = std::move(viz);
                g_viz_new.store(true, std::memory_order_release);
            }
        }

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_hb).count() >= 1) {
            int sec = (int)std::chrono::duration_cast<std::chrono::seconds>(now - last_hb).count();
            stat_fps = (float)stat_frames_sec / std::max(1, sec);
            stat_frames_sec = 0;
            stat_track_ms_avg = stat_track_calls > 0 ? (float)(stat_track_ms_sum / stat_track_calls) : 0;
            stat_track_ms_max_v = (float)stat_track_ms_max;
            fprintf(stderr,
                    "[LightTrack] FPS=%.1f track_ms=%.1f/%.1f frames=%d tracking=%d "
                    "cx=%.4f cy=%.4f h=%.4f score=%.3f\n",
                    stat_fps, stat_track_ms_avg, stat_track_ms_max_v, stat_track_calls,
                    (int)g_tracking.load(), smooth_cx, smooth_cy, smooth_h, smooth_score);
            stat_track_calls = 0;
            stat_track_ms_sum = 0;
            stat_track_ms_max = 0;
            char hb[384];
            std::snprintf(hb, sizeof(hb),
                "{\"tracker\":\"lighttrack\",\"cam\":\"%s\",\"alive\":true,\"tracking\":%s,"
                "\"fps\":%.1f,\"track_ms_avg\":%.1f,\"track_ms_max\":%.1f,"
                "\"cx\":%.4f,\"cy\":%.4f,\"h\":%.4f,\"score\":%.3f,\"lost_streak\":%d}",
                cam_name.c_str(), g_tracking.load() ? "true" : "false",
                stat_fps, stat_track_ms_avg, stat_track_ms_max_v,
                smooth_cx, smooth_cy, smooth_h, smooth_score, lost_streak);
            sendto(telem_sock, hb, std::strlen(hb), 0,
                   reinterpret_cast<struct sockaddr*>(&telem_addr), sizeof(telem_addr));
            last_hb = now;
        }

        frame_id++;
        stat_frames_sec++;
    }

    tracker.release();
    shm_close_persistent();
    close(telem_sock);
    std::printf("[LightTrack] stopped (%d frames)\n", frame_id);
    return 0;
}
