/** osd_overlay — SHM frame + LightTrack bbox → V4L2 sink (USB CVBS / loopback → VTX). */
#include "camera_orchestrator.hpp"

#include <arpa/inet.h>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <opencv2/opencv.hpp>
#include <signal.h>
#include <string>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>

static std::atomic<bool> g_run{true};
static void* g_shm = nullptr;
static size_t g_shm_size = 0;

static void on_sig(int) { g_run = false; }

static std::string shm_path_from_dev(const std::string& device) {
  std::string p = "/dev/shm/drone_cam";
  for (char c : device) p += (c == '/') ? '_' : c;
  return p;
}

static bool shm_open_ro(const std::string& device) {
  std::string path = shm_path_from_dev(device);
  int fd = open(path.c_str(), O_RDONLY);
  if (fd < 0) return false;
  struct stat st {};
  if (fstat(fd, &st) != 0) {
    close(fd);
    return false;
  }
  g_shm_size = static_cast<size_t>(st.st_size);
  g_shm = mmap(nullptr, g_shm_size, PROT_READ, MAP_SHARED, fd, 0);
  close(fd);
  return g_shm && g_shm != MAP_FAILED;
}

static bool shm_read(cv::Mat& frame, uint64_t* fid) {
  if (!g_shm) return false;
  const ShmHeader* hdr = static_cast<const ShmHeader*>(g_shm);
  __sync_synchronize();
  if (!hdr->ready || hdr->width == 0 || hdr->height == 0) return false;
  if (fid) *fid = hdr->frame_id;
  const uint8_t* data = static_cast<const uint8_t*>(g_shm) + sizeof(ShmHeader);
  cv::Mat(hdr->height, hdr->width, hdr->channels == 1 ? CV_8UC1 : CV_8UC3,
          const_cast<uint8_t*>(data))
      .copyTo(frame);
  return true;
}

struct Bbox {
  std::atomic<bool> have{false};
  std::atomic<bool> lost{false};
  std::atomic<double> cx{0.5}, cy{0.5}, w{0.15}, h{0.15};
};

static void telem_loop(int port, Bbox* b) {
  int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) return;
  int yes = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  addr.sin_addr.s_addr = INADDR_ANY;
  if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    close(fd);
    return;
  }
  char buf[2048];
  while (g_run) {
    ssize_t n = recvfrom(fd, buf, sizeof(buf) - 1, 0, nullptr, nullptr);
    if (n <= 0) continue;
    buf[n] = 0;
    if (std::strstr(buf, "\"lost\":true") || std::strstr(buf, "\"lost\": true")) {
      b->lost = true;
      continue;
    }
    const char* p = std::strstr(buf, "bbox_norm");
    if (!p) continue;
    p = std::strchr(p, '[');
    if (!p) continue;
    double cx, cy, w, h;
    if (std::sscanf(p, "[%lf,%lf,%lf,%lf]", &cx, &cy, &w, &h) == 4 ||
        std::sscanf(p, "[%lf ,%lf ,%lf ,%lf]", &cx, &cy, &w, &h) == 4) {
      b->cx = cx;
      b->cy = cy;
      b->w = w;
      b->h = h;
      b->lost = false;
      b->have = true;
    }
  }
  close(fd);
}

int main(int argc, char** argv) {
  std::string camera = "/dev/cam_usb2";
  std::string sink;
  int telem_port = 12350;  // chase_fc forwards LightTrack telem here (primary :12345)
  int fps = 30;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--camera") == 0 && i + 1 < argc) camera = argv[++i];
    else if (std::strcmp(argv[i], "--sink") == 0 && i + 1 < argc) sink = argv[++i];
    else if (std::strcmp(argv[i], "--telem-port") == 0 && i + 1 < argc)
      telem_port = std::atoi(argv[++i]);
    else if (std::strcmp(argv[i], "--fps") == 0 && i + 1 < argc) fps = std::atoi(argv[++i]);
    else if (std::strcmp(argv[i], "-h") == 0 || std::strcmp(argv[i], "--help") == 0) {
      std::printf(
          "Usage: osd_overlay --camera /dev/cam_usb2 --sink /dev/video10 [--telem-port 12350]\n"
          "  Reads SHM frames, draws LightTrack bbox, writes to V4L2 sink (CVBS encoder / loopback).\n"
          "  Default telem port 12350 (chase_fc forwards from :12345).\n");
      return 0;
    }
  }

  signal(SIGINT, on_sig);
  signal(SIGTERM, on_sig);

  if (!shm_open_ro(camera)) {
    std::fprintf(stderr, "[osd] SHM open failed for %s (%s)\n", camera.c_str(),
                 shm_path_from_dev(camera).c_str());
    return 1;
  }

  Bbox bbox;
  std::thread(telem_loop, telem_port, &bbox).detach();

  cv::VideoWriter writer;
  bool sink_ok = false;
  int w = 0, h = 0;
  uint64_t last_fid = UINT64_MAX;

  std::printf("[osd] camera=%s sink=%s telem=:%d\n", camera.c_str(),
              sink.empty() ? "(none)" : sink.c_str(), telem_port);

  while (g_run) {
    cv::Mat frame;
    uint64_t fid = 0;
    if (!shm_read(frame, &fid) || frame.empty()) {
      usleep(5000);
      continue;
    }
    if (fid == last_fid) {
      usleep(1000);
      continue;
    }
    last_fid = fid;
    if (frame.channels() == 1) cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);

    if (bbox.have.load() && !bbox.lost.load()) {
      int W = frame.cols, H = frame.rows;
      double cx = bbox.cx.load(), cy = bbox.cy.load(), bw = bbox.w.load(), bh = bbox.h.load();
      int x1 = static_cast<int>((cx - bw / 2) * W);
      int y1 = static_cast<int>((cy - bh / 2) * H);
      int x2 = static_cast<int>((cx + bw / 2) * W);
      int y2 = static_cast<int>((cy + bh / 2) * H);
      cv::rectangle(frame, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);
      cv::drawMarker(frame, cv::Point(static_cast<int>(cx * W), static_cast<int>(cy * H)),
                     cv::Scalar(0, 255, 255), cv::MARKER_CROSS, 20, 2);
    } else if (bbox.lost.load()) {
      cv::putText(frame, "LOST", cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                  cv::Scalar(0, 0, 255), 2);
    }
    // center crosshair always
    cv::drawMarker(frame, cv::Point(frame.cols / 2, frame.rows / 2), cv::Scalar(255, 255, 255),
                   cv::MARKER_CROSS, 12, 1);

    if (!sink.empty()) {
      if (!sink_ok || frame.cols != w || frame.rows != h) {
        writer.release();
        w = frame.cols;
        h = frame.rows;
        // V4L2 raw BGR3; many USB CVBS devices want YUYV — try MJPG/raw
        sink_ok = writer.open(sink, cv::CAP_V4L2, cv::VideoWriter::fourcc('B', 'G', 'R', '3'), fps,
                              cv::Size(w, h));
        if (!sink_ok) {
          sink_ok = writer.open(sink, cv::CAP_V4L2, 0, fps, cv::Size(w, h));
        }
        if (!sink_ok) {
          std::fprintf(stderr, "[osd] WARN cannot open sink %s\n", sink.c_str());
        } else {
          std::printf("[osd] sink open %s %dx%d@%d\n", sink.c_str(), w, h, fps);
        }
      }
      if (sink_ok) writer.write(frame);
    }
  }

  writer.release();
  if (g_shm && g_shm != MAP_FAILED) munmap(g_shm, g_shm_size);
  return 0;
}
