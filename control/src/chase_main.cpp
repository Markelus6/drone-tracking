/** chase_fc — ELRS CRSF → MSP SET_RAW_RC + LightTrack engage + cam servo + PID chase. */
#include "chase_config.hpp"
#include "crsf.hpp"
#include "msp.hpp"
#include "pid.hpp"
#include "servo_pwm.hpp"
#include "uart.hpp"

#include <arpa/inet.h>
#include <atomic>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <netinet/in.h>
#include <signal.h>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

static std::atomic<bool> g_run{true};
static void on_sig(int) { g_run = false; }

static uint64_t now_ms() {
  using namespace std::chrono;
  return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

static void udp_send(const std::string& host, int port, const std::string& msg) {
  int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) return;
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  inet_pton(AF_INET, host.c_str(), &addr.sin_addr);
  sendto(fd, msg.data(), msg.size(), 0, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
  ::close(fd);
}

struct TelemState {
  std::atomic<bool> have{false};
  std::atomic<bool> lost{false};
  std::atomic<double> cx{0.5};
  std::atomic<double> cy{0.5};
  std::atomic<uint64_t> last_ms{0};
};

static void forward_udp(const char* data, size_t n, int port) {
  int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) return;
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
  sendto(fd, data, n, 0, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
  ::close(fd);
}

static void telem_thread(int port, int forward_a, int forward_b, TelemState* st) {
  int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) return;
  int yes = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  addr.sin_addr.s_addr = INADDR_ANY;
  if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    std::fprintf(stderr, "[chase] telem bind :%d failed\n", port);
    ::close(fd);
    return;
  }
  char buf[2048];
  while (g_run) {
    sockaddr_in src{};
    socklen_t sl = sizeof(src);
    ssize_t n = recvfrom(fd, buf, sizeof(buf) - 1, 0, reinterpret_cast<sockaddr*>(&src), &sl);
    if (n <= 0) continue;
    if (forward_a > 0) forward_udp(buf, static_cast<size_t>(n), forward_a);
    if (forward_b > 0) forward_udp(buf, static_cast<size_t>(n), forward_b);
    buf[n] = 0;
    st->last_ms = now_ms();
    if (std::strstr(buf, "\"lost\":true") || std::strstr(buf, "\"lost\": true")) {
      st->lost = true;
      continue;
    }
    const char* p = std::strstr(buf, "bbox_norm");
    if (!p) continue;
    p = std::strchr(p, '[');
    if (!p) continue;
    double cx, cy, w, h;
    if (std::sscanf(p, "[%lf ,%lf ,%lf ,%lf]", &cx, &cy, &w, &h) == 4 ||
        std::sscanf(p, "[%lf,%lf,%lf,%lf]", &cx, &cy, &w, &h) == 4) {
      st->cx = cx;
      st->cy = cy;
      st->lost = false;
      st->have = true;
    }
  }
  ::close(fd);
}

static int clamp_us(int v) {
  if (v < 1000) return 1000;
  if (v > 2000) return 2000;
  return v;
}

int main(int argc, char** argv) {
  std::string cfg_path = "deploy/chase.json";
  bool dry = false;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--config") == 0 && i + 1 < argc) cfg_path = argv[++i];
    else if (std::strcmp(argv[i], "--dry-run") == 0) dry = true;
    else if (std::strcmp(argv[i], "-h") == 0 || std::strcmp(argv[i], "--help") == 0) {
      std::printf("Usage: chase_fc [--config deploy/chase.json] [--dry-run]\n");
      return 0;
    }
  }

  ChaseConfig cfg;
  std::string err;
  if (!load_chase_config(cfg_path, cfg, &err)) {
    std::fprintf(stderr, "[chase] config: %s — using defaults\n", err.c_str());
  }

  signal(SIGINT, on_sig);
  signal(SIGTERM, on_sig);

  TelemState telem;
  // :12345 chase; fan-out :12346 stats, :12350 osd_overlay
  const int fwd_stats = cfg.track_telem_port + 1;
  const int fwd_osd = cfg.track_telem_port + 5;
  std::thread th(telem_thread, cfg.track_telem_port, fwd_stats, fwd_osd, &telem);
  th.detach();
  std::printf("[chase] telem :%d → forward :%d (stats) :%d (osd)\n", cfg.track_telem_port,
              fwd_stats, fwd_osd);

  std::string uerr;
  int crsf_fd = -1;
  if (!dry) {
    crsf_fd = uart_open(cfg.crsf_uart, cfg.crsf_baud, &uerr);
    if (crsf_fd < 0) {
      std::fprintf(stderr, "[chase] CRSF UART %s\n", uerr.c_str());
      return 1;
    }
  }

  msp::Tx msp;
  if (!dry) {
    if (!msp.open(cfg.msp_uart, cfg.msp_baud)) {
      std::fprintf(stderr, "[chase] MSP open failed %s @ %d\n", cfg.msp_uart.c_str(), cfg.msp_baud);
      uart_close(crsf_fd);
      return 1;
    }
  }

  ServoPwm servo;
  if (!dry && !cfg.servo_pwmchip.empty()) {
    if (!servo.open(cfg.servo_pwmchip, cfg.servo_channel, cfg.servo_period_ns)) {
      std::fprintf(stderr, "[chase] WARN servo open failed (%s)\n", cfg.servo_pwmchip.c_str());
    } else {
      servo.set_pulse_us(cfg.servo_idle_us);
      servo.set_target_us(cfg.servo_idle_us, 1);
    }
  }

  Pid pid_yaw, pid_pitch;
  pid_yaw.kp = cfg.pid_yaw_kp;
  pid_yaw.ki = cfg.pid_yaw_ki;
  pid_yaw.kd = cfg.pid_yaw_kd;
  pid_yaw.out_min = -cfg.yaw_limit_us;
  pid_yaw.out_max = cfg.yaw_limit_us;
  pid_pitch.kp = cfg.pid_pitch_kp;
  pid_pitch.ki = cfg.pid_pitch_ki;
  pid_pitch.kd = cfg.pid_pitch_kd;
  pid_pitch.out_min = -cfg.pitch_limit_us;
  pid_pitch.out_max = cfg.pitch_limit_us;

  crsf::Parser parser;
  bool engaged = false;
  uint64_t engage_at_ms = 0;  // PID starts after servo ramp
  uint64_t last_osd_ms = 0;
  uint16_t last_ch[16];
  for (int i = 0; i < 16; ++i) last_ch[i] = 1500;
  last_ch[2] = 1000;  // throttle low

  const int period_ms = std::max(5, 1000 / std::max(1, cfg.msp_hz));
  auto t_prev = now_ms();

  std::printf("[chase] CRSF %s@%d → MSP %s@%d engage CH%d servo=%s\n",
              cfg.crsf_uart.c_str(), cfg.crsf_baud, cfg.msp_uart.c_str(), cfg.msp_baud,
              cfg.engage_ch, cfg.servo_pwmchip.empty() ? "off" : cfg.servo_pwmchip.c_str());
  std::printf("[chase] OSD box via MSP DisplayPort on FC UART (no Pi→VTX video)\n");
  std::printf("[chase] Flow: aim target in center frame → flip CH%d high → lock → servo nose → chase\n",
              cfg.engage_ch);

  uint8_t rbuf[512];
  while (g_run) {
    if (crsf_fd >= 0) {
      ssize_t n = ::read(crsf_fd, rbuf, sizeof(rbuf));
      if (n > 0) parser.feed(rbuf, static_cast<size_t>(n));
    }

    const int* ch = parser.channels();
    const uint64_t t = now_ms();
    const bool rc_ok = parser.last_rc_ms() > 0 &&
                       (t - parser.last_rc_ms()) < static_cast<uint64_t>(cfg.failsafe_ms);

    if (rc_ok) {
      for (int i = 0; i < 16; ++i) last_ch[i] = static_cast<uint16_t>(clamp_us(ch[i]));
    }

    const int aux_i = cfg.engage_ch - 1;
    int aux = (aux_i >= 0 && aux_i < 16) ? static_cast<int>(last_ch[aux_i]) : 1500;

    // Rising edge CH8: 1) LightTrack init (захват) 2) servo nose 3) chase after ramp
    if (rc_ok && aux >= cfg.engage_high_us && !engaged) {
      engaged = true;
      engage_at_ms = t + static_cast<uint64_t>(std::max(0, cfg.servo_ramp_ms));
      pid_yaw.reset();
      pid_pitch.reset();
      char msg[160];
      std::snprintf(msg, sizeof(msg),
                    "{\"cmd\":\"init\",\"bbox_norm\":[%.4f,%.4f,%.4f,%.4f]}",
                    cfg.init_bbox[0], cfg.init_bbox[1], cfg.init_bbox[2], cfg.init_bbox[3]);
      udp_send(cfg.track_cmd_host, cfg.track_cmd_port, msg);
      if (servo.ok()) servo.set_target_us(cfg.servo_nose_us, cfg.servo_ramp_ms);
      std::printf("[chase] LOCK+SERVO then CHASE @+%dms: %s\n", cfg.servo_ramp_ms, msg);
    } else if (rc_ok && aux <= cfg.engage_low_us && engaged) {
      engaged = false;
      engage_at_ms = 0;
      udp_send(cfg.track_cmd_host, cfg.track_cmd_port, "{\"cmd\":\"reset\"}");
      if (servo.ok()) servo.set_target_us(cfg.servo_idle_us, cfg.servo_ramp_ms);
      if (!dry) msp.send_osd_clear();
      std::printf("[chase] DISENGAGE\n");
    }

    uint16_t out[16];
    for (int i = 0; i < 16; ++i) out[i] = last_ch[i];

    const int dt = static_cast<int>(t - t_prev);
    t_prev = t;
    if (servo.ok()) servo.tick(dt > 0 ? dt : period_ms);

    const bool chase_active = engaged && rc_ok && t >= engage_at_ms;
    if (chase_active) {
      bool use_pid = telem.have.load() && !telem.lost.load();
      if (use_pid) {
        double dt_s = dt > 0 ? dt / 1000.0 : period_ms / 1000.0;
        double err_x = telem.cx.load() - 0.5;
        double err_y = telem.cy.load() - 0.5;
        int dyaw = static_cast<int>(pid_yaw.step(err_x, dt_s));
        int dpitch = static_cast<int>(pid_pitch.step(-err_y, dt_s));
        out[3] = static_cast<uint16_t>(clamp_us(static_cast<int>(out[3]) + dyaw));
        out[1] = static_cast<uint16_t>(clamp_us(static_cast<int>(out[1]) + dpitch));
      } else if (!cfg.lost_passthrough) {
        out[0] = 1500;
        out[1] = 1500;
        out[3] = 1500;
      }
    }

    // MSP DisplayPort OSD on FC → mixed into analog before VTX (operator goggles)
    if (!dry && msp.ok() && (t - last_osd_ms) >= 100) {
      last_osd_ms = t;
      if (engaged && telem.have.load() && !telem.lost.load()) {
        msp.send_osd_bbox(telem.cx.load(), telem.cy.load(), 0.18, 0.18);
      } else {
        // Aim reticle: operator centers target here, then flips CH8
        msp.send_osd_aim(cfg.init_bbox[2], cfg.init_bbox[3]);
      }
    }

    if (!rc_ok) {
      std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
      continue;
    }

    if (!dry) msp.send_raw_rc(out);
    std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
  }

  if (engaged) udp_send(cfg.track_cmd_host, cfg.track_cmd_port, "{\"cmd\":\"reset\"}");
  if (servo.ok()) {
    servo.set_target_us(cfg.servo_idle_us, cfg.servo_ramp_ms);
    for (int i = 0; i < cfg.servo_ramp_ms; i += 20) {
      servo.tick(20);
      usleep(20000);
    }
    servo.close();
  }
  msp.close();
  uart_close(crsf_fd);
  std::printf("[chase] exit\n");
  return 0;
}
