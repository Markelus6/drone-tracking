#include "chase_config.hpp"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

static bool extract_string(const std::string& j, const char* key, std::string& out) {
  std::string pat = std::string("\"") + key + "\"";
  auto p = j.find(pat);
  if (p == std::string::npos) return false;
  p = j.find(':', p);
  if (p == std::string::npos) return false;
  p = j.find('"', p);
  if (p == std::string::npos) return false;
  auto q = j.find('"', p + 1);
  if (q == std::string::npos) return false;
  out = j.substr(p + 1, q - p - 1);
  return true;
}

static bool extract_number(const std::string& j, const char* key, double& out) {
  std::string pat = std::string("\"") + key + "\"";
  auto p = j.find(pat);
  if (p == std::string::npos) return false;
  p = j.find(':', p);
  if (p == std::string::npos) return false;
  ++p;
  while (p < j.size() && (j[p] == ' ' || j[p] == '\t')) ++p;
  try {
    size_t n = 0;
    out = std::stod(j.substr(p), &n);
    return n > 0;
  } catch (...) {
    return false;
  }
}

static bool extract_bool(const std::string& j, const char* key, bool& out) {
  std::string pat = std::string("\"") + key + "\"";
  auto p = j.find(pat);
  if (p == std::string::npos) return false;
  p = j.find(':', p);
  if (p == std::string::npos) return false;
  auto t = j.find("true", p);
  auto f = j.find("false", p);
  if (t != std::string::npos && (f == std::string::npos || t < f)) {
    out = true;
    return true;
  }
  if (f != std::string::npos) {
    out = false;
    return true;
  }
  return false;
}

bool load_chase_config(const std::string& path, ChaseConfig& cfg, std::string* err) {
  std::ifstream in(path);
  if (!in) {
    if (err) *err = "cannot open " + path;
    return false;
  }
  std::ostringstream ss;
  ss << in.rdbuf();
  const std::string j = ss.str();

  std::string s;
  double d;
  bool b;
  if (extract_string(j, "crsf_uart", s)) cfg.crsf_uart = s;
  if (extract_number(j, "crsf_baud", d)) cfg.crsf_baud = static_cast<int>(d);
  if (extract_string(j, "msp_uart", s)) cfg.msp_uart = s;
  if (extract_number(j, "msp_baud", d)) cfg.msp_baud = static_cast<int>(d);
  if (extract_number(j, "engage_ch", d)) cfg.engage_ch = static_cast<int>(d);
  if (extract_number(j, "engage_high_us", d)) cfg.engage_high_us = static_cast<int>(d);
  if (extract_number(j, "engage_low_us", d)) cfg.engage_low_us = static_cast<int>(d);
  if (extract_string(j, "servo_pwmchip", s)) cfg.servo_pwmchip = s;
  if (extract_number(j, "servo_channel", d)) cfg.servo_channel = static_cast<int>(d);
  if (extract_number(j, "servo_idle_us", d)) cfg.servo_idle_us = static_cast<int>(d);
  if (extract_number(j, "servo_nose_us", d)) cfg.servo_nose_us = static_cast<int>(d);
  if (extract_number(j, "servo_ramp_ms", d)) cfg.servo_ramp_ms = static_cast<int>(d);
  if (extract_number(j, "servo_period_ns", d)) cfg.servo_period_ns = static_cast<int>(d);
  if (extract_number(j, "pid_yaw_kp", d)) cfg.pid_yaw_kp = d;
  if (extract_number(j, "pid_yaw_ki", d)) cfg.pid_yaw_ki = d;
  if (extract_number(j, "pid_yaw_kd", d)) cfg.pid_yaw_kd = d;
  if (extract_number(j, "pid_pitch_kp", d)) cfg.pid_pitch_kp = d;
  if (extract_number(j, "pid_pitch_ki", d)) cfg.pid_pitch_ki = d;
  if (extract_number(j, "pid_pitch_kd", d)) cfg.pid_pitch_kd = d;
  if (extract_number(j, "yaw_limit_us", d)) cfg.yaw_limit_us = static_cast<int>(d);
  if (extract_number(j, "pitch_limit_us", d)) cfg.pitch_limit_us = static_cast<int>(d);
  if (extract_string(j, "track_cmd_host", s)) cfg.track_cmd_host = s;
  if (extract_number(j, "track_cmd_port", d)) cfg.track_cmd_port = static_cast<int>(d);
  if (extract_number(j, "track_telem_port", d)) cfg.track_telem_port = static_cast<int>(d);
  if (extract_number(j, "msp_hz", d)) cfg.msp_hz = static_cast<int>(d);
  if (extract_number(j, "failsafe_ms", d)) cfg.failsafe_ms = static_cast<int>(d);
  if (extract_bool(j, "lost_passthrough", b)) cfg.lost_passthrough = b;

  auto bp = j.find("\"init_bbox\"");
  if (bp != std::string::npos) {
    auto lb = j.find('[', bp);
    auto rb = j.find(']', lb);
    if (lb != std::string::npos && rb != std::string::npos) {
      double a, b2, c, d2;
      if (std::sscanf(j.c_str() + lb, "[%lf ,%lf ,%lf ,%lf]", &a, &b2, &c, &d2) == 4 ||
          std::sscanf(j.c_str() + lb, "[%lf,%lf,%lf,%lf]", &a, &b2, &c, &d2) == 4) {
        cfg.init_bbox[0] = a;
        cfg.init_bbox[1] = b2;
        cfg.init_bbox[2] = c;
        cfg.init_bbox[3] = d2;
      }
    }
  }
  return true;
}
