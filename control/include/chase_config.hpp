#pragma once
#include <cstdint>
#include <string>
#include <vector>

struct ChaseConfig {
  std::string crsf_uart = "/dev/ttyS1";
  int crsf_baud = 420000;
  std::string msp_uart = "/dev/ttyS3";
  int msp_baud = 115200;
  int engage_ch = 8;
  int engage_high_us = 1800;
  int engage_low_us = 1200;
  std::string servo_pwmchip;
  int servo_channel = 0;
  int servo_idle_us = 1250;
  int servo_nose_us = 2000;
  int servo_ramp_ms = 300;
  int servo_period_ns = 20000000;
  double pid_yaw_kp = 350, pid_yaw_ki = 0, pid_yaw_kd = 25;
  double pid_pitch_kp = 350, pid_pitch_ki = 0, pid_pitch_kd = 25;
  int yaw_limit_us = 200;
  int pitch_limit_us = 200;
  std::string track_cmd_host = "127.0.0.1";
  int track_cmd_port = 12349;
  int track_telem_port = 12345;  // chase binds here; forwards to port+1
  double init_bbox[4] = {0.5, 0.5, 0.15, 0.15};
  int msp_hz = 100;
  int failsafe_ms = 1000;
  bool lost_passthrough = true;
};

bool load_chase_config(const std::string& path, ChaseConfig& out, std::string* err = nullptr);
