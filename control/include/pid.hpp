#pragma once

struct Pid {
  double kp = 0, ki = 0, kd = 0;
  double i = 0;
  double prev_e = 0;
  double out_min = -200, out_max = 200;

  void reset() {
    i = 0;
    prev_e = 0;
  }

  double step(double error, double dt_s) {
    if (dt_s <= 0) dt_s = 0.01;
    i += error * dt_s;
    double d = (error - prev_e) / dt_s;
    prev_e = error;
    double u = kp * error + ki * i + kd * d;
    if (u < out_min) u = out_min;
    if (u > out_max) u = out_max;
    return u;
  }
};
