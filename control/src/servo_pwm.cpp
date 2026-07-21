#include "servo_pwm.hpp"

#include <cmath>
#include <fstream>
#include <sstream>

static bool write_file(const std::string& path, const std::string& val) {
  std::ofstream out(path);
  if (!out) return false;
  out << val;
  return static_cast<bool>(out);
}

bool ServoPwm::open(const std::string& pwmchip_path, int channel, int period_ns) {
  close();
  if (pwmchip_path.empty()) return false;
  chip_ = pwmchip_path;
  while (!chip_.empty() && chip_.back() == '/') chip_.pop_back();
  ch_ = channel;
  period_ns_ = period_ns;

  // export (ignore if busy)
  write_file(chip_ + "/export", std::to_string(ch_));
  std::ostringstream base;
  base << chip_ << "/pwm" << ch_;
  const std::string pwm = base.str();

  if (!write_file(pwm + "/period", std::to_string(period_ns_))) return false;
  if (!write_file(pwm + "/duty_cycle", "1500000")) return false;
  if (!write_file(pwm + "/enable", "1")) return false;
  exported_ = true;
  current_us_ = 1500;
  target_us_ = 1500;
  return true;
}

void ServoPwm::close() {
  if (!exported_) return;
  std::ostringstream base;
  base << chip_ << "/pwm" << ch_;
  write_file(base.str() + "/enable", "0");
  write_file(chip_ + "/unexport", std::to_string(ch_));
  exported_ = false;
}

bool ServoPwm::set_pulse_us(int us) {
  if (!exported_) return false;
  if (us < 800) us = 800;
  if (us > 2500) us = 2500;
  long long duty = static_cast<long long>(us) * 1000LL;  // us → ns
  if (duty >= period_ns_) duty = period_ns_ - 1;
  std::ostringstream base;
  base << chip_ << "/pwm" << ch_;
  if (!write_file(base.str() + "/duty_cycle", std::to_string(duty))) return false;
  current_us_ = us;
  return true;
}

void ServoPwm::set_target_us(int us, int ramp_ms) {
  target_us_ = us;
  ramp_start_us_ = current_us_;
  ramp_total_ms_ = ramp_ms > 0 ? ramp_ms : 1;
  ramp_remaining_ms_ = ramp_total_ms_;
}

void ServoPwm::tick(int dt_ms) {
  if (!exported_) return;
  if (ramp_remaining_ms_ <= 0) {
    if (current_us_ != target_us_) set_pulse_us(target_us_);
    return;
  }
  ramp_remaining_ms_ -= dt_ms;
  if (ramp_remaining_ms_ < 0) ramp_remaining_ms_ = 0;
  double t = 1.0 - static_cast<double>(ramp_remaining_ms_) / static_cast<double>(ramp_total_ms_);
  if (t < 0) t = 0;
  if (t > 1) t = 1;
  int us = static_cast<int>(std::lround(ramp_start_us_ + (target_us_ - ramp_start_us_) * t));
  set_pulse_us(us);
}
