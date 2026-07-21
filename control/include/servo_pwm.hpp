#pragma once
#include <cstdint>
#include <string>

class ServoPwm {
 public:
  bool open(const std::string& pwmchip_path, int channel, int period_ns);
  void close();
  bool ok() const { return exported_; }
  /** Pulse width in microseconds (typically 1000–2000). */
  bool set_pulse_us(int us);
  /** Smooth move toward target_us over ramp_ms (non-blocking step). Call every loop. */
  void set_target_us(int us, int ramp_ms);
  void tick(int dt_ms);
  int current_us() const { return current_us_; }

 private:
  std::string chip_;
  int ch_ = 0;
  int period_ns_ = 20000000;
  bool exported_ = false;
  int current_us_ = 1500;
  int target_us_ = 1500;
  int ramp_remaining_ms_ = 0;
  int ramp_start_us_ = 1500;
  int ramp_total_ms_ = 0;
};
