#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

namespace crsf {

constexpr uint8_t SYNC = 0xC8;
constexpr uint8_t TYPE_RC = 0x16;

uint8_t crc8(const uint8_t* data, size_t len);
inline int ticks_to_us(int t) { return (t - 992) * 5 / 8 + 1500; }

/** Unpack 16x11-bit channels from 22-byte payload → microseconds. */
bool unpack_channels(const uint8_t* payload22, int out_us[16]);

class Parser {
 public:
  /** Feed bytes; returns true when a fresh RC frame was decoded into channels_us_. */
  bool feed(const uint8_t* data, size_t n);
  const int* channels() const { return channels_us_; }
  uint64_t last_rc_ms() const { return last_rc_ms_; }

 private:
  std::vector<uint8_t> buf_;
  int channels_us_[16]{};
  uint64_t last_rc_ms_ = 0;
};

}  // namespace crsf
