#pragma once
#include <cstdint>
#include <string>

namespace msp {

constexpr uint8_t MSP_SET_RAW_RC = 200;
constexpr uint8_t MSP_DISPLAYPORT = 182;

// DisplayPort sub-commands (Betaflight)
constexpr uint8_t DP_HEARTBEAT = 0;
constexpr uint8_t DP_RELEASE = 1;
constexpr uint8_t DP_CLEAR = 2;
constexpr uint8_t DP_WRITE = 3;
constexpr uint8_t DP_DRAW = 4;

class Tx {
 public:
  bool open(const std::string& path, int baud);
  void close();
  bool ok() const { return fd_ >= 0; }
  /** Send MSP v1 SET_RAW_RC with 16 channels in microseconds. */
  bool send_raw_rc(const uint16_t ch_us[16]);

  /** Character-OSD box from normalized bbox (cx,cy,w,h in 0..1). cols/rows = OSD grid. */
  bool send_osd_bbox(double cx, double cy, double bw, double bh, int cols = 30, int rows = 16);
  /** Center aim reticle (static frame for CH8 aim-before-lock). */
  bool send_osd_aim(double bw = 0.15, double bh = 0.15, int cols = 30, int rows = 16);
  bool send_osd_clear();

 private:
  bool send_v1(uint8_t cmd, const uint8_t* payload, uint8_t size);
  bool dp_write(int row, int col, const char* text);
  bool dp_clear();
  bool dp_draw();
  int fd_ = -1;
};

}  // namespace msp
