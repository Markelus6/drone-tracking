#include "msp.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace msp {

static bool set_baud(int fd, int baud) {
  speed_t sp = B115200;
  switch (baud) {
    case 9600: sp = B9600; break;
    case 57600: sp = B57600; break;
    case 115200: sp = B115200; break;
    case 230400: sp = B230400; break;
    case 460800: sp = B460800; break;
    default: sp = B115200; break;
  }
  termios tio{};
  if (tcgetattr(fd, &tio) != 0) return false;
  cfmakeraw(&tio);
  cfsetispeed(&tio, sp);
  cfsetospeed(&tio, sp);
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~CRTSCTS;
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  return tcsetattr(fd, TCSANOW, &tio) == 0;
}

bool Tx::open(const std::string& path, int baud) {
  close();
  fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) return false;
  if (!set_baud(fd_, baud)) {
    close();
    return false;
  }
  return true;
}

void Tx::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool Tx::send_v1(uint8_t cmd, const uint8_t* payload, uint8_t size) {
  if (fd_ < 0) return false;
  uint8_t crc = size ^ cmd;
  for (uint8_t i = 0; i < size; ++i) crc ^= payload[i];

  std::vector<uint8_t> frame(5 + size + 1);
  frame[0] = '$';
  frame[1] = 'M';
  frame[2] = '<';
  frame[3] = size;
  frame[4] = cmd;
  if (size) std::memcpy(frame.data() + 5, payload, size);
  frame[5 + size] = crc;

  size_t off = 0;
  while (off < frame.size()) {
    ssize_t n = ::write(fd_, frame.data() + off, frame.size() - off);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        usleep(100);
        continue;
      }
      return false;
    }
    off += static_cast<size_t>(n);
  }
  return true;
}

bool Tx::send_raw_rc(const uint16_t ch_us[16]) {
  uint8_t payload[32];
  for (int i = 0; i < 16; ++i) {
    payload[i * 2] = static_cast<uint8_t>(ch_us[i] & 0xFF);
    payload[i * 2 + 1] = static_cast<uint8_t>((ch_us[i] >> 8) & 0xFF);
  }
  return send_v1(MSP_SET_RAW_RC, payload, 32);
}

bool Tx::dp_clear() {
  uint8_t p[] = {DP_CLEAR};
  return send_v1(MSP_DISPLAYPORT, p, 1);
}

bool Tx::dp_draw() {
  uint8_t p[] = {DP_DRAW};
  return send_v1(MSP_DISPLAYPORT, p, 1);
}

bool Tx::dp_write(int row, int col, const char* text) {
  if (!text) return false;
  size_t len = std::strlen(text);
  if (len > 40) len = 40;
  uint8_t payload[3 + 40];
  payload[0] = DP_WRITE;
  payload[1] = static_cast<uint8_t>(std::max(0, row));
  payload[2] = static_cast<uint8_t>(std::max(0, col));
  std::memcpy(payload + 3, text, len);
  return send_v1(MSP_DISPLAYPORT, payload, static_cast<uint8_t>(3 + len));
}

bool Tx::send_osd_clear() {
  return dp_clear() && dp_draw();
}

bool Tx::send_osd_bbox(double cx, double cy, double bw, double bh, int cols, int rows) {
  if (cols < 8 || rows < 4) return false;
  int x1 = static_cast<int>(std::lround((cx - bw / 2.0) * (cols - 1)));
  int y1 = static_cast<int>(std::lround((cy - bh / 2.0) * (rows - 1)));
  int x2 = static_cast<int>(std::lround((cx + bw / 2.0) * (cols - 1)));
  int y2 = static_cast<int>(std::lround((cy + bh / 2.0) * (rows - 1)));
  x1 = std::clamp(x1, 0, cols - 1);
  x2 = std::clamp(x2, 0, cols - 1);
  y1 = std::clamp(y1, 0, rows - 1);
  y2 = std::clamp(y2, 0, rows - 1);
  if (x2 < x1) std::swap(x1, x2);
  if (y2 < y1) std::swap(y1, y2);

  dp_clear();

  // top / bottom edges
  std::string line(static_cast<size_t>(x2 - x1 + 1), '-');
  if (!line.empty()) {
    line.front() = '+';
    line.back() = '+';
    dp_write(y1, x1, line.c_str());
    dp_write(y2, x1, line.c_str());
  }
  // sides
  for (int r = y1 + 1; r < y2; ++r) {
    dp_write(r, x1, "|");
    dp_write(r, x2, "|");
  }
  // center cross
  int mx = (x1 + x2) / 2;
  int my = (y1 + y2) / 2;
  dp_write(my, mx, "+");
  return dp_draw();
}

bool Tx::send_osd_aim(double bw, double bh, int cols, int rows) {
  return send_osd_bbox(0.5, 0.5, bw, bh, cols, rows);
}

}  // namespace msp
