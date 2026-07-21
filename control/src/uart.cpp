#include "uart.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>

#include <asm/termbits.h>
#include <sys/ioctl.h>

int uart_open(const std::string& path, int baud, std::string* err) {
  int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    if (err) *err = std::string("open ") + path + ": " + std::strerror(errno);
    return -1;
  }
  termios2 tio{};
  if (ioctl(fd, TCGETS2, &tio) != 0) {
    if (err) *err = std::strerror(errno);
    ::close(fd);
    return -1;
  }
  tio.c_cflag &= ~CBAUD;
  tio.c_cflag |= BOTHER;
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~CSIZE;
  tio.c_cflag |= CS8;
  tio.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
  tio.c_iflag = 0;
  tio.c_oflag = 0;
  tio.c_lflag = 0;
  tio.c_ispeed = baud;
  tio.c_ospeed = baud;
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  if (ioctl(fd, TCSETS2, &tio) != 0) {
    if (err) *err = std::string("TCSETS2: ") + std::strerror(errno);
    ::close(fd);
    return -1;
  }
  return fd;
}

void uart_close(int fd) {
  if (fd >= 0) ::close(fd);
}
