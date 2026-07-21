#pragma once
#include <string>

/** Open UART raw 8N1; supports non-standard baud (e.g. 420000) via termios2. */
int uart_open(const std::string& path, int baud, std::string* err = nullptr);
void uart_close(int fd);
