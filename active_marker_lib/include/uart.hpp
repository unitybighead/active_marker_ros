#ifndef UART_HPP_
#define UART_HPP_

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <iostream>

namespace lib {

class Uart {
 public:
  Uart(const char* port, int baud_rate) {
    is_open_ = false;
    port_ = port;
    baud_rate_ = baud_rate;
    uart_filestream_ = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (uart_filestream_ == -1) {
      std::cerr << "Unable to open UART" << std::endl;
    }
    is_open_ = true;
    setOption();
  }
  ~Uart() { close(uart_filestream_); }

 private:
  void setOption();
  void transmit(const char* data);
  void receive(char* received_data);
  bool is_open();

  const char* port_;
  int uart_filestream_;
  int baud_rate_;
  bool is_open_;
};
}  // namespace lib

#endif  // UART_HPP_