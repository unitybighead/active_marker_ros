#ifndef UART_HPP_
#define UART_HPP_

#include <fcntl.h>
#include <malloc.h>
#include <sys/file.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <iostream>
namespace lib {

class Uart {
 public:
  Uart(const char* port, int baud_rate) {
    port_ = port;
    baud_rate_ = baud_rate;
    uart_filestream_ = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (uart_filestream_ == -1) {
      std::cerr << "Unable to open UART" << std::endl;
    }
    setOption();
  }
  ~Uart() { close(uart_filestream_); }

  void transmit(const uint8_t* data, const int size);
  void receive(uint8_t* received_data);

 private:
  void setOption();
  bool is_open();

  const char* port_;
  int uart_filestream_;
  int baud_rate_;
  bool is_open_;
  const char* kuart_lock_file_ = "/tmp/uart_lock";
};
}  // namespace lib

#endif  // UART_HPP_