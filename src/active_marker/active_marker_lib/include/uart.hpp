#ifndef UART_HPP_
#define UART_HPP_
#include <termios.h>

#include <cstdint>

namespace lib {

class Uart {
 public:
  Uart(const char* port, int baud_rate);
  ~Uart();

  void transmit(const uint8_t* data, const int size);
  void receive(uint8_t* received_data, const int buffer_size);

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