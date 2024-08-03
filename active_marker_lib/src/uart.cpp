#include "../include/uart.hpp"

namespace lib {
void Uart::setOption() {
  struct termios options;
  tcgetattr(uart_filestream_, &options);  // get current settings

  // set baud_rate
  cfsetispeed(&options, baud_rate_);
  cfsetospeed(&options, baud_rate_);

  // set data_bit stop_bit parity
  options.c_cflag = baud_rate_ | CS8 | CLOCAL | CREAD;
  options.c_iflag = IGNPAR;
  options.c_oflag = 0;
  options.c_lflag = 0;

  tcflush(uart_filestream_, TCIFLUSH);             // clear io buffer
  tcsetattr(uart_filestream_, TCSANOW, &options);  // apply new settings
}

void Uart::transmit(const char* data) {
  if (uart_filestream_ != -1) {
    int count = write(uart_filestream_, data, strlen(data));
    if (count < 0) {
      std::cerr << "UART TX Error" << std::endl;
    }
  }
}

void Uart::receive(char* received_data) {
  if (uart_filestream_ != -1) {
    unsigned char rx_buffer[256];
    int rx_rength = read(uart_filestream_, (void*)rx_buffer, 255);
    if (rx_rength < 0) {
      std::cerr << "UART RX Error" << std::endl;
    } else if (rx_rength == 0) {
      std::cout << "No data received" << std::endl;
    } else {
      rx_buffer[rx_rength] = '\0';
      std::copy(rx_buffer, rx_buffer + rx_rength, received_data);
      std::cout << "Recived" << rx_buffer << std::endl;
    }
  }
}

bool Uart::is_open() { return (uart_filestream_ != -1); }

}  // namespace lib