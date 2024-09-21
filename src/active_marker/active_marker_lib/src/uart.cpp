#include "../include/uart.hpp"

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <iostream>

namespace lib {
Uart::Uart(const char* port, int baud_rate) {
  port_ = port;
  baud_rate_ = baud_rate;
  uart_filestream_ = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
  if (uart_filestream_ == -1) {
    std::cerr << "Unable to open UART" << std::endl;
  }
  setOption();
}

Uart::~Uart() { close(uart_filestream_); }

void Uart::setOption() {
  struct termios options;
  tcgetattr(uart_filestream_, &options);  // get current settings

  // set baud_rate
  cfsetispeed(&options, baud_rate_);
  cfsetospeed(&options, baud_rate_);

  // set data_bit stop_bit parity
  options.c_cflag = baud_rate_ | CS8 | CLOCAL | CREAD | PARENB | PARODD;
  options.c_iflag = IGNPAR;
  options.c_oflag = 0;
  options.c_lflag = 0;

  tcflush(uart_filestream_, TCIFLUSH);             // clear io buffer
  tcsetattr(uart_filestream_, TCSANOW, &options);  // apply new settings
}

void Uart::transmit(const uint8_t* data, const int size) {
  // set mutex file
  int fd = open(kuart_lock_file_, O_CREAT | O_RDWR, 0666);

  // transmit
  if (uart_filestream_ != -1 && flock(fd, LOCK_EX) == 0) {
    int count = write(uart_filestream_, data, sizeof(data));
    if (count < 0) {
      std::cerr << "UART TX Error" << std::endl;
    }
  }
  flock(fd, LOCK_UN);
  close(fd);
}

void Uart::receive(uint8_t* received_data, const int buffer_size) {
  // set mutex file
  int fd = open(kuart_lock_file_, O_CREAT | O_RDWR, 0666);

  if (fd == -1) {
    std::cerr << "Error opening lock file" << std::endl;
    return;
  }

  if (flock(fd, LOCK_EX) != 0) {
    std::cerr << "Error locking file" << std::endl;
    close(fd);
    return;
  }

  if (uart_filestream_ != -1) {
    unsigned char rx_buffer[256];
    int rx_length = read(uart_filestream_, rx_buffer, sizeof(rx_buffer));

    if (rx_length < 0) {
      std::cerr << "UART RX Error: " << strerror(errno) << std::endl;
    } else if (rx_length == 0) {
      std::cout << "No data received yet" << std::endl;
    } else {
      std::cout << "Bytes received: " << rx_length << std::endl;
      std::copy(rx_buffer, rx_buffer + std::min(rx_length, (int)buffer_size),
                received_data);
    }
  }

  // Unlock and close the file
  flock(fd, LOCK_UN);
  close(fd);
}

bool Uart::is_open() { return (uart_filestream_ != -1); }

}  // namespace lib