#ifndef UART_ASIO_HPP_
#define UART_ASIO_HPP_
#include <boost/asio.hpp>

using namespace boost::asio;

namespace lib {
class UartAsio {
 public:
  UartAsio(io_service& io, const std::string& port, int baudrate);
  void set_option(int baudrate);
  std::vector<std::uint8_t> get_data();
  void start_transmit(const std::vector<std::uint8_t> data);
  void start_receive();

 private:
  serial_port serial_;
  std::vector<std::uint8_t> receive_buffer_;
};
}  // namespace lib

#endif