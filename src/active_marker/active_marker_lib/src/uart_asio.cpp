#include "../include/uart_asio.hpp"

#include <iostream>

using namespace boost::asio;

namespace lib {
UartAsio::UartAsio(io_service& io, const std::string& port, int baudrate)
    : serial_(io, port), receive_buffer_(8) {
  set_option(baudrate);
  start_receive();
}

void UartAsio::set_option(int baudrate) {
  serial_.set_option(serial_port_base::baud_rate(baudrate));
  serial_.set_option(serial_port_base::character_size(8));
  serial_.set_option(serial_port_base::parity(serial_port_base::parity::odd));
  serial_.set_option(
      serial_port_base::stop_bits(serial_port_base::stop_bits::one));
  serial_.set_option(
      serial_port_base::flow_control(serial_port_base::flow_control::none));
}

std::vector<std::uint8_t> UartAsio::get_data() { return receive_buffer_; }

void UartAsio::start_transmit(const std::vector<std::uint8_t> data) {
  boost::asio::async_write(
      serial_, buffer(data),
      [this](boost::system::error_code ec, std::size_t length) {
        if (!ec) {
          std::cout << "Sent " << length << " bytes successfully." << std::endl;
        } else {
          std::cerr << "Send Error: " << ec.message() << std::endl;
        }
      });
}

void UartAsio::start_receive() {
  async_read(serial_, buffer(receive_buffer_, 8),
             [this](boost::system::error_code ec, std::size_t length) {
               if (!ec && length == 8) {
                 std::cout << "received 8 bytes" << std::endl;
               } else {
                 std::cerr << "Error: " << ec.message() << std::endl;
               }
               start_receive();
             });
}

}  // namespace lib