#include <iostream>

#include "../active_marker_lib/include/uart.hpp"
#include "../include/active_marker/uart_proto.hpp"
int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cerr << "Too few arguments." << std::endl;
    return -1;
  }

  // set team color
  uint8_t data[3] = {0};
  data[0] = static_cast<uint8_t>(active_marker::UartCommand::ID);

  uint8_t ID_arg = std::atoi(argv[1]);
  if (ID_arg >= 0 || ID_arg <= 15) {
    data[1] = ID_arg;
  } else {
    std::cerr << "Invalid ID argument." << std::endl;
    return -1;
  }

  if (argv[2] == "yellow") {
    data[2] = static_cast<uint8_t>(active_marker::TeamColor::YELLOW);
  } else if (argv[2] == "blue") {
    data[2] = static_cast<uint8_t>(active_marker::TeamColor::BLUE);
  } else {
    std::cerr << "Invalid team color argument." << std::endl;
    return -1;
  }

  lib::Uart uart("/dev/ttyTHS2", B38400);
  uart.transmit(data, sizeof(data));

  std::cout << "ID settings finished successfully." << std::endl;
  return 0;
}