#include <iostream>
#include <string>
#include <unistd.h>

#include "../active_marker_lib/include/uart.hpp"
#include "../include/active_marker/uart_proto.hpp"

int main(int argc, char* argv[]) {
  // argv[1]: ID, argv[2]: Team color
  if (argc < 3) {
    std::cerr << "Too few arguments." << std::endl;
    return -1;
  }
  uint8_t data[3] = {0};
  data[0] = static_cast<uint8_t>(active_marker::UartCommand::ID);

  uint8_t ID_arg = std::atoi(argv[1]);
  if (ID_arg >= 0 && ID_arg <= 16) {
    data[1] = ID_arg;
  } else {
    std::cerr << "Invalid ID argument." << std::endl;
    return -1;
  }

  if (std::string(argv[2]) == "yellow") {
    data[2] = static_cast<uint8_t>(active_marker::TeamColor::YELLOW);
  } else if (std::string(argv[2]) == "blue") {
    data[2] = static_cast<uint8_t>(active_marker::TeamColor::BLUE);
  } else {
    std::cerr << "Invalid team color argument." << std::endl;
    return -1;
  }

  lib::Uart uart("/dev/ttyTHS2", B38400);
  while(1){
    uart.transmit(data, sizeof(data));
    sleep(3);
  }
  return 0;
}