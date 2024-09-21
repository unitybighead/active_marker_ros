#include "../include/udp.hpp"

#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <sys/socket.h>

namespace lib {

UdpReceiver::UdpReceiver(int port, std::function<void(uint16_t)> callback)
    : callback_(callback), is_runnning_(false) {
  sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd_ < 0) {
    throw std::runtime_error("Failed to create socket");
  }

  sockaddr_in servaddr;
  memset(&servaddr, 0, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = INADDR_ANY;
  servaddr.sin_port = htons(port);

  if (bind(sockfd_, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
    throw std::runtime_error("Failed to bind socket");
  }
  start();
}

UdpReceiver::~UdpReceiver() {
  stop();
  close(sockfd_);
}

void UdpReceiver::start() {
  is_runnning_ = true;
  udp_thread_ = std::thread(&UdpReceiver::udp_receive, this);
}

void UdpReceiver::stop() {
  is_runnning_ = false;
  if (udp_thread_.joinable()) {
    udp_thread_.join();
  }
}

// TODO: 受信と変換の切り離し
void UdpReceiver::udp_receive() {
  char buffer[1024];
  sockaddr_in cliaddr;
  socklen_t len = sizeof(cliaddr);

  while (is_runnning_) {
    ssize_t n = recvfrom(sockfd_, buffer, sizeof(buffer), 0,
                         (struct sockaddr *)&cliaddr, &len);
    if (n > 0) {
      buffer[n] = '\0';
      try {
        uint16_t value = buffer[0] + (buffer[1] << 8);
        callback_(value);
      } catch (const std::exception &e) {
        std::cerr << "Failed to parse illuminance value: " << e.what()
                  << std::endl;
      }
    }
  }
}

}  // namespace lib
