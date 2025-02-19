#ifndef UDP_HPP_
#define UDP_HPP_

#include <unistd.h>

#include <functional>
#include <thread>

namespace lib {

class UdpReceiver {
 public:
  UdpReceiver(int port, std::function<void(uint16_t)> callback);
  ~UdpReceiver();

  void start();
  void stop();

 private:
  void udp_receive();
  int sockfd_;
  std::thread udp_thread_;
  bool is_runnning_;
  std::function<void(uint16_t)> callback_;
};

}  // namespace lib

#endif  // UDP_HPP_
