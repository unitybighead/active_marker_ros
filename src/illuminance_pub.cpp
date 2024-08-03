#include "active_marker/illuminance_pub.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<active_marker::IlluminancePubNode>());
  rclcpp::shutdown();
  return 0;
}