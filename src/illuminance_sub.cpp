#include "active_marker/illuminance_sub.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<illuminance_sub::IlluminanceSubNode>());
  rclcpp::shutdown();
  return 0;
}