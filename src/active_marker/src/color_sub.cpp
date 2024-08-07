#include "active_marker/color_sub.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<active_marker::ColorSubNode>());
  rclcpp::shutdown();
  return 0;
}