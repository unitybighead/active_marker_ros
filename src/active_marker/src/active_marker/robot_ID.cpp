#include "active_marker/robot_ID.hpp"

#include "active_marker/uart_proto.hpp"

using namespace std::chrono_literals;

namespace active_marker {
RobotIDNode::RobotIDNode()
    : Node("robot_ID", "/am"),
      ID_(10),
      team_color_("yellow"),
      uart_("/dev/ttyTHS2", B115200) {
  RCLCPP_INFO(this->get_logger(), "%d", ID_);
  RCLCPP_INFO(this->get_logger(), "%s", team_color_.c_str());
  last_key_subsctiption_ = this->create_subscription<Int16Msg>(
      "last_key", rclcpp::QoS(1).reliable(),
      std::bind(&RobotIDNode::set_team_color, this, std::placeholders::_1));
  timer_ =
      this->create_wall_timer(1000ms, std::bind(&RobotIDNode::update, this));
  this->declare_parameter<int>("ID", 10);
  this->declare_parameter<std::string>("team_color", "yellow");
}

void RobotIDNode::set_team_color(Int16Msg::SharedPtr msg) {
 // if (ID_ == 16) {  // debug only
  if(1){
    switch (msg->data) {
      case 'b':
      case 'B':
        this->set_parameters({rclcpp::Parameter("team_color", "blue")});
        break;
      case 'y':
      case 'Y':
        this->set_parameters({rclcpp::Parameter("team_color", "yellow")});
        break;
      default:
        break;
    }
  }
}

void RobotIDNode::update() {
  this->get_parameter("ID", ID_);
  this->get_parameter("team_color", team_color_);
  uint8_t data[8] = {0};
  data[0] = static_cast<uint8_t>(UartCommand::ID);
  if (ID_ < 0 || ID_ > 16) {
    RCLCPP_ERROR(this->get_logger(), "Invalid ID");
    return;
  }
  data[1] = static_cast<uint8_t>(ID_);
  if (team_color_ == "blue") {
    data[2] = static_cast<uint8_t>(TeamColor::BLUE);
  } else if (team_color_ == "yellow") {
    data[2] = static_cast<uint8_t>(TeamColor::YELLOW);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Invalid team color");
    return;
  }

  uart_.transmit(data, sizeof(data));
}

}  // namespace active_marker
