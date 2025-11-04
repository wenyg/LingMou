#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "demo_interfaces/msg/control.hpp"

using namespace std::chrono_literals;

class ControlPublisherNode : public rclcpp::Node {
public:
  ControlPublisherNode() : Node("control_publisher_node") {
    publisher_ = this->create_publisher<demo_interfaces::msg::Control>(
        "/control_cmd", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&ControlPublisherNode::onTimer, this));
    start_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "control_publisher_node started. Publishing to /control_cmd");
  }

private:
  void onTimer() {
    auto now = this->now();
    double t = (now - start_time_).seconds();

    demo_interfaces::msg::Control msg;
    msg.stamp = now;
    msg.speed_mps = 5.0f + 2.0f * std::sin(t);  // simple variation
    msg.steering_deg = 10.0f * std::sin(t * 0.5);
    msg.gear = 1;

    publisher_->publish(msg);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Publishing Control: speed=%.2f mps, steer=%.2f deg, gear=%d",
      msg.speed_mps, msg.steering_deg, msg.gear);
  }

  rclcpp::Publisher<demo_interfaces::msg::Control>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlPublisherNode>());
  rclcpp::shutdown();
  return 0;
}

