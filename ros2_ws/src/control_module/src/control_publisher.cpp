#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "demo_interfaces/msg/control.hpp"
#include "demo_interfaces/msg/box.hpp"
#include "demo_interfaces/msg/box_array.hpp"
#include "demo_interfaces/msg/point.hpp"
#include "demo_interfaces/msg/point_array.hpp"
#include "demo_interfaces/msg/line.hpp"

using namespace std::chrono_literals;

class ControlPublisherNode : public rclcpp::Node {
public:
  ControlPublisherNode() : Node("control_publisher_node") {
    // Control message publisher
    control_publisher_ = this->create_publisher<demo_interfaces::msg::Control>(
        "/control_cmd", 10);
    
    // Custom visualization publishers
    box_publisher_ = this->create_publisher<demo_interfaces::msg::Box>(
        "/visualization/box", 10);
    box_array_publisher_ = this->create_publisher<demo_interfaces::msg::BoxArray>(
        "/visualization/box_array", 10);
    point_array_publisher_ = this->create_publisher<demo_interfaces::msg::PointArray>(
        "/visualization/point_array", 10);
    line_publisher_ = this->create_publisher<demo_interfaces::msg::Line>(
        "/visualization/line", 10);
    
    timer_ = this->create_wall_timer(100ms, std::bind(&ControlPublisherNode::onTimer, this));
    start_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "control_publisher_node started. Publishing to /control_cmd and custom visualization topics");
  }

private:
  void onTimer() {
    auto now = this->now();
    double t = (now - start_time_).seconds();

    // Publish Control message
    demo_interfaces::msg::Control control_msg;
    control_msg.stamp = now;
    control_msg.speed_mps = 5.0f + 2.0f * std::sin(t);
    control_msg.steering_deg = 10.0f * std::sin(t * 0.5);
    control_msg.gear = 1;
    control_publisher_->publish(control_msg);

    // Publish custom visualization messages
    publishPoints(now, t);
    publishLines(now, t);
    publishBoxes(now, t);
    publishBoxArray(now, t);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Publishing Control: speed=%.2f mps, steer=%.2f deg, gear=%d",
      control_msg.speed_mps, control_msg.steering_deg, control_msg.gear);
  }

  void publishPoints(const rclcpp::Time& now, double t) {
    demo_interfaces::msg::PointArray point_array;
    point_array.stamp = now;
    
    // Points in a circle
    int num_points = 8;
    for (int i = 0; i < num_points; ++i) {
      double angle = 2.0 * M_PI * i / num_points + t;
      demo_interfaces::msg::Point pt;
      pt.stamp = now;
      pt.x = 3.0f * static_cast<float>(std::cos(angle));
      pt.y = 3.0f * static_cast<float>(std::sin(angle));
      pt.z = 0.5f + 0.5f * static_cast<float>(std::sin(t + i));
      point_array.points.push_back(pt);
    }
    
    point_array_publisher_->publish(point_array);
  }

  void publishLines(const rclcpp::Time& now, double t) {
    demo_interfaces::msg::Line line;
    line.stamp = now;
    
    // Create a line that moves
    float offset = static_cast<float>(2.0 * std::sin(t));
    line.start_x = -5.0f;
    line.start_y = offset;
    line.start_z = 1.0f;
    
    line.end_x = 5.0f;
    line.end_y = offset + 1.0f;
    line.end_z = 1.0f;
    
    line_publisher_->publish(line);
  }

  void publishBoxes(const rclcpp::Time& now, double t) {
    demo_interfaces::msg::Box box;
    box.stamp = now;
    
    // Moving box with center position and dimensions
    box.center_x = 2.0f * static_cast<float>(std::cos(t));
    box.center_y = 2.0f * static_cast<float>(std::sin(t));
    box.center_z = 1.0f;
    
    // Dimensions (length, width, height)
    box.length = 1.0f;
    box.width = 0.5f;
    box.height = 0.3f;
    
    // Rotation (yaw)
    box.yaw = static_cast<float>(t);
    
    box_publisher_->publish(box);
  }

  void publishBoxArray(const rclcpp::Time& now, double t) {
    demo_interfaces::msg::BoxArray box_array;
    box_array.stamp = now;
    
    // Create multiple boxes in an array
    int num_boxes = 5;
    for (int i = 0; i < num_boxes; ++i) {
      demo_interfaces::msg::Box box;
      box.stamp = now;
      
      box.center_x = -2.0f + i * 1.0f;
      box.center_y = -3.0f;
      box.center_z = 0.5f + 0.3f * static_cast<float>(std::sin(t + i));
      
      box.length = 0.5f;
      box.width = 0.5f;
      box.height = 0.5f;
      
      box.yaw = static_cast<float>(i * 0.3);
      
      box_array.boxes.push_back(box);
    }
    
    box_array_publisher_->publish(box_array);
  }

  rclcpp::Publisher<demo_interfaces::msg::Control>::SharedPtr control_publisher_;
  rclcpp::Publisher<demo_interfaces::msg::Box>::SharedPtr box_publisher_;
  rclcpp::Publisher<demo_interfaces::msg::BoxArray>::SharedPtr box_array_publisher_;
  rclcpp::Publisher<demo_interfaces::msg::PointArray>::SharedPtr point_array_publisher_;
  rclcpp::Publisher<demo_interfaces::msg::Line>::SharedPtr line_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlPublisherNode>());
  rclcpp::shutdown();
  return 0;
}

