#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "demo_interfaces/msg/box.hpp"
#include "demo_interfaces/msg/box_array.hpp"
#include "demo_interfaces/msg/point_array.hpp"
#include "demo_interfaces/msg/line.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"

class MarkerConverterNode : public rclcpp::Node {
public:
  MarkerConverterNode() : Node("marker_converter_node") {
    // Subscribe to custom messages
    box_sub_ = this->create_subscription<demo_interfaces::msg::Box>(
        "/visualization/box", 10,
        std::bind(&MarkerConverterNode::onBox, this, std::placeholders::_1));
    
    box_array_sub_ = this->create_subscription<demo_interfaces::msg::BoxArray>(
        "/visualization/box_array", 10,
        std::bind(&MarkerConverterNode::onBoxArray, this, std::placeholders::_1));
    
    point_array_sub_ = this->create_subscription<demo_interfaces::msg::PointArray>(
        "/visualization/point_array", 10,
        std::bind(&MarkerConverterNode::onPointArray, this, std::placeholders::_1));
    
    line_sub_ = this->create_subscription<demo_interfaces::msg::Line>(
        "/visualization/line", 10,
        std::bind(&MarkerConverterNode::onLine, this, std::placeholders::_1));
    
    // Publish visualization_msgs/Marker
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/visualization/marker", 10);
    marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/visualization/marker_array", 10);
    
    RCLCPP_INFO(this->get_logger(), "marker_converter_node started. Converting custom messages to visualization_msgs/Marker");
  }

private:
  void onBox(const demo_interfaces::msg::Box::ConstSharedPtr msg) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = msg->stamp;
    marker.ns = "box";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose.position.x = msg->center_x;
    marker.pose.position.y = msg->center_y;
    marker.pose.position.z = msg->center_z;
    
    // Convert yaw to quaternion
    double yaw = msg->yaw;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = std::sin(yaw / 2.0);
    marker.pose.orientation.w = std::cos(yaw / 2.0);
    
    marker.scale.x = msg->length;
    marker.scale.y = msg->width;
    marker.scale.z = msg->height;
    
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 0.8;
    
    marker_pub_->publish(marker);
  }

  void onBoxArray(const demo_interfaces::msg::BoxArray::ConstSharedPtr msg) {
    visualization_msgs::msg::MarkerArray marker_array;
    
    for (size_t i = 0; i < msg->boxes.size(); ++i) {
      const auto& box = msg->boxes[i];
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "base_link";
      marker.header.stamp = box.stamp;
      marker.ns = "box_array";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      
      marker.pose.position.x = box.center_x;
      marker.pose.position.y = box.center_y;
      marker.pose.position.z = box.center_z;
      
      double yaw = box.yaw;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = std::sin(yaw / 2.0);
      marker.pose.orientation.w = std::cos(yaw / 2.0);
      
      marker.scale.x = box.length;
      marker.scale.y = box.width;
      marker.scale.z = box.height;
      
      // Color gradient
      marker.color.r = 1.0f - i * 0.2f;
      marker.color.g = 1.0f;
      marker.color.b = i * 0.2f;
      marker.color.a = 0.9;
      
      marker_array.markers.push_back(marker);
    }
    
    marker_array_pub_->publish(marker_array);
  }

  void onPointArray(const demo_interfaces::msg::PointArray::ConstSharedPtr msg) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = msg->stamp;
    marker.ns = "points";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    
    for (const auto& pt : msg->points) {
      geometry_msgs::msg::Point p;
      p.x = pt.x;
      p.y = pt.y;
      p.z = pt.z;
      marker.points.push_back(p);
    }
    
    marker_pub_->publish(marker);
  }

  void onLine(const demo_interfaces::msg::Line::ConstSharedPtr msg) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = msg->stamp;
    marker.ns = "line";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    
    geometry_msgs::msg::Point start;
    start.x = msg->start_x;
    start.y = msg->start_y;
    start.z = msg->start_z;
    marker.points.push_back(start);
    
    geometry_msgs::msg::Point end;
    end.x = msg->end_x;
    end.y = msg->end_y;
    end.z = msg->end_z;
    marker.points.push_back(end);
    
    marker_pub_->publish(marker);
  }

  rclcpp::Subscription<demo_interfaces::msg::Box>::SharedPtr box_sub_;
  rclcpp::Subscription<demo_interfaces::msg::BoxArray>::SharedPtr box_array_sub_;
  rclcpp::Subscription<demo_interfaces::msg::PointArray>::SharedPtr point_array_sub_;
  rclcpp::Subscription<demo_interfaces::msg::Line>::SharedPtr line_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarkerConverterNode>());
  rclcpp::shutdown();
  return 0;
}

