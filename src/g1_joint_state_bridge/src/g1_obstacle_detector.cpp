#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/string.hpp"
#include <cmath>
#include <string>

const double WARN_DIST = 1.5;
const double STOP_DIST = 0.7;

class G1ObstacleDetector : public rclcpp::Node {
public:
  G1ObstacleDetector() : Node("g1_obstacle_detector") {
    alert_pub_ = this->create_publisher<std_msgs::msg::String>("/obstacle_alert", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/obstacle_markers", 10);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        process_scan(msg);
      });

    RCLCPP_INFO(this->get_logger(), "Obstacle detector started (LiDAR-based). Warn=%.1fm Stop=%.1fm",
                WARN_DIST, STOP_DIST);
  }

private:
  void process_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    double min_range = msg->range_max;
    int min_idx = 0;

    for (int i = 0; i < (int)msg->ranges.size(); i++) {
      float r = msg->ranges[i];
      if (r > msg->range_min && r < min_range) {
        min_range = r;
        min_idx = i;
      }
    }

    // Direction of closest obstacle
    float closest_angle = msg->angle_min + min_idx * msg->angle_increment;
    float closest_x = min_range * cos(closest_angle);
    float closest_y = min_range * sin(closest_angle);

    // Publish alert
    auto alert = std_msgs::msg::String();
    if (min_range < STOP_DIST) {
      alert.data = "STOP: obstacle at " +
        std::to_string(min_range).substr(0,4) + "m angle=" +
        std::to_string((int)(closest_angle * 180.0 / M_PI)) + "deg";
      RCLCPP_WARN(this->get_logger(), "%s", alert.data.c_str());
    } else if (min_range < WARN_DIST) {
      alert.data = "WARN: obstacle at " +
        std::to_string(min_range).substr(0,4) + "m angle=" +
        std::to_string((int)(closest_angle * 180.0 / M_PI)) + "deg";
      RCLCPP_INFO(this->get_logger(), "%s", alert.data.c_str());
    } else {
      alert.data = "CLEAR";
    }
    alert_pub_->publish(alert);

    // Publish marker at closest obstacle point
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "pelvis";
    m.header.stamp = this->now();
    m.ns = "closest_obstacle";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = closest_x;
    m.pose.position.y = closest_y;
    m.pose.position.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.3;
    if (min_range < STOP_DIST) {
      m.color.r=1.0f; m.color.g=0.0f; m.color.b=0.0f; m.color.a=1.0f;
    } else if (min_range < WARN_DIST) {
      m.color.r=1.0f; m.color.g=1.0f; m.color.b=0.0f; m.color.a=1.0f;
    } else {
      m.color.r=0.0f; m.color.g=1.0f; m.color.b=0.0f; m.color.a=0.5f;
    }
    arr.markers.push_back(m);
    marker_pub_->publish(arr);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alert_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<G1ObstacleDetector>());
  rclcpp::shutdown();
  return 0;
}
