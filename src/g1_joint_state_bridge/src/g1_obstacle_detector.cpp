#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <cmath>
#include <vector>
#include <string>

struct Obstacle {
  std::string name;
  double x, y, z;       // center
  double sx, sy, sz;    // half-sizes (collision bbox)
};

// Key obstacles G1 can actually hit (waist height and below)
const std::vector<Obstacle> OBSTACLES = {
  // Walls
  {"wall_north",    0,  10, 3.0,  8,   0.1, 3.0},
  {"wall_south",    0, -10, 3.0,  8,   0.1, 3.0},
  {"wall_east",     8,   0, 3.0,  0.1, 10,  3.0},
  {"wall_west",    -8,   0, 3.0,  0.1, 10,  3.0},
  // Conference table
  {"conf_table",    0,  -5, 0.75, 1.5, 3.0, 0.75},
  // L-desk
  {"desk_L",       -5,   6, 0.75, 2.0, 1.5, 0.75},
  // Std desks
  {"desk_std_1",    5,   8, 0.75, 0.8, 1.2, 0.75},
  {"desk_std_2",    5,   6, 0.75, 0.8, 1.2, 0.75},
  // Humanoids
  {"human_A",      -5.5, 7.0, 0.9, 0.3, 0.3, 0.9},
  {"human_B",       5.5, 7.0, 0.9, 0.3, 0.3, 0.9},
};

const double WARN_DIST = 1.5;   // meters — show yellow
const double STOP_DIST = 0.7;   // meters — show red + alert

class G1ObstacleDetector : public rclcpp::Node {
public:
  G1ObstacleDetector() : Node("g1_obstacle_detector") {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    alert_pub_ = this->create_publisher<std_msgs::msg::String>("/obstacle_alert", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/obstacle_markers", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
      [this]() { check_obstacles(); });

    RCLCPP_INFO(this->get_logger(), "Obstacle detector started. Warn=%.1fm Stop=%.1fm",
                WARN_DIST, STOP_DIST);
  }

private:
  double point_to_box_dist(double px, double py, double pz, const Obstacle &o) {
    // Distance from point to box surface (0 if inside)
    double dx = std::max(0.0, std::abs(px - o.x) - o.sx);
    double dy = std::max(0.0, std::abs(py - o.y) - o.sy);
    double dz = std::max(0.0, std::abs(pz - o.z) - o.sz);
    return std::sqrt(dx*dx + dy*dy + dz*dz);
  }

  void check_obstacles() {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform("odom", "pelvis", tf2::TimePointZero);
    } catch (const tf2::TransformException &e) {
      return;
    }

    double rx = tf.transform.translation.x;
    double ry = tf.transform.translation.y;
    double rz = tf.transform.translation.z;

    visualization_msgs::msg::MarkerArray arr;
    std::string closest_name;
    double closest_dist = 999.0;

    for (int i = 0; i < (int)OBSTACLES.size(); i++) {
      const auto &obs = OBSTACLES[i];
      double dist = point_to_box_dist(rx, ry, rz, obs);

      if (dist < closest_dist) {
        closest_dist = dist;
        closest_name = obs.name;
      }

      // Colored bbox marker
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "odom";
      m.header.stamp = this->now();
      m.ns = "collision_boxes";
      m.id = i;
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = obs.x;
      m.pose.position.y = obs.y;
      m.pose.position.z = obs.z;
      m.pose.orientation.w = 1.0;
      m.scale.x = obs.sx*2; m.scale.y = obs.sy*2; m.scale.z = obs.sz*2;

      if (dist < STOP_DIST) {
        m.color.r=1.0f; m.color.g=0.0f; m.color.b=0.0f; m.color.a=0.5f; // red
      } else if (dist < WARN_DIST) {
        m.color.r=1.0f; m.color.g=1.0f; m.color.b=0.0f; m.color.a=0.4f; // yellow
      } else {
        m.color.r=0.0f; m.color.g=1.0f; m.color.b=0.0f; m.color.a=0.2f; // green
      }
      arr.markers.push_back(m);
    }

    marker_pub_->publish(arr);

    // Publish alert
    auto alert = std_msgs::msg::String();
    if (closest_dist < STOP_DIST) {
      alert.data = "STOP: " + closest_name + " dist=" + std::to_string(closest_dist).substr(0,4) + "m";
      RCLCPP_WARN(this->get_logger(), "%s", alert.data.c_str());
    } else if (closest_dist < WARN_DIST) {
      alert.data = "WARN: " + closest_name + " dist=" + std::to_string(closest_dist).substr(0,4) + "m";
      RCLCPP_INFO(this->get_logger(), "%s", alert.data.c_str());
    } else {
      alert.data = "CLEAR";
    }
    alert_pub_->publish(alert);
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alert_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<G1ObstacleDetector>());
  rclcpp::shutdown();
  return 0;
}
