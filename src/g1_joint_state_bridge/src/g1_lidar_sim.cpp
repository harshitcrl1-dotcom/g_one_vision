#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <cmath>
#include <vector>

struct Obstacle {
  double x, y, z;
  double sx, sy, sz;
};

const std::vector<Obstacle> OBSTACLES = {
  {0,  10, 3.0,  8,   0.1, 3.0},
  {0, -10, 3.0,  8,   0.1, 3.0},
  {8,   0, 3.0,  0.1, 10,  3.0},
  {-8,  0, 3.0,  0.1, 10,  3.0},
  {0,  -5, 0.75, 1.5, 3.0, 0.75},
  {-5,  6, 0.75, 2.0, 1.5, 0.75},
  {5,   8, 0.75, 0.8, 1.2, 0.75},
  {5,   6, 0.75, 0.8, 1.2, 0.75},
  {-5.5,7, 0.9,  0.3, 0.3, 0.9},
  {5.5, 7, 0.9,  0.3, 0.3, 0.9},
};

double ray_box_intersect(double ox, double oy,
                         double dx, double dy,
                         const Obstacle &o) {
  double tmin = 0.0, tmax = 1e9;
  if (std::abs(dx) > 1e-9) {
    double t1 = (o.x - o.sx - ox) / dx;
    double t2 = (o.x + o.sx - ox) / dx;
    if (t1 > t2) std::swap(t1, t2);
    tmin = std::max(tmin, t1);
    tmax = std::min(tmax, t2);
  } else if (ox < o.x - o.sx || ox > o.x + o.sx) return -1;
  if (std::abs(dy) > 1e-9) {
    double t1 = (o.y - o.sy - oy) / dy;
    double t2 = (o.y + o.sy - oy) / dy;
    if (t1 > t2) std::swap(t1, t2);
    tmin = std::max(tmin, t1);
    tmax = std::min(tmax, t2);
  } else if (oy < o.y - o.sy || oy > o.y + o.sy) return -1;
  if (tmax < tmin || tmax < 0) return -1;
  return tmin >= 0 ? tmin : tmax;
}

class G1LidarSim : public rclcpp::Node {
public:
  G1LidarSim() : Node("g1_lidar_sim") {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      [this]() { publish_scan(); });
    RCLCPP_INFO(this->get_logger(), "G1 LiDAR simulator started");
  }

private:
  void publish_scan() {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform("odom", "pelvis", tf2::TimePointZero);
    } catch (...) { return; }

    double rx = tf.transform.translation.x;
    double ry = tf.transform.translation.y;

    // Extract yaw from quaternion
    double qx = tf.transform.rotation.x;
    double qy = tf.transform.rotation.y;
    double qz = tf.transform.rotation.z;
    double qw = tf.transform.rotation.w;
    double yaw = atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz));

    const int NUM_RAYS = 360;
    const float MAX_RANGE = 10.0f;

    auto msg = sensor_msgs::msg::LaserScan();
    msg.header.stamp = this->now();
    msg.header.frame_id = "pelvis";
    msg.angle_min = 0.0f;
    msg.angle_max = 2.0f * M_PI;
    msg.angle_increment = 2.0f * M_PI / NUM_RAYS;
    msg.range_min = 0.1f;
    msg.range_max = MAX_RANGE;
    msg.ranges.resize(NUM_RAYS, MAX_RANGE);

    for (int i = 0; i < NUM_RAYS; i++) {
      // Ray angle in robot frame, rotated by yaw to world frame
      float angle = yaw + i * 2.0f * M_PI / NUM_RAYS;
      double dx = cos(angle);
      double dy = sin(angle);
      double min_dist = MAX_RANGE;

      for (const auto &obs : OBSTACLES) {
        Obstacle rel_obs = obs;
        rel_obs.x = obs.x - rx;
        rel_obs.y = obs.y - ry;
        double dist = ray_box_intersect(0, 0, dx, dy, rel_obs);
        if (dist > 0.1 && dist < min_dist)
          min_dist = dist;
      }
      msg.ranges[i] = (float)min_dist;
    }
    pub_->publish(msg);
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<G1LidarSim>());
  rclcpp::shutdown();
  return 0;
}
