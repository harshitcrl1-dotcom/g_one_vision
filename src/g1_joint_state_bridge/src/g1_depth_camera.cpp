#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
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

// Ray-box intersection returning hit point
bool ray_box_hit(double ox, double oy, double oz,
                 double dx, double dy, double dz,
                 const Obstacle &o,
                 double &hx, double &hy, double &hz) {
  double tmin = 0.0, tmax = 1e9;
  auto slab = [&](double o_c, double d_c, double bmin, double bmax) -> bool {
    if (std::abs(d_c) < 1e-9) return o_c >= bmin && o_c <= bmax;
    double t1 = (bmin - o_c) / d_c;
    double t2 = (bmax - o_c) / d_c;
    if (t1 > t2) std::swap(t1, t2);
    tmin = std::max(tmin, t1);
    tmax = std::min(tmax, t2);
    return tmax >= tmin;
  };
  if (!slab(ox, dx, o.x-o.sx, o.x+o.sx)) return false;
  if (!slab(oy, dy, o.y-o.sy, o.y+o.sy)) return false;
  if (!slab(oz, dz, o.z-o.sz, o.z+o.sz)) return false;
  if (tmax < 0 || tmin > 8.0) return false;
  double t = tmin >= 0 ? tmin : tmax;
  hx = ox + t*dx;
  hy = oy + t*dy;
  hz = oz + t*dz;
  return true;
}

class G1DepthCamera : public rclcpp::Node {
public:
  G1DepthCamera() : Node("g1_depth_camera") {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/depth_camera/points", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), // 10Hz
      [this]() { publish_pointcloud(); });
    RCLCPP_INFO(this->get_logger(), "G1 depth camera simulator started");
  }

private:
  void publish_pointcloud() {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform("odom", "torso_link", tf2::TimePointZero);
    } catch (...) {
      try {
        tf = tf_buffer_->lookupTransform("odom", "pelvis", tf2::TimePointZero);
      } catch (...) { return; }
    }

    double cx = tf.transform.translation.x;
    double cy = tf.transform.translation.y;
    double cz = tf.transform.translation.z + 0.3; // camera height offset

    // Camera orientation from quaternion
    double qx = tf.transform.rotation.x;
    double qy = tf.transform.rotation.y;
    double qz = tf.transform.rotation.z;
    double qw = tf.transform.rotation.w;
    double yaw = atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz));

    // D435 FOV: 87deg horizontal, 58deg vertical
    const float H_FOV = 87.0 * M_PI / 180.0;
    const float V_FOV = 58.0 * M_PI / 180.0;
    const int H_RES = 64;  // reduced for performance
    const int V_RES = 32;

    std::vector<float> points;

    for (int v = 0; v < V_RES; v++) {
      for (int h = 0; h < H_RES; h++) {
        float h_angle = yaw + (h - H_RES/2.0f) * H_FOV / H_RES;
        float v_angle = (v - V_RES/2.0f) * V_FOV / V_RES;

        double dx = cos(v_angle) * cos(h_angle);
        double dy = cos(v_angle) * sin(h_angle);
        double dz = sin(v_angle);

        double best_t = 8.0;
        double hx=0, hy=0, hz=0;
        bool hit = false;

        for (const auto &obs : OBSTACLES) {
          double thx, thy, thz;
          if (ray_box_hit(cx, cy, cz, dx, dy, dz, obs, thx, thy, thz)) {
            double t = sqrt((thx-cx)*(thx-cx)+(thy-cy)*(thy-cy)+(thz-cz)*(thz-cz));
            if (t < best_t) {
              best_t = t; hx=thx; hy=thy; hz=thz; hit=true;
            }
          }
        }

        if (hit) {
          // Rotate to robot-relative frame
          double dx_w = hx - cx;
          double dy_w = hy - cy;
          double dz_w = hz - cz;
          // Rotate by -yaw to get robot frame
          double px =  dx_w * cos(-yaw) - dy_w * sin(-yaw);
          double py =  dx_w * sin(-yaw) + dy_w * cos(-yaw);
          double pz =  dz_w;
          points.push_back((float)px);
          points.push_back((float)py);
          points.push_back((float)pz);
        }
      }
    }

    if (points.empty()) return;

    // Pack into PointCloud2
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "pelvis";
    msg.height = 1;
    msg.width = points.size() / 3;
    msg.is_dense = true;
    msg.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier mod(msg);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(points.size() / 3);

    sensor_msgs::PointCloud2Iterator<float> ix(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iy(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iz(msg, "z");

    for (size_t i = 0; i < points.size()/3; i++, ++ix, ++iy, ++iz) {
      *ix = points[i*3+0];
      *iy = points[i*3+1];
      *iz = points[i*3+2];
    }

    pub_->publish(msg);
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<G1DepthCamera>());
  rclcpp::shutdown();
  return 0;
}
