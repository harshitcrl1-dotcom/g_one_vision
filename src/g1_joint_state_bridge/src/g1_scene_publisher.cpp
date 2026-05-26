#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

class G1ScenePublisher : public rclcpp::Node {
public:
  G1ScenePublisher() : Node("g1_scene_publisher") {
    pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/scene_markers", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
      [this]() { publish_scene(); });
    RCLCPP_INFO(this->get_logger(), "Scene publisher started");
  }

private:
  int id_ = 0;

  void add_box(visualization_msgs::msg::MarkerArray &arr,
               const std::string &ns,
               double x, double y, double z,
               double sx, double sy, double sz,
               float r, float g, float b, float a = 1.0,
               double rx=0, double ry=0, double rz=0, double rw=1) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "odom";
    m.header.stamp = this->now();
    m.ns = ns; m.id = id_++;
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = z;
    m.pose.orientation.x = rx; m.pose.orientation.y = ry;
    m.pose.orientation.z = rz; m.pose.orientation.w = rw;
    m.scale.x = sx*2; m.scale.y = sy*2; m.scale.z = sz*2;
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
    arr.markers.push_back(m);
  }

  void add_cylinder(visualization_msgs::msg::MarkerArray &arr,
                    const std::string &ns,
                    double x, double y, double z,
                    double radius, double half_h,
                    float r, float g, float b, float a = 1.0) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "odom";
    m.header.stamp = this->now();
    m.ns = ns; m.id = id_++;
    m.type = visualization_msgs::msg::Marker::CYLINDER;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = z;
    m.pose.orientation.w = 1.0;
    m.scale.x = radius*2; m.scale.y = radius*2; m.scale.z = half_h*2;
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
    arr.markers.push_back(m);
  }

  void add_sphere(visualization_msgs::msg::MarkerArray &arr,
                  const std::string &ns,
                  double x, double y, double z, double radius,
                  float r, float g, float b, float a = 1.0) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "odom";
    m.header.stamp = this->now();
    m.ns = ns; m.id = id_++;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = z;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = radius*2;
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
    arr.markers.push_back(m);
  }

  void publish_scene() {
    id_ = 0;
    visualization_msgs::msg::MarkerArray arr;

    // Floor
    add_box(arr,"floor",  0,0,0,  8,10,0.1,  0.15,0.15,0.15, 0.4);
    // Walls
    add_box(arr,"walls",  0, 10,3.0,  8,0.1,3.0,  0.85,0.85,0.85,0.3);
    add_box(arr,"walls",  0,-10,3.0,  8,0.1,3.0,  0.85,0.85,0.85,0.3);
    add_box(arr,"walls",  8,  0,3.0,  0.1,10,3.0, 0.85,0.85,0.85,0.3);
    add_box(arr,"walls", -8,  0,3.0,  0.1,10,3.0, 0.85,0.85,0.85,0.3);
    // Ceiling
    add_box(arr,"ceiling", 0,0,6.0, 8,10,0.1, 0.95,0.95,0.95,0.15);
    // AC units
    add_box(arr,"ac",  0, 9.9,4.5, 1.2,0.2,0.4, 0.5,0.5,0.5);
    add_box(arr,"ac",  0,-9.9,4.5, 1.2,0.2,0.4, 0.5,0.5,0.5);

    // Conference table (body pos=0,-5,0)
    add_box(arr,"furniture", 0,-5,0.75,   1.5,3,0.03,  0.6,0.4,0.2);
    add_cylinder(arr,"furniture", 0+1.2,-5+2.7,0.375, 0.05,0.375, 0.5,0.5,0.5);
    add_cylinder(arr,"furniture", 0-1.2,-5+2.7,0.375, 0.05,0.375, 0.5,0.5,0.5);
    add_cylinder(arr,"furniture", 0+1.2,-5-2.7,0.375, 0.05,0.375, 0.5,0.5,0.5);
    add_cylinder(arr,"furniture", 0-1.2,-5-2.7,0.375, 0.05,0.375, 0.5,0.5,0.5);
    // Conf chairs
    add_box(arr,"chairs", 0+1.8,-5+1.5,0.45, 0.25,0.25,0.02, 0.2,0.2,0.2);
    add_box(arr,"chairs", 0-1.8,-5+1.5,0.45, 0.25,0.25,0.02, 0.2,0.2,0.2);

    // L-desk (body pos=-5,6,0)
    add_box(arr,"furniture", -5+0,-6+2,0.75, 2,0.4,0.02, 0.6,0.4,0.2);
    add_box(arr,"furniture", -5-1.6,6+0.5,0.75, 0.4,1.5,0.02, 0.6,0.4,0.2);
    add_box(arr,"chairs",    -5-0.5,6+1.5,0.45, 0.25,0.25,0.02, 0.2,0.2,0.2);

    // Std desks (body pos=5,6,0)
    add_box(arr,"furniture", 5,6+2,0.75, 0.8,1.2,0.02, 0.6,0.4,0.2);
    add_box(arr,"furniture", 5,6+0,0.75, 0.8,1.2,0.02, 0.6,0.4,0.2);
    add_box(arr,"chairs",    5+0.5,6+1.5,0.45, 0.25,0.25,0.02, 0.2,0.2,0.2);
    add_box(arr,"chairs",    5+0.5,6-0.5,0.45, 0.25,0.25,0.02, 0.2,0.2,0.2);

    // Humanoid A (pos=-5.5,7,0.45)
    add_sphere(arr,"humans", -5.5,7.0,0.45+0.65, 0.1, 0.9,0.7,0.6);  // head
    add_box(arr,"humans",    -5.5,7.0,0.45+0.3,  0.12,0.12,0.2, 0.2,0.4,0.6); // torso

    // Humanoid B (pos=5.5,7,0.45)
    add_sphere(arr,"humans", 5.5,7.0,0.45+0.65, 0.1, 0.6,0.4,0.3);
    add_box(arr,"humans",    5.5,7.0,0.45+0.3,  0.12,0.12,0.2, 0.6,0.3,0.3);

    pub_->publish(arr);
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<G1ScenePublisher>());
  rclcpp::shutdown();
  return 0;
}
