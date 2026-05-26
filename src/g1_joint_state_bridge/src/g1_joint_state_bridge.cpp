#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "unitree_hg/msg/low_state.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <vector>
#include <string>
#include <mutex>

const std::vector<std::string> G1_JOINT_NAMES = {
  "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
  "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
  "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
  "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
  "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
  "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
  "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
  "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
  "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
};

class G1JointStateBridge : public rclcpp::Node {
public:
  G1JointStateBridge() : Node("g1_joint_state_bridge") {
    pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    sport_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
      "sportmodestate", 10,
      [this](const unitree_go::msg::SportModeState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        x_ = msg->position[0];
        y_ = msg->position[1];
        z_ = msg->position[2];
      });

    low_sub_ = this->create_subscription<unitree_hg::msg::LowState>(
      "lowstate", 10,
      [this](const unitree_hg::msg::LowState::SharedPtr msg) {
        auto now = this->now();
        auto js = sensor_msgs::msg::JointState();
        js.header.stamp = now;
        js.name = G1_JOINT_NAMES;
        for (int i = 0; i < 29; i++) {
          js.position.push_back(msg->motor_state[i].q);
          js.velocity.push_back(msg->motor_state[i].dq);
          js.effort.push_back(msg->motor_state[i].tau_est);
        }
        pub_->publish(js);

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = now;
        tf.header.frame_id = "odom";
        tf.child_frame_id = "pelvis";
        {
          std::lock_guard<std::mutex> lock(mutex_);
          tf.transform.translation.x = x_;
          tf.transform.translation.y = y_;
          tf.transform.translation.z = z_;
        }
        tf.transform.rotation.w = msg->imu_state.quaternion[0];
        tf.transform.rotation.x = msg->imu_state.quaternion[1];
        tf.transform.rotation.y = msg->imu_state.quaternion[2];
        tf.transform.rotation.z = msg->imu_state.quaternion[3];
        tf_broadcaster_->sendTransform(tf);
      });
    RCLCPP_INFO(this->get_logger(), "G1 joint state bridge started");
  }
private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr low_sub_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sport_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::mutex mutex_;
  double x_ = 0.0, y_ = 0.0, z_ = 0.8;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<G1JointStateBridge>());
  rclcpp::shutdown();
  return 0;
}
// ADD THIS AS SEPARATE FILE
