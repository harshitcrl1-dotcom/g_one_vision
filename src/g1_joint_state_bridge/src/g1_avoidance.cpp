#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"

const int G1_NUM_MOTOR = 29;

class G1Avoidance : public rclcpp::Node {
public:
  G1Avoidance() : Node("g1_avoidance") {
    cmd_pub_ = this->create_publisher<unitree_hg::msg::LowCmd>("lowcmd", 10);

    // Get current joint positions from lowstate
    state_sub_ = this->create_subscription<unitree_hg::msg::LowState>(
      "lowstate", 10,
      [this](const unitree_hg::msg::LowState::SharedPtr msg) {
        for (int i = 0; i < G1_NUM_MOTOR; i++)
          current_q_[i] = msg->motor_state[i].q;
        state_received_ = true;
      });

    alert_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/obstacle_alert", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        if (!state_received_) return;

        if (msg->data.substr(0, 4) == "STOP") {
          if (!stopped_) {
            RCLCPP_WARN(this->get_logger(), "OBSTACLE DETECTED - FREEZING ROBOT");
            stopped_ = true;
          }
          send_hold_cmd();
        } else {
          if (stopped_) {
            RCLCPP_INFO(this->get_logger(), "Path clear - releasing");
            stopped_ = false;
          }
        }
      });

    RCLCPP_INFO(this->get_logger(), "G1 avoidance node started");
  }

private:
  void send_hold_cmd() {
    auto cmd = unitree_hg::msg::LowCmd();
    cmd.mode_pr = 0;
    cmd.mode_machine = 0;

    for (int i = 0; i < G1_NUM_MOTOR; i++) {
      cmd.motor_cmd[i].mode = 1;  // servo mode
      cmd.motor_cmd[i].q   = current_q_[i]; // hold current position
      cmd.motor_cmd[i].dq  = 0.0;
      cmd.motor_cmd[i].tau = 0.0;
      cmd.motor_cmd[i].kp  = 60.0;
      cmd.motor_cmd[i].kd  = 2.0;
    }
    cmd_pub_->publish(cmd);
  }

  rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr cmd_pub_;
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr alert_sub_;

  float current_q_[G1_NUM_MOTOR] = {0};
  bool state_received_ = false;
  bool stopped_ = false;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<G1Avoidance>());
  rclcpp::shutdown();
  return 0;
}
