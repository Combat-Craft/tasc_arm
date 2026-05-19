#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace
{

class JoystickTeleop : public rclcpp::Node
{
public:
  JoystickTeleop()
  : Node("arm_2026_joystick_teleop")
  {
    publish_controller_commands_ = this->declare_parameter<bool>(
      "publish_controller_commands", true);
    publish_direct_actuator_cmd_ = this->declare_parameter<bool>(
      "publish_direct_actuator_cmd", true);
    require_joint_state_before_motion_ = this->declare_parameter<bool>(
      "require_joint_state_before_motion", false);

    base_axis_scale_ = this->declare_parameter<double>("base_axis_scale", 0.8);
    wrist_axis_scale_ = this->declare_parameter<double>("wrist_axis_scale", 0.8);
    shoulder_button_drive_ = this->declare_parameter<double>("shoulder_button_drive", 0.5);
    elbow_axis_drive_ = this->declare_parameter<double>("elbow_axis_drive", 0.5);

    command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/position_controller/commands", 10);

    actuator_cmd_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      "/arm_actuator_drive_cmd", 10);

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&JoystickTeleop::joint_state_callback, this, std::placeholders::_1));

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&JoystickTeleop::joy_callback, this, std::placeholders::_1));

    last_loop_time_ = std::chrono::steady_clock::now();

    print_help();
  }

  void spin()
  {
    rclcpp::WallRate loop_rate(30.0);

    while (rclcpp::ok()) {
      rclcpp::spin_some(shared_from_this());
      update_from_joystick();
      loop_rate.sleep();
    }

    actuator_drive_cmd_ = {0.0f, 0.0f};
    publish_actuator_drive_command();
  }

private:
  void print_help() const
  {
    std::printf(
      "\n"
      "Joystick teleop for arm_2026\n"
      "Logitech Extreme 3D mapping:\n"
      "  Axis 2 : base rotation (counterclockwise=+, clockwise=-)\n"
      "  Axis 1 : elbow actuator trigger-style\n"
      "           <= 0.25 => -1, >= 0.75 => +1, else 0\n"
      "  Axis 5 : wrist_roll\n"
      "  Axis 4 : wrist_twist\n"
      "  Axis 3 : step size control (higher axis => bigger step size)\n"
      "  Button 5 : shoulder extend\n"
      "  Button 3 : shoulder retract\n"
      "\n");
    print_status();
  }

  void print_status() const
  {
    std::printf(
      "Target [rad] base=%.3f shoulder=%.3f elbow=%.3f wrist_roll=%.3f wrist_twist=%.3f | step=%.3f rad (%.1f deg)\n",
      target_positions_[0],
      target_positions_[1],
      target_positions_[2],
      target_positions_[3],
      target_positions_[4],
      step_size_rad_,
      step_size_rad_ * 180.0 / M_PI);
    std::fflush(stdout);
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    const std::array<std::string, 5> joint_names{
      "base_yaw",
      "shoulder_extension",
      "elbow_extension",
      "wrist_roll",
      "wrist_twist"};

    bool any_joint_updated = false;
    for (std::size_t target_index = 0; target_index < joint_names.size(); ++target_index) {
      for (std::size_t msg_index = 0; msg_index < msg->name.size(); ++msg_index) {
        if (msg->name[msg_index] == joint_names[target_index] &&
            msg_index < msg->position.size()) {
          measured_positions_[target_index] = msg->position[msg_index];
          any_joint_updated = true;
          break;
        }
      }
    }

    if (!have_joint_state_ && any_joint_updated) {
      target_positions_ = measured_positions_;
      have_joint_state_ = true;
      RCLCPP_INFO(get_logger(), "Initialized joystick teleop target from /joint_states.");
      print_status();
    }
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    last_joy_msg_ = msg;
    have_joy_msg_ = true;
  }

  static double threshold_axis(double value)
  {
    if (value >= 0.5) {
      return 1.0;
    }
    if (value <= -0.5) {
      return -1.0;
    }
    return 0.0;
  }

  static double threshold_trigger_axis(double value)
  {
    if (value >= 0.75) {
      return 1.0;
    }
    if (value <= 0.25) {
      return -1.0;
    }
    return 0.0;
  }

  void update_step_size_from_axis3(double axis3)
  {
    const double normalized = (axis3 + 1.0) * 0.5;  // [-1,1] -> [0,1]
    step_size_rad_ =
      min_step_size_rad_ + normalized * (max_step_size_rad_ - min_step_size_rad_);
  }

  void update_from_joystick()
  {
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - last_loop_time_).count();
    last_loop_time_ = now;

    if (!have_joy_msg_) {
      return;
    }

    if (!have_joint_state_ && require_joint_state_before_motion_) {
      return;
    }

    const auto & joy = last_joy_msg_;

    double axis1 = 0.0;
    double axis2 = 0.0;
    double axis3 = 0.0;
    double axis4 = 0.0;
    double axis5 = 0.0;

    if (joy->axes.size() > 1) axis1 = joy->axes[1];
    if (joy->axes.size() > 2) axis2 = joy->axes[2];
    if (joy->axes.size() > 3) axis3 = joy->axes[3];
    if (joy->axes.size() > 4) axis4 = joy->axes[4];
    if (joy->axes.size() > 5) axis5 = joy->axes[5];

    int button3 = 0;
    int button5 = 0;
    if (joy->buttons.size() > 3) button3 = joy->buttons[3];
    if (joy->buttons.size() > 5) button5 = joy->buttons[5];

    const double base_cmd = threshold_axis(axis2);
    const double wrist_twist_cmd = threshold_axis(axis4);
    const double wrist_roll_cmd = threshold_axis(axis5);
    const double elbow_cmd = threshold_trigger_axis(axis1);

    update_step_size_from_axis3(axis3);

    bool changed = false;

    // Base rotation: axis 2, CCW=+, CW=-
    if (std::fabs(base_cmd) > 0.0) {
      target_positions_[0] = std::clamp(
        target_positions_[0] + dt * base_axis_scale_ * base_cmd,
        joint_mins_[0], joint_maxs_[0]);
      changed = true;
    }

    // Wrist twist: axis 4
    if (std::fabs(wrist_twist_cmd) > 0.0) {
      target_positions_[4] = std::clamp(
        target_positions_[4] + step_size_rad_ * wrist_twist_cmd * wrist_gain_ratio_,
        joint_mins_[4], joint_maxs_[4]);
      changed = true;
    }

    // Wrist roll: axis 5
    if (std::fabs(wrist_roll_cmd) > 0.0) {
      target_positions_[3] = std::clamp(
        target_positions_[3] + step_size_rad_ * wrist_roll_cmd * wrist_gain_ratio_,
        joint_mins_[3], joint_maxs_[3]);
      changed = true;
    }

    // Shoulder buttons
    float shoulder_cmd = 0.0f;
    if (button5) {
      shoulder_cmd = static_cast<float>(shoulder_button_drive_);
    } else if (button3) {
      shoulder_cmd = static_cast<float>(-shoulder_button_drive_);
    }

    actuator_drive_cmd_[0] = shoulder_cmd;
    actuator_drive_cmd_[1] = static_cast<float>(elbow_cmd * elbow_axis_drive_);

    if (std::fabs(actuator_drive_cmd_[0]) > 0.0f) {
      target_positions_[1] = std::clamp(
        target_positions_[1] + shoulder_actuator_sim_speed_rad_per_sec_ *
        static_cast<double>(actuator_drive_cmd_[0]) * dt,
        joint_mins_[1], joint_maxs_[1]);
      changed = true;
    }

    if (std::fabs(actuator_drive_cmd_[1]) > 0.0f) {
      target_positions_[2] = std::clamp(
        target_positions_[2] + elbow_actuator_sim_speed_rad_per_sec_ *
        static_cast<double>(actuator_drive_cmd_[1]) * dt,
        joint_mins_[2], joint_maxs_[2]);
      changed = true;
    }

    publish_actuator_drive_command();

    if (changed) {
      publish_target();
    }
  }

  void publish_actuator_drive_command()
  {
    if (!publish_direct_actuator_cmd_ || !actuator_cmd_pub_) {
      return;
    }

    std_msgs::msg::Float32MultiArray actuator_msg;
    actuator_msg.data.assign(actuator_drive_cmd_.begin(), actuator_drive_cmd_.end());
    actuator_cmd_pub_->publish(actuator_msg);
  }

  void publish_target()
  {
    if (publish_controller_commands_ && command_pub_) {
      std_msgs::msg::Float64MultiArray msg;
      msg.data.assign(target_positions_.begin(), target_positions_.end());
      command_pub_->publish(msg);
    }

    print_status();
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr actuator_cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  sensor_msgs::msg::Joy::SharedPtr last_joy_msg_;
  bool have_joy_msg_{false};

  std::array<double, 5> measured_positions_{{0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 5> target_positions_{{0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<float, 2> actuator_drive_cmd_{{0.0f, 0.0f}};

  const std::array<double, 5> joint_mins_{{-3.14, -1.57, -1.57, -1.57, -1.57}};
  const std::array<double, 5> joint_maxs_{{ 3.14,  1.57,  1.57,  1.57,  1.57}};

  const double base_phidget_scale_deg_per_count_{0.00146103896};
  const double wrist_phidget_scale_deg_per_count_{0.00416666667};
  const double wrist_gain_ratio_{wrist_phidget_scale_deg_per_count_ / base_phidget_scale_deg_per_count_};

  const double shoulder_actuator_sim_speed_rad_per_sec_{0.5};
  const double elbow_actuator_sim_speed_rad_per_sec_{0.3};

  double axis_deadzone_{0.15};
  double base_axis_scale_{0.8};
  double wrist_axis_scale_{0.8};
  double shoulder_button_drive_{0.5};
  double elbow_axis_drive_{0.5};

  double step_size_rad_{5.0 * M_PI / 180.0};
  const double min_step_size_rad_{1.0 * M_PI / 180.0};
  const double max_step_size_rad_{45.0 * M_PI / 180.0};

  std::chrono::steady_clock::time_point last_loop_time_{};

  bool have_joint_state_{false};
  bool publish_controller_commands_{true};
  bool publish_direct_actuator_cmd_{true};
  bool require_joint_state_before_motion_{false};
};

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoystickTeleop>();
  node->spin();
  rclcpp::shutdown();
  return 0;
}
