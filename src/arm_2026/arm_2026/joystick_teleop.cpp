#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class ArmJoystickTeleop : public rclcpp::Node
{
public:
  ArmJoystickTeleop()
  : Node("arm_2026_joystick_teleop")
  {
    declare_parameters();

    command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      position_command_topic_, 10);

    actuator_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      actuator_command_topic_, 10);

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_, 10,
      std::bind(&ArmJoystickTeleop::joy_callback, this, std::placeholders::_1));

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&ArmJoystickTeleop::joint_state_callback, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / update_rate_hz_),
      std::bind(&ArmJoystickTeleop::update, this));

    RCLCPP_INFO(get_logger(), "Arm joystick teleop started.");
    RCLCPP_INFO(get_logger(), "Controls:");
    RCLCPP_INFO(get_logger(), "  Trigger: deadman");
    RCLCPP_INFO(get_logger(), "  Stick X: shoulder");
    RCLCPP_INFO(get_logger(), "  Stick Y: elbow");
    RCLCPP_INFO(get_logger(), "  Twist: base");
    RCLCPP_INFO(get_logger(), "  Hat up/down: wrist roll");
    RCLCPP_INFO(get_logger(), "  Hat left/right: wrist twist");
    RCLCPP_INFO(get_logger(), "  Button 5: open gripper");
    RCLCPP_INFO(get_logger(), "  Button 6: close gripper");
  }

private:
  void declare_parameters()
  {
    joy_topic_ = declare_parameter<std::string>("joy_topic", "/joy");
    position_command_topic_ = declare_parameter<std::string>(
      "position_command_topic", "/position_controller/commands");
    actuator_command_topic_ = declare_parameter<std::string>(
      "actuator_command_topic", "/arm_actuator_drive_cmd");

    update_rate_hz_ = declare_parameter<double>("update_rate_hz", 30.0);
    deadzone_ = declare_parameter<double>("deadzone", 0.4);

    // Logitech Extreme 3D Pro common Linux /joy mapping.
    stick_x_axis_ = declare_parameter<int>("stick_x_axis", 0);
    stick_y_axis_ = declare_parameter<int>("stick_y_axis", 1);
    twist_axis_ = declare_parameter<int>("twist_axis", 2);
    throttle_axis_ = declare_parameter<int>("throttle_axis", 3);
    hat_x_axis_ = declare_parameter<int>("hat_x_axis", 4);
    hat_y_axis_ = declare_parameter<int>("hat_y_axis", 5);

    deadman_button_ = declare_parameter<int>("deadman_button", 0);
    gripper_open_button_ = declare_parameter<int>("gripper_open_button", 4);
    gripper_close_button_ = declare_parameter<int>("gripper_close_button", 5);
    stop_button_ = declare_parameter<int>("stop_button", 7);
    home_button_ = declare_parameter<int>("home_button", 6);

    base_speed_rad_s_ = declare_parameter<double>("base_speed_rad_s", 0.50);
    wrist_roll_speed_rad_s_ = declare_parameter<double>("wrist_roll_speed_rad_s", 0.60);
    wrist_twist_speed_rad_s_ = declare_parameter<double>("wrist_twist_speed_rad_s", 0.60);
    claw_speed_rad_s_ = declare_parameter<double>("claw_speed_rad_s", 0.80);

    shoulder_drive_scale_ = declare_parameter<double>("shoulder_drive_scale", 1.0);
    elbow_drive_scale_ = declare_parameter<double>("elbow_drive_scale", 1.0);

    base_min_ = declare_parameter<double>("base_min", -1.5708);
    base_max_ = declare_parameter<double>("base_max", 1.5708);
    shoulder_min_ = declare_parameter<double>("shoulder_min", -1.0);
    shoulder_max_ = declare_parameter<double>("shoulder_max", 1.0);
    elbow_min_ = declare_parameter<double>("elbow_min", -1.0);
    elbow_max_ = declare_parameter<double>("elbow_max", 1.0);
    wrist_roll_min_ = declare_parameter<double>("wrist_roll_min", -1.5708);
    wrist_roll_max_ = declare_parameter<double>("wrist_roll_max", 1.5708);
    wrist_twist_min_ = declare_parameter<double>("wrist_twist_min", -1.5708);
    wrist_twist_max_ = declare_parameter<double>("wrist_twist_max", 1.5708);
    claw_min_ = declare_parameter<double>("claw_min", -1.0);
    claw_max_ = declare_parameter<double>("claw_max", 1.0);

    home_positions_[0] = declare_parameter<double>("home_base_yaw", 0.0);
    home_positions_[1] = declare_parameter<double>("home_shoulder_extension", 0.0);
    home_positions_[2] = declare_parameter<double>("home_elbow_extension", 0.0);
    home_positions_[3] = declare_parameter<double>("home_wrist_roll", 0.0);
    home_positions_[4] = declare_parameter<double>("home_wrist_twist", 0.0);
    home_positions_[5] = declare_parameter<double>("home_claw", 0.0);

    joint_mins_ = {base_min_, shoulder_min_, elbow_min_, wrist_roll_min_, wrist_twist_min_, claw_min_};
    joint_maxs_ = {base_max_, shoulder_max_, elbow_max_, wrist_roll_max_, wrist_twist_max_, claw_max_};
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    latest_joy_ = msg;
    last_joy_time_ = now();
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (have_joint_state_) {
      return;
    }

    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (i >= msg->position.size()) {
        continue;
      }

      const auto & name = msg->name[i];
      const double pos = msg->position[i];

      if (name == "base_yaw") {
        target_positions_[0] = pos;
      } else if (name == "wrist_roll") {
        target_positions_[3] = pos;
      } else if (name == "wrist_twist") {
        target_positions_[4] = pos;
      } else if (name == "claw") {
        target_positions_[5] = pos;
      }
    }

    target_positions_[1] = 0.0;
    target_positions_[2] = 0.0;

    have_joint_state_ = true;
  }

  double get_axis(const sensor_msgs::msg::Joy::SharedPtr & joy, int index) const
  {
    if (!joy || index < 0 || static_cast<size_t>(index) >= joy->axes.size()) {
      return 0.0;
    }

    double value = joy->axes[index];

    if (std::fabs(value) < deadzone_) {
      return 0.0;
    }

    return std::clamp(value, -1.0, 1.0);
  }

  bool get_button(const sensor_msgs::msg::Joy::SharedPtr & joy, int index) const
  {
    if (!joy || index < 0 || static_cast<size_t>(index) >= joy->buttons.size()) {
      return false;
    }

    return joy->buttons[index] != 0;
  }

  void update()
  {
    const auto current_time = now();
    const double dt = std::clamp((current_time - last_update_time_).seconds(), 0.0, 0.1);
    last_update_time_ = current_time;

    if (!latest_joy_) {
      publish_zero_actuators();
      return;
    }

    const bool deadman = get_button(latest_joy_, deadman_button_);
    const bool stop = get_button(latest_joy_, stop_button_);
    const bool home = get_button(latest_joy_, home_button_);

    if (!deadman || stop) {
      target_positions_[1] = 0.0;
      target_positions_[2] = 0.0;
      publish_zero_actuators();
      publish_position_target();
      return;
    }

    if (home) {
      target_positions_ = home_positions_;
      publish_zero_actuators();
      publish_position_target();
      return;
    }

    // Diagram mapping:
    // Stick X -> shoulder
    // Stick Y -> elbow
    // Twist -> base
    // Hat up/down -> wrist roll
    // Hat left/right -> wrist twist
    // Button 5/6 -> gripper open/close

    const double shoulder_cmd = get_axis(latest_joy_, stick_x_axis_) * shoulder_drive_scale_;
    const double elbow_cmd = get_axis(latest_joy_, stick_y_axis_) * elbow_drive_scale_;
    const double base_cmd = get_axis(latest_joy_, twist_axis_);
    const double wrist_roll_cmd = get_axis(latest_joy_, hat_y_axis_);
    const double wrist_twist_cmd = get_axis(latest_joy_, hat_x_axis_);

    const bool gripper_open = get_button(latest_joy_, gripper_open_button_);
    const bool gripper_close = get_button(latest_joy_, gripper_close_button_);

    double claw_cmd = 0.0;
    if (gripper_open && !gripper_close) {
      claw_cmd = 1.0;
    } else if (gripper_close && !gripper_open) {
      claw_cmd = -1.0;
    }

    // Phidget joints: integrate joystick input into position targets.
    target_positions_[0] += base_cmd * base_speed_rad_s_ * dt;
    target_positions_[3] += wrist_roll_cmd * wrist_roll_speed_rad_s_ * dt;
    target_positions_[4] += wrist_twist_cmd * wrist_twist_speed_rad_s_ * dt;
    target_positions_[5] += claw_cmd * claw_speed_rad_s_ * dt;

    // Shoulder and elbow match keyboard teleop behavior:
    // command is ±1 while input is active, 0 when released.
    target_positions_[1] = std::clamp(shoulder_cmd, -1.0, 1.0);
    target_positions_[2] = std::clamp(elbow_cmd, -1.0, 1.0);

    clamp_targets();

    publish_position_target();
    publish_actuator_command(shoulder_cmd, elbow_cmd);
  }

  void clamp_targets()
  {
    for (size_t i = 0; i < target_positions_.size(); ++i) {
      target_positions_[i] = std::clamp(target_positions_[i], joint_mins_[i], joint_maxs_[i]);
    }
  }

  void publish_position_target()
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.assign(target_positions_.begin(), target_positions_.end());
    command_pub_->publish(msg);
  }

  void publish_actuator_command(double shoulder_cmd, double elbow_cmd)
  {
    std_msgs::msg::Float32MultiArray msg;
    msg.data = {
      static_cast<float>(std::clamp(shoulder_cmd, -1.0, 1.0)),
      static_cast<float>(std::clamp(elbow_cmd, -1.0, 1.0))
    };
    actuator_pub_->publish(msg);
  }

  void publish_zero_actuators()
  {
    publish_actuator_command(0.0, 0.0);
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr actuator_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::Joy::SharedPtr latest_joy_;

  std::string joy_topic_;
  std::string position_command_topic_;
  std::string actuator_command_topic_;

  double update_rate_hz_{30.0};
  double deadzone_{0.10};

  int stick_x_axis_{0};
  int stick_y_axis_{1};
  int twist_axis_{2};
  int throttle_axis_{3};
  int hat_x_axis_{4};
  int hat_y_axis_{5};

  int deadman_button_{0};
  int gripper_open_button_{4};
  int gripper_close_button_{5};
  int stop_button_{7};
  int home_button_{6};

  double base_speed_rad_s_{0.50};
  double wrist_roll_speed_rad_s_{0.60};
  double wrist_twist_speed_rad_s_{0.60};
  double claw_speed_rad_s_{0.80};

  double shoulder_drive_scale_{1.0};
  double elbow_drive_scale_{1.0};

  double base_min_{-1.5708};
  double base_max_{1.5708};
  double shoulder_min_{-1.0};
  double shoulder_max_{1.0};
  double elbow_min_{-1.0};
  double elbow_max_{1.0};
  double wrist_roll_min_{-1.5708};
  double wrist_roll_max_{1.5708};
  double wrist_twist_min_{-1.5708};
  double wrist_twist_max_{1.5708};
  double claw_min_{-1.0};
  double claw_max_{1.0};

  bool have_joint_state_{false};

  std::array<double, 6> target_positions_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 6> home_positions_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 6> joint_mins_{-1.5708, -1.0, -1.0, -1.5708, -1.5708, -1.0};
  std::array<double, 6> joint_maxs_{1.5708, 1.0, 1.0, 1.5708, 1.5708, 1.0};

  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_joy_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmJoystickTeleop>());
  rclcpp::shutdown();
  return 0;
}