#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class ArmJoystickTeleop : public rclcpp::Node
{
public:
  ArmJoystickTeleop()
  : Node("arm_2026_joystick_teleop")
  {
    declare_parameters();

    command_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(
        command_topic_,
        10);

    joy_sub_ =
      create_subscription<sensor_msgs::msg::Joy>(
        joy_topic_,
        10,
        std::bind(
          &ArmJoystickTeleop::joy_callback,
          this,
          std::placeholders::_1));

    timer_ =
      create_wall_timer(
        std::chrono::duration<double>(1.0 / update_rate_hz_),
        std::bind(&ArmJoystickTeleop::update, this));

    last_joy_time_ = now();

    RCLCPP_INFO(get_logger(), "Arm velocity joystick teleop started");
    RCLCPP_INFO(get_logger(), "Publishing to: %s", command_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Hold button %d as deadman", deadman_button_);
    RCLCPP_INFO(get_logger(), "Stick forward/back: shoulder");
    RCLCPP_INFO(
      get_logger(),
      "Hold button %d + stick forward/back: elbow",
      elbow_mode_button_);
    RCLCPP_INFO(get_logger(), "Twist: base");
    RCLCPP_INFO(get_logger(), "Hat left/right: wrist roll");
    RCLCPP_INFO(get_logger(), "Hat forward/back: wrist twist");
    RCLCPP_INFO(get_logger(), "Button %d: open claw", gripper_open_button_);
    RCLCPP_INFO(get_logger(), "Button %d: close claw", gripper_close_button_);
    RCLCPP_INFO(get_logger(), "Button %d: emergency stop", stop_button_);
  }

  ~ArmJoystickTeleop() override
  {
    publish_stop();
  }

private:
  void declare_parameters()
  {
    joy_topic_ =
      declare_parameter<std::string>(
        "joy_topic",
        "/joy");

    command_topic_ =
      declare_parameter<std::string>(
        "command_topic",
        "/manual_controller/commands");

    update_rate_hz_ =
      declare_parameter<double>(
        "update_rate_hz",
        30.0);

    joy_timeout_sec_ =
      declare_parameter<double>(
        "joy_timeout_sec",
        0.5);

    deadzone_ =
      declare_parameter<double>(
        "deadzone",
        0.15);

    base_deadzone_ =
      declare_parameter<double>(
        "base_deadzone",
        0.45);

    // Logitech Extreme 3D Pro mapping
    stick_x_axis_ =
      declare_parameter<int>(
        "stick_x_axis",
        0);

    stick_y_axis_ =
      declare_parameter<int>(
        "stick_y_axis",
        1);

    twist_axis_ =
      declare_parameter<int>(
        "twist_axis",
        2);

    hat_x_axis_ =
      declare_parameter<int>(
        "hat_x_axis",
        4);

    hat_y_axis_ =
      declare_parameter<int>(
        "hat_y_axis",
        5);

    deadman_button_ =
      declare_parameter<int>(
        "deadman_button",
        0);

    elbow_mode_button_ =
      declare_parameter<int>(
        "elbow_mode_button",
        1);

    gripper_open_button_ =
      declare_parameter<int>(
        "gripper_open_button",
        4);

    gripper_close_button_ =
      declare_parameter<int>(
        "gripper_close_button",
        5);

    stop_button_ =
      declare_parameter<int>(
        "stop_button",
        7);

    base_speed_rad_s_ =
      declare_parameter<double>(
        "base_speed_rad_s",
        0.25);

    shoulder_speed_rad_s_ =
      declare_parameter<double>(
        "shoulder_speed_rad_s",
        1.0);

    elbow_speed_rad_s_ =
      declare_parameter<double>(
        "elbow_speed_rad_s",
        1.0);

    wrist_roll_speed_rad_s_ =
      declare_parameter<double>(
        "wrist_roll_speed_rad_s",
        0.60);

    wrist_twist_speed_rad_s_ =
      declare_parameter<double>(
        "wrist_twist_speed_rad_s",
        0.60);

    claw_speed_rad_s_ =
      declare_parameter<double>(
        "claw_speed_rad_s",
        1.0);

    base_direction_ =
      declare_parameter<double>(
        "base_direction",
        -1.0);

    shoulder_direction_ =
      declare_parameter<double>(
        "shoulder_direction",
        1.0);

    elbow_direction_ =
      declare_parameter<double>(
        "elbow_direction",
        1.0);

    wrist_roll_direction_ =
      declare_parameter<double>(
        "wrist_roll_direction",
        1.0);

    wrist_twist_direction_ =
      declare_parameter<double>(
        "wrist_twist_direction",
        1.0);

    claw_direction_ =
      declare_parameter<double>(
        "claw_direction",
        1.0);
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    latest_joy_ = msg;
    last_joy_time_ = now();
  }

  double get_axis(
    const sensor_msgs::msg::Joy::SharedPtr & joy,
    int index,
    double deadzone) const
  {
    if (
      !joy ||
      index < 0 ||
      static_cast<std::size_t>(index) >= joy->axes.size())
    {
      return 0.0;
    }

    const double value =
      std::clamp(
        static_cast<double>(joy->axes[index]),
        -1.0,
        1.0);

    if (std::fabs(value) < deadzone)
    {
      return 0.0;
    }

    return value;
  }

  bool get_button(
    const sensor_msgs::msg::Joy::SharedPtr & joy,
    int index) const
  {
    if (
      !joy ||
      index < 0 ||
      static_cast<std::size_t>(index) >= joy->buttons.size())
    {
      return false;
    }

    return joy->buttons[index] != 0;
  }

  void update()
  {
    if (!latest_joy_)
    {
      publish_stop();
      return;
    }

    const double joy_age =
      (now() - last_joy_time_).seconds();

    if (joy_age > joy_timeout_sec_)
    {
      publish_stop();

      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Joystick data timed out. Publishing zero commands.");

      return;
    }

    const bool deadman =
      get_button(
        latest_joy_,
        deadman_button_);

    const bool stop =
      get_button(
        latest_joy_,
        stop_button_);

    if (!deadman || stop)
    {
      publish_stop();
      return;
    }

    const double forward_back_input =
      get_axis(
        latest_joy_,
        stick_y_axis_,
        deadzone_);

    const bool elbow_mode =
      get_button(
        latest_joy_,
        elbow_mode_button_);

    double shoulder_input = 0.0;
    double elbow_input = 0.0;

    if (elbow_mode){
      elbow_input = forward_back_input;
      }
    else
    {
      shoulder_input = forward_back_input;
    }

    const double base_input =
      get_axis(
        latest_joy_,
        twist_axis_,
        base_deadzone_);

    const double wrist_roll_input =
      get_axis(
        latest_joy_,
        hat_x_axis_,
        deadzone_);

    const double wrist_twist_input =
      get_axis(
        latest_joy_,
        hat_y_axis_,
        deadzone_);

    const bool claw_open =
      get_button(
        latest_joy_,
        gripper_open_button_);

    const bool claw_close =
      get_button(
        latest_joy_,
        gripper_close_button_);

    double claw_input = 0.0;

    if (claw_open && !claw_close)
    {
      claw_input = 1.0;
    }
    else if (claw_close && !claw_open)
    {
      claw_input = -1.0;
    }

    const std::array<double, 6> velocity_commands{
      base_input *
        base_speed_rad_s_ *
        base_direction_,

      shoulder_input *
        shoulder_speed_rad_s_ *
        shoulder_direction_,

      elbow_input *
        elbow_speed_rad_s_ *
        elbow_direction_,

      wrist_roll_input *
        wrist_roll_speed_rad_s_ *
        wrist_roll_direction_,

      wrist_twist_input *
        wrist_twist_speed_rad_s_ *
        wrist_twist_direction_,

      claw_input *
        claw_speed_rad_s_ *
        claw_direction_
    };

    publish_commands(velocity_commands);
  }

  void publish_commands(
    const std::array<double, 6> & commands)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.assign(commands.begin(), commands.end());
    command_pub_->publish(msg);
  }

  void publish_stop()
  {
    static const std::array<double, 6> zero_commands{
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0
    };

    publish_commands(zero_commands);
  }

  rclcpp::Publisher<
    std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;

  rclcpp::Subscription<
    sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::Joy::SharedPtr latest_joy_;

  std::string joy_topic_;
  std::string command_topic_;

  double update_rate_hz_{30.0};
  double joy_timeout_sec_{0.5};

  double deadzone_{0.15};
  double base_deadzone_{0.45};

  int stick_x_axis_{0};
  int stick_y_axis_{1};
  int twist_axis_{2};
  int hat_x_axis_{4};
  int hat_y_axis_{5};

  int deadman_button_{0};
  int elbow_mode_button_{1};
  int gripper_open_button_{4};
  int gripper_close_button_{5};
  int stop_button_{7};

  double base_speed_rad_s_{0.25};
  double shoulder_speed_rad_s_{1.0};
  double elbow_speed_rad_s_{1.0};
  double wrist_roll_speed_rad_s_{0.60};
  double wrist_twist_speed_rad_s_{0.60};
  double claw_speed_rad_s_{1.0};

  double base_direction_{-1.0};
  double shoulder_direction_{1.0};
  double elbow_direction_{1.0};
  double wrist_roll_direction_{1.0};
  double wrist_twist_direction_{1.0};
  double claw_direction_{1.0};

  rclcpp::Time last_joy_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmJoystickTeleop>());
  rclcpp::shutdown();
  return 0;
}
