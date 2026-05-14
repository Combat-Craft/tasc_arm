#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace
{

class TerminalGuard
{
public:
  TerminalGuard()
  {
    if (!isatty(STDIN_FILENO)) {
      return;
    }

    if (tcgetattr(STDIN_FILENO, &original_) != 0) {
      return;
    }

    termios raw = original_;
    raw.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;

    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) {
      active_ = true;
    }
  }

  ~TerminalGuard()
  {
    if (active_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &original_);
    }
  }

  TerminalGuard(const TerminalGuard &) = delete;
  TerminalGuard & operator=(const TerminalGuard &) = delete;

private:
  termios original_{};
  bool active_{false};
};

class KeyboardTeleop : public rclcpp::Node
{
public:
  KeyboardTeleop()
  : Node("arm_2026_keyboard_teleop")
  {
    publish_controller_commands_ = this->declare_parameter<bool>(
      "publish_controller_commands", true);
    publish_direct_actuator_cmd_ = this->declare_parameter<bool>(
      "publish_direct_actuator_cmd", true);
    require_joint_state_before_motion_ = this->declare_parameter<bool>(
      "require_joint_state_before_motion", false);

    command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/position_controller/commands", 10);

    actuator_cmd_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      "/arm_actuator_cmd", 10);

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&KeyboardTeleop::joint_state_callback, this, std::placeholders::_1));

    print_help();
  }

  void spin()
  {
    if (!isatty(STDIN_FILENO)) {
      RCLCPP_ERROR(get_logger(), "Keyboard teleop requires a real terminal (TTY).");
      return;
    }

    TerminalGuard terminal_guard;
    rclcpp::WallRate loop_rate(30.0);

    while (rclcpp::ok()) {
      rclcpp::spin_some(shared_from_this());

      char key = 0;
      if (read_key(key)) {
        if (!handle_key(key)) {
          break;
        }
      }

      loop_rate.sleep();
    }
  }

private:
  void print_help() const
  {
    std::printf(
      "\n"
      "Keyboard teleop for arm_2026\n"
      "Commands:\n"
      "  q/a : base + / -\n"
      "  w/s : shoulder + / -\n"
      "  e/d : elbow + / -\n"
      "  r/f : wrist_twist + / -\n"
      "  t/g : wrist_roll + / -\n"
      "  z/x : decrease / increase step size\n"
      "  space : publish current target again\n"
      "  h : show help\n"
      "  c : reset target to current measured joints\n"
      "  Ctrl-C : quit\n"
      "\n");
    std::printf(
      "Publish modes: /position_controller/commands=%s, /arm_actuator_cmd=%s, require_joint_state=%s\n",
      publish_controller_commands_ ? "on" : "off",
      publish_direct_actuator_cmd_ ? "on" : "off",
      require_joint_state_before_motion_ ? "on" : "off");
    print_status();
  }

  void print_status() const
  {
    std::printf(
      "Target [rad] base=%.3f shoulder=%.3f elbow=%.3f wrist_roll=%.3f wrist_twist=%.3f  |  step=%.3f rad (%.1f deg)\n",
      target_positions_[0],
      target_positions_[1],
      target_positions_[2],
      target_positions_[3],
      target_positions_[4],
      step_size_rad_,
      step_size_rad_ * 180.0 / M_PI);
    std::fflush(stdout);
  }

  bool read_key(char & key) const
  {
    fd_set read_set;
    FD_ZERO(&read_set);
    FD_SET(STDIN_FILENO, &read_set);

    timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    const int ready = select(STDIN_FILENO + 1, &read_set, nullptr, nullptr, &timeout);
    if (ready < 0) {
      if (errno != EINTR) {
        RCLCPP_WARN(
          get_logger(),
          "select() failed while reading keyboard input: %s",
          std::strerror(errno));
      }
      return false;
    }

    if (ready == 0) {
      return false;
    }

    const ssize_t bytes_read = ::read(STDIN_FILENO, &key, 1);
    return bytes_read == 1;
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
      RCLCPP_INFO(get_logger(), "Initialized teleop target from /joint_states.");
      print_status();
    }
  }

  bool handle_key(char key)
  {
    switch (key) {
      case 'q':
        nudge_joint(0, step_size_rad_);
        return true;
      case 'a':
        nudge_joint(0, -step_size_rad_);
        return true;
      case 'w':
        nudge_joint(1, step_size_rad_);
        return true;
      case 's':
        nudge_joint(1, -step_size_rad_);
        return true;
      case 'e':
        nudge_joint(2, step_size_rad_);
        return true;
      case 'd':
        nudge_joint(2, -step_size_rad_);
        return true;
      case 'r':
        nudge_joint(4, step_size_rad_);
        return true;
      case 'f':
        nudge_joint(4, -step_size_rad_);
        return true;
      case 't':
        nudge_joint(3, step_size_rad_);
        return true;
      case 'g':
        nudge_joint(3, -step_size_rad_);
        return true;
      case 'z':
        step_size_rad_ = std::max(min_step_size_rad_, step_size_rad_ / 2.0);
        print_status();
        return true;
      case 'x':
        step_size_rad_ = std::min(max_step_size_rad_, step_size_rad_ * 2.0);
        print_status();
        return true;
      case ' ':
        publish_target();
        return true;
      case 'c':
        target_positions_ = measured_positions_;
        publish_target();
        return true;
      case 'h':
        print_help();
        return true;
      default:
        return true;
    }
  }

  void nudge_joint(std::size_t joint_index, double delta)
  {
    if (!have_joint_state_ && require_joint_state_before_motion_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for /joint_states before accepting teleop commands.");
      return;
    }

    target_positions_[joint_index] = std::clamp(
      target_positions_[joint_index] + delta,
      joint_mins_[joint_index],
      joint_maxs_[joint_index]);

    publish_target();
  }

  static float rad_to_bridge_cmd(double rad_value)
  {
    if (rad_value > 1.0) return 1.0f;
    if (rad_value < -1.0) return -1.0f;
    return static_cast<float>(rad_value);
  }

  void publish_target()
  {
    if (publish_controller_commands_ && command_pub_) {
      std_msgs::msg::Float64MultiArray msg;
      msg.data.assign(target_positions_.begin(), target_positions_.end());
      command_pub_->publish(msg);
    }

    if (publish_direct_actuator_cmd_ && actuator_cmd_pub_) {
      std_msgs::msg::Float32MultiArray actuator_msg;
      actuator_msg.data = {
        rad_to_bridge_cmd(target_positions_[1]),
        rad_to_bridge_cmd(target_positions_[2])
      };
      actuator_cmd_pub_->publish(actuator_msg);
    }

    print_status();
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr actuator_cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  std::array<double, 5> measured_positions_{{0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 5> target_positions_{{0.0, 0.0, 0.0, 0.0, 0.0}};

  const std::array<double, 5> joint_mins_{{-3.14, -1.57, -1.57, -1.57, -1.57}};
  const std::array<double, 5> joint_maxs_{{ 3.14,  1.57,  1.57,  1.57,  1.57}};

  double step_size_rad_{5.0 * M_PI / 180.0};
  const double min_step_size_rad_{1.0 * M_PI / 180.0};
  const double max_step_size_rad_{45.0 * M_PI / 180.0};

  bool have_joint_state_{false};
  bool publish_controller_commands_{true};
  bool publish_direct_actuator_cmd_{true};
  bool require_joint_state_before_motion_{false};
};

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyboardTeleop>();
  node->spin();
  rclcpp::shutdown();
  return 0;
}
