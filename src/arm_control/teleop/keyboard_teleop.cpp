#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace
{

class TerminalGuard
{
public:
  TerminalGuard()
  {
    if (!isatty(STDIN_FILENO))
    {
      return;
    }

    if (tcgetattr(STDIN_FILENO, &original_) != 0)
    {
      return;
    }

    termios raw = original_;
    raw.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;

    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0)
    {
      active_ = true;
    }
  }

  ~TerminalGuard()
  {
    if (active_)
    {
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
  : Node("keyboard_teleop")
  {
    publish_controller_commands_ =
      declare_parameter<bool>(
        "publish_controller_commands",
        true);

    require_joint_state_before_motion_ =
      declare_parameter<bool>(
        "require_joint_state_before_motion",
        false);

    command_speed_rad_s_ =
      declare_parameter<double>(
        "command_speed_rad_s",
        0.25);

    hold_timeout_ms_ =
      declare_parameter<int>(
        "hold_timeout_ms",
        180);

    /*
     * Relative topic names are used so launching this node under the
     * "arm" namespace produces:
     *
     *   /arm/manual_controller/commands
     *   /arm/debug_mode
     *   /arm/joint_states
     */
    command_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(
        "/arm/manual_controller/commands",
        10);

    debug_pub_ =
      create_publisher<std_msgs::msg::Bool>(
        "/arm/debug_mode",
        10);

    joint_state_sub_ =
      create_subscription<sensor_msgs::msg::JointState>(
        "/arm/joint_states",
        10,
        std::bind(
          &KeyboardTeleop::joint_state_callback,
          this,
          std::placeholders::_1));

    print_help();
  }

  void spin()
  {
    if (!isatty(STDIN_FILENO))
    {
      RCLCPP_ERROR(
        get_logger(),
        "Keyboard teleop requires a real terminal (TTY).");

      return;
    }

    TerminalGuard terminal_guard;
    rclcpp::WallRate loop_rate(50.0);

    while (rclcpp::ok())
    {
      rclcpp::spin_some(shared_from_this());

      char key = 0;

      if (read_key(key))
      {
        if (!handle_key(key))
        {
          break;
        }
      }

      update_hold_commands();
      loop_rate.sleep();
    }

    stop_all();
  }

private:
  void print_help() const
  {
    std::printf(
      "\n"
      "Keyboard teleop for arm\n"
      "Commands:\n"
      "  q/a : base + / -\n"
      "  w/s : shoulder + / -\n"
      "  e/d : elbow + / -\n"
      "  r/f : wrist twist + / -\n"
      "  t/g : wrist roll + / -\n"
      "  y/h : claw + / -\n"
      "  z/x : decrease / increase speed\n"
      "  p   : toggle debug mode\n"
      "  space or c : stop all joints\n"
      "  Ctrl-C : quit\n"
      "\n");

    std::printf(
      "Publishing to /arm/manual_controller/commands: %s\n",
      publish_controller_commands_ ? "yes" : "no");

    std::printf(
      "Require /arm/joint_states: %s\n",
      require_joint_state_before_motion_ ? "yes" : "no");

    print_status();
  }

  void print_status() const
  {
    std::printf(
      "Velocity command [rad/s] "
      "base=%.3f shoulder=%.3f elbow=%.3f "
      "wrist_roll=%.3f wrist_twist=%.3f claw=%.3f "
      "| selected speed=%.3f rad/s\n",
      velocity_commands_[0],
      velocity_commands_[1],
      velocity_commands_[2],
      velocity_commands_[3],
      velocity_commands_[4],
      velocity_commands_[5],
      command_speed_rad_s_);

    std::fflush(stdout);
  }

  void print_debug_banner() const
  {
    std::printf("\n");
    std::printf("########################################\n");
    std::printf(
      "#          DEBUG MODE %s             #\n",
      debug_mode_enabled_ ? "ON " : "OFF");
    std::printf("########################################\n");
    std::printf("\n");

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

    const int ready =
      select(
        STDIN_FILENO + 1,
        &read_set,
        nullptr,
        nullptr,
        &timeout);

    if (ready < 0)
    {
      if (errno != EINTR)
      {
        RCLCPP_WARN(
          get_logger(),
          "select() failed while reading keyboard input: %s",
          std::strerror(errno));
      }

      return false;
    }

    if (ready == 0)
    {
      return false;
    }

    const ssize_t bytes_read =
      ::read(
        STDIN_FILENO,
        &key,
        1);

    return bytes_read == 1;
  }

  void joint_state_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (
      !have_joint_state_ &&
      !msg->name.empty() &&
      !msg->position.empty())
    {
      have_joint_state_ = true;

      RCLCPP_INFO(
        get_logger(),
        "Received initial /arm/joint_states message.");
    }
  }

  bool handle_key(char key)
  {
    switch (key)
    {
      case 'q':
        set_hold_joint(
          BASE_IDX,
          command_speed_rad_s_);
        return true;

      case 'a':
        set_hold_joint(
          BASE_IDX,
          -command_speed_rad_s_);
        return true;

      case 'w':
        set_hold_joint(
          SHOULDER_IDX,
          command_speed_rad_s_);
        return true;

      case 's':
        set_hold_joint(
          SHOULDER_IDX,
          -command_speed_rad_s_);
        return true;

      case 'e':
        set_hold_joint(
          ELBOW_IDX,
          command_speed_rad_s_);
        return true;

      case 'd':
        set_hold_joint(
          ELBOW_IDX,
          -command_speed_rad_s_);
        return true;

      case 'r':
        set_hold_joint(
          WRIST_TWIST_IDX,
          command_speed_rad_s_);
        return true;

      case 'f':
        set_hold_joint(
          WRIST_TWIST_IDX,
          -command_speed_rad_s_);
        return true;

      case 't':
        set_hold_joint(
          WRIST_ROLL_IDX,
          command_speed_rad_s_);
        return true;

      case 'g':
        set_hold_joint(
          WRIST_ROLL_IDX,
          -command_speed_rad_s_);
        return true;

      case 'y':
        set_hold_joint(
          CLAW_IDX,
          command_speed_rad_s_);
        return true;

      case 'h':
        set_hold_joint(
          CLAW_IDX,
          -command_speed_rad_s_);
        return true;

      case 'z':
        command_speed_rad_s_ =
          std::max(
            min_command_speed_rad_s_,
            command_speed_rad_s_ / 2.0);

        print_status();
        return true;

      case 'x':
        command_speed_rad_s_ =
          std::min(
            max_command_speed_rad_s_,
            command_speed_rad_s_ * 2.0);

        print_status();
        return true;

      case 'p':
        toggle_debug_mode();
        return true;

      case ' ':
      case 'c':
        stop_all();
        return true;

      default:
        return true;
    }
  }

  void set_hold_joint(
    std::size_t joint_index,
    double velocity)
  {
    if (
      !have_joint_state_ &&
      require_joint_state_before_motion_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Waiting for /arm/joint_states before accepting commands.");

      return;
    }

    velocity_commands_[joint_index] =
      std::clamp(
        velocity,
        -max_command_speed_rad_s_,
        max_command_speed_rad_s_);

    hold_deadlines_[joint_index] =
      std::chrono::steady_clock::now() +
      std::chrono::milliseconds(
        hold_timeout_ms_);

    publish_commands();
  }

  void update_hold_commands()
  {
    const auto now =
      std::chrono::steady_clock::now();

    bool command_changed = false;

    for (
      std::size_t i = 0;
      i < velocity_commands_.size();
      ++i)
    {
      if (
        std::fabs(velocity_commands_[i]) > 1e-9 &&
        now > hold_deadlines_[i])
      {
        velocity_commands_[i] = 0.0;
        command_changed = true;
      }
    }

    if (command_changed)
    {
      publish_commands();
    }
  }

  void stop_all()
  {
    velocity_commands_.fill(0.0);
    publish_commands();
  }

  void toggle_debug_mode()
  {
    debug_mode_enabled_ =
      !debug_mode_enabled_;

    std_msgs::msg::Bool msg;
    msg.data = debug_mode_enabled_;

    debug_pub_->publish(msg);

    print_debug_banner();
  }

  void publish_commands()
  {
    if (
      publish_controller_commands_ &&
      command_pub_)
    {
      std_msgs::msg::Float64MultiArray msg;

      msg.data.assign(
        velocity_commands_.begin(),
        velocity_commands_.end());

      command_pub_->publish(msg);
    }

    print_status();
  }

  static constexpr std::size_t NUM_JOINTS = 6;

  static constexpr std::size_t BASE_IDX = 0;
  static constexpr std::size_t SHOULDER_IDX = 1;
  static constexpr std::size_t ELBOW_IDX = 2;
  static constexpr std::size_t WRIST_ROLL_IDX = 3;
  static constexpr std::size_t WRIST_TWIST_IDX = 4;
  static constexpr std::size_t CLAW_IDX = 5;

  rclcpp::Publisher<
    std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;

  rclcpp::Publisher<
    std_msgs::msg::Bool>::SharedPtr debug_pub_;

  rclcpp::Subscription<
    sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  std::array<double, NUM_JOINTS> velocity_commands_{
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
  };

  std::array<
    std::chrono::steady_clock::time_point,
    NUM_JOINTS> hold_deadlines_{};

  double command_speed_rad_s_{0.25};

  const double min_command_speed_rad_s_{0.05};
  const double max_command_speed_rad_s_{1.0};

  int hold_timeout_ms_{180};

  bool have_joint_state_{false};
  bool publish_controller_commands_{true};
  bool require_joint_state_before_motion_{false};
  bool debug_mode_enabled_{false};
};

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<KeyboardTeleop>();

  node->spin();

  rclcpp::shutdown();

  return 0;
}
