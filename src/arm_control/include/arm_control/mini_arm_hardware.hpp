#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>

#include <atomic>
#include <string>
#include <vector>

namespace arm_control
{

class MG996RServoSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
  rclcpp::executors::SingleThreadedExecutor exec_;

  std::string command_topic_{"/arm_servo_deg"};

  // ros2_control boundary is radians
  std::vector<double> hw_states_pos_;
  std::vector<double> hw_commands_pos_;

  // Per-joint direction (+1 or -1). Loaded from "joint_signs" param.
  std::vector<double> joint_sign_;

  // Global servo mapping (still global for now, we can make per-joint later)
  double rad_to_deg_scale_{180.0 / 3.14159265358979323846};
  double center_deg_{90.0};
  double deg_min_{0.0};
  double deg_max_{180.0};

  std::atomic<bool> active_{false};

  static float clamp_f(float v, float lo, float hi)
  {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
  }
};

}  // namespace arm_control

