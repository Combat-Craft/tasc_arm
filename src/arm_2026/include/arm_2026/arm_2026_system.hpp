#ifndef ARM_2026__ARM_2026_SYSTEM_HPP_
#define ARM_2026__ARM_2026_SYSTEM_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <phidget22.h>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace arm_2026
{

class Arm2026System : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Arm2026System)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  void actuator_state_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  bool phidget_ok(PhidgetReturnCode code, const char * context) const;
  double clampf(double x, double lo, double hi) const;

  static constexpr std::size_t NUM_JOINTS = 3;
  static constexpr std::size_t BASE_IDX = 0;
  static constexpr std::size_t SHOULDER_IDX = 1;
  static constexpr std::size_t ELBOW_IDX = 2;

  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
  std::vector<std::string> joint_names_;

  // Joint limits from URDF
  double base_min_ = -3.14;
  double base_max_ =  3.14;

  double shoulder_min_ = -1.57;
  double shoulder_max_ =  1.57;

  double elbow_min_ = -1.57;
  double elbow_max_ =  1.57;

  // ROS2 bridge for ESP32 linear actuators
  rclcpp::Node::SharedPtr comms_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
  std::atomic<bool> executor_running_{false};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr actuator_cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr actuator_state_sub_;

  // shoulder, elbow
  double actuator_command_shoulder_ = 0.0;
  double actuator_command_elbow_ = 0.0;

  double actuator_state_shoulder_ = 0.0;
  double actuator_state_elbow_ = 0.0;

  std::atomic<bool> actuator_state_received_{false};

  // Base Phidget stepper
  PhidgetStepperHandle base_stepper_ = nullptr;
  bool base_stepper_attached_ = false;

  int base_device_serial_ = 766944;
  int base_hub_port_ = 0;
  int base_channel_ = 0;

  // You measured this experimentally
  double base_rescale_factor_deg_ = 0.00146103896; // Calculation: 360.0 / (200.0 * 16.0 * 77.0)

  // URDF velocity is 1.0 rad/s for base
  double base_velocity_limit_deg_ = 30;
  double base_acceleration_deg_ = 40;
};

}  // namespace arm_2026

#endif  // ARM_2026__ARM_2026_SYSTEM_HPP_
