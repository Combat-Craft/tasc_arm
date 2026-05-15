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
#include "std_msgs/msg/float32_multi_array.hpp"

namespace arm_2026
{

class Arm2026System : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Arm2026System)

  ~Arm2026System() override;

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
  void actuator_state_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void cleanup_phidgets();
  void cleanup_ros_bridge();

  bool phidget_ok(PhidgetReturnCode code, const char * context) const;
  double clampf(double x, double lo, double hi) const;

  static constexpr std::size_t NUM_JOINTS = 5;
  static constexpr std::size_t BASE_IDX = 0;
  static constexpr std::size_t SHOULDER_IDX = 1;
  static constexpr std::size_t ELBOW_IDX = 2;
  static constexpr std::size_t WRIST_ROLL_IDX = 3;
  static constexpr std::size_t WRIST_TWIST_IDX = 4;

  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
  std::vector<std::string> joint_names_;

  // Joint limits
  double base_min_ = -3.14;
  double base_max_ =  3.14;

  double shoulder_min_ = -1.57;
  double shoulder_max_ =  1.57;

  double elbow_min_ = -1.57;
  double elbow_max_ =  1.57;

  double wrist_roll_min_ = -1.57;
  double wrist_roll_max_ =  1.57;

  double wrist_twist_min_ = -1.57;
  double wrist_twist_max_ =  1.57;

  // ROS2 bridge for ESP32 linear actuators
  rclcpp::Node::SharedPtr comms_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
  std::atomic<bool> executor_running_{false};

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr actuator_cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr actuator_state_sub_;

  // Current ESP32 command/state bridge
  // data[0] = shoulder target/state
  // data[1] = elbow target/state
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

  // Wrist differential Phidget steppers
  PhidgetStepperHandle wrist_motor_1_ = nullptr;
  PhidgetStepperHandle wrist_motor_2_ = nullptr;
  bool wrist_motor_1_attached_ = false;
  bool wrist_motor_2_attached_ = false;

  int wrist_device_serial_ = 766944;
  int wrist_motor_1_hub_port_ = 1;
  int wrist_motor_2_hub_port_ = 2;
  int wrist_channel_ = 0;

  // Rescale factors in deg/count
  double base_rescale_factor_deg_ = 0.00146103896;

  // Placeholder wrist rescale factors, tune later if needed
  double wrist_motor_1_rescale_factor_deg_ = 0.00146103896;
  double wrist_motor_2_rescale_factor_deg_ = 0.00146103896;

  // Motion tuning in Phidget rescaled units
  double base_velocity_limit_deg_ = 30.0;
  double base_acceleration_deg_ = 40.0;

  double wrist_velocity_limit_deg_ = 30.0;
  double wrist_acceleration_deg_ = 40.0;
};

}  // namespace arm_2026

#endif  // ARM_2026__ARM_2026_SYSTEM_HPP_
