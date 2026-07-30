#ifndef ARM_2026__ARM_2026_SYSTEM_HPP_
#define ARM_2026__ARM_2026_SYSTEM_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <phidget22.h>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/bool.hpp"
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

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

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
  void actuator_state_callback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg);

  void debug_mode_callback(
    const std_msgs::msg::Bool::SharedPtr msg);

  void cleanup_phidgets();
  void cleanup_ros_bridge();

  bool phidget_ok(
    PhidgetReturnCode code,
    const char * context) const;

  double clampf(
    double value,
    double minimum,
    double maximum) const;

  bool open_actuator_serial();
  void close_actuator_serial();

  bool send_actuator_packet(
    uint8_t shoulder_cmd,
    uint8_t elbow_cmd);

  uint8_t command_to_byte(double value) const;

  /*
   * Base encoder and PID helpers.
   */
  bool base_encoder_read_valid_ = false;
  bool read_base_encoder_counts(int64_t & counts);

  double base_encoder_counts_to_output_rad(
    int64_t relative_counts) const;

  void reset_base_pid();

  double calculate_base_pid_velocity(
    double target_position_rad,
    double measured_position_rad,
    double measured_velocity_rad_s,
    double dt);

  static constexpr std::size_t NUM_JOINTS = 6;

  static constexpr std::size_t BASE_IDX = 0;
  static constexpr std::size_t SHOULDER_IDX = 1;
  static constexpr std::size_t ELBOW_IDX = 2;
  static constexpr std::size_t WRIST_ROLL_IDX = 3;
  static constexpr std::size_t WRIST_TWIST_IDX = 4;
  static constexpr std::size_t CLAW_IDX = 5;

  static constexpr uint8_t ACT_STOP = 0;
  static constexpr uint8_t ACT_EXTEND = 1;
  static constexpr uint8_t ACT_RETRACT = 2;

  /*
   * ROS command interfaces contain velocity commands.
   * ROS state interfaces contain measured or estimated positions.
   */
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
  std::vector<std::string> joint_names_;

  /*
   * Joint position limits, in radians.
   */
  double base_min_ = -3.14;
  double base_max_ = 3.14;

  double shoulder_min_ = -1.57;
  double shoulder_max_ = 1.57;

  double elbow_min_ = -1.57;
  double elbow_max_ = 1.57;

  double wrist_roll_min_ = -1.57;
  double wrist_roll_max_ = 1.57;

  double wrist_twist_min_ = -1.57;
  double wrist_twist_max_ = 1.57;

  double claw_min_ = -1.57;
  double claw_max_ = 1.57;

  /*
   * Maximum accepted manual velocity commands.
   */
  double base_command_limit_rad_s_ = 1.0;
  double shoulder_command_limit_rad_s_ = 1.0;
  double elbow_command_limit_rad_s_ = 1.0;
  double wrist_roll_command_limit_rad_s_ = 1.0;
  double wrist_twist_command_limit_rad_s_ = 1.0;
  double claw_command_limit_rad_s_ = 1.0;

  /*
   * Internal target positions for the Phidget steppers.
   */
  double base_target_position_rad_ = 0.0;
  double wrist_roll_target_position_rad_ = 0.0;
  double wrist_twist_target_position_rad_ = 0.0;
  double claw_target_position_rad_ = 0.0;

  /*
   * Shoulder and elbow linear actuator handling.
   */
  double actuator_command_deadband_ = 0.05;

  double actuator_estimated_shoulder_pos_ = 0.0;
  double actuator_estimated_elbow_pos_ = 0.0;

  double actuator_estimated_shoulder_speed_rad_s_ = 0.45;
  double actuator_estimated_elbow_speed_rad_s_ = 0.45;

  rclcpp::Node::SharedPtr comms_node_;

  std::shared_ptr<
    rclcpp::executors::SingleThreadedExecutor> executor_;

  std::thread executor_thread_;
  std::atomic<bool> executor_running_{false};

  rclcpp::Subscription<
    std_msgs::msg::Float32MultiArray>::SharedPtr actuator_state_sub_;

  rclcpp::Subscription<
    std_msgs::msg::Bool>::SharedPtr debug_mode_sub_;

  double actuator_command_shoulder_ = 0.0;
  double actuator_command_elbow_ = 0.0;

  double actuator_state_shoulder_ = 0.0;
  double actuator_state_elbow_ = 0.0;

  std::atomic<bool> actuator_state_received_{false};
  std::atomic<bool> debug_mode_enabled_{false};

  /*
   * ESP32 serial connection for the linear actuators.
   */
  int actuator_serial_fd_ = -1;

  std::string actuator_serial_device_ =
    "/dev/ttyUSB0";

  int actuator_serial_baud_ = 115200;

  /*
   * Base stepper.
   */
  PhidgetStepperHandle base_stepper_ = nullptr;
  bool base_stepper_attached_ = false;

  int base_device_serial_ = 766944;
  int base_hub_port_ = 0;
  int base_channel_ = 0;

  /*
   * Base quadrature encoder.
   *
   * ENC1001 is connected to VINT hub port 5.
   * The encoder is mounted on the motor shaft before the gearbox.
   */
  PhidgetEncoderHandle base_encoder_ = nullptr;
  bool base_encoder_attached_ = false;

  int base_encoder_device_serial_ = 766944;
  int base_encoder_hub_port_ = 5;
  int base_encoder_channel_ = 0;

  /*
   * Encoder configuration.
   *
   * A 300 CPR quadrature encoder commonly produces 1200 decoded
   * position counts per motor-shaft revolution.
   *
   * Verify this by rotating the encoder shaft exactly one revolution
   * and checking the raw position-count change.
   */
  double base_encoder_counts_per_motor_rev_ = 1200.0;

  /*
   * Actual gearbox ratio for the nominal 77:1 Phidget gearbox.
   * Change this if your motor datasheet specifies a different ratio.
   */
  double base_gear_ratio_ = 76.765625;

  /*
   * Use -1.0 if positive motor motion produces negative encoder counts.
   * Use +1.0 if their positive directions already match.
   */
  double base_encoder_direction_ = -1.0;

  int64_t base_encoder_zero_counts_ = 0;
  int64_t base_encoder_last_counts_ = 0;

  double base_encoder_position_rad_ = 0.0;
  double base_encoder_previous_position_rad_ = 0.0;
  double base_encoder_velocity_rad_s_ = 0.0;

  /*
   * Base position PID.
   *
   * Input:
   *   target base angle minus encoder-measured base angle
   *
   * Output:
   *   requested base output velocity in rad/s
   *
   * Start conservatively. Tune P first, then add D if needed.
   */
  bool base_pid_enabled_ = true;

  double base_pid_kp_ = 0.50;
  double base_pid_ki_ = 0.0;
  double base_pid_kd_ = 0.0;

  double base_pid_integral_ = 0.0;
  double base_pid_previous_error_ = 0.0;

  double base_pid_integral_limit_ = 0.25;

  /*
   * Stop correcting inside this position error to avoid reversing
   * continuously inside mechanical backlash.
   *
   * 0.01 rad is approximately 0.57 degrees.
   */
  double base_pid_position_tolerance_rad_ = 0.01;

  /*
   * Maximum PID output speed at the gearbox output.
   */
  double base_pid_max_velocity_rad_s_ = 0.50;

  /*
   * Wrist differential steppers.
   */
  PhidgetStepperHandle wrist_motor_1_ = nullptr;
  PhidgetStepperHandle wrist_motor_2_ = nullptr;

  bool wrist_motor_1_attached_ = false;
  bool wrist_motor_2_attached_ = false;

  int wrist_device_serial_ = 766944;
  int wrist_motor_1_hub_port_ = 1;
  int wrist_motor_2_hub_port_ = 2;
  int wrist_channel_ = 0;

  /*
   * Claw stepper.
   */
  PhidgetStepperHandle claw_stepper_ = nullptr;
  bool claw_stepper_attached_ = false;

  int claw_device_serial_ = 766944;
  int claw_hub_port_ = 3;
  int claw_channel_ = 0;

  /*
   * Phidget position scaling.
   */
  double base_rescale_factor_deg_ = 0.00146103896;

  double wrist_motor_1_rescale_factor_deg_ = 0.001125;
  double wrist_motor_2_rescale_factor_deg_ = 0.001125;

  double claw_rescale_factor_deg_ = 0.001125;

  /*
   * Phidget stepper motor electrical limits.
   */
  double base_current_limit_a_ = 0.67;
  double wrist_motor_1_current_limit_a_ = 0.67;
  double wrist_motor_2_current_limit_a_ = 0.67;
  double claw_current_limit_a_ = 0.67;

  /*
   * Stepper velocity and acceleration limits.
   */
  double base_velocity_limit_deg_ = 75.0;
  double base_acceleration_deg_ = 150.0;

  double wrist_velocity_limit_deg_ = 75.0;
  double wrist_acceleration_deg_ = 150.0;

  double claw_velocity_limit_deg_ = 75.0;
  double claw_acceleration_deg_ = 150.0;
};

}  // namespace arm_2026

#endif  // ARM_2026__ARM_2026_SYSTEM_HPP_
