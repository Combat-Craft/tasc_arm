#include "phidgets_hardware/phidgets_stepper_hardware.hpp"

#include <cmath>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace phidgets_hardware
{

hardware_interface::CallbackReturn
PhidgetsStepperHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::ActuatorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 1) {
    RCLCPP_ERROR(rclcpp::get_logger("PhidgetsStepperHardware"),
                 "Expected exactly 1 joint, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto get_param = [&](const std::string & key, const std::string & def) -> std::string {
    auto it = info_.hardware_parameters.find(key);
    return (it == info_.hardware_parameters.end()) ? def : it->second;
  };

  device_serial_ = std::stoi(get_param("device_serial", "0"));
  hub_port_      = std::stoi(get_param("hub_port", "0"));
  channel_       = std::stoi(get_param("channel", "0"));

  acceleration_    = std::stod(get_param("acceleration", "1000.0"));
  velocity_limit_  = std::stod(get_param("velocity_limit", "479.0"));

  current_limit_ = std::stod(get_param("current_limit", "1.2"));
  holding_current_limit_ = std::stod(get_param("holding_current_limit", "0.6"));

  rescale_factor_rot_ = std::stod(get_param("rescale_factor_rot", "0.00416666667"));

  command_limit_rad_ = std::stod(get_param("command_limit_rad", "6.283185307179586"));

  cmd_pos_rad_ = 0.0;

  RCLCPP_INFO(
    rclcpp::get_logger("PhidgetsStepperHardware"),
    "Init: serial=%d hub_port=%d channel=%d accel=%.3f vel_limit=%.3f current=%.3f hold_current=%.3f rescale_factor_rot=%.11f cmd_limit_rad=%.3f",
    device_serial_, hub_port_, channel_,
    acceleration_, velocity_limit_,
    current_limit_, holding_current_limit_,
    rescale_factor_rot_,
    command_limit_rad_
  );

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
PhidgetsStepperHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &pos_state_rad_);
  state_interfaces.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &vel_state_);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
PhidgetsStepperHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &cmd_pos_rad_);
  return command_interfaces;
}

hardware_interface::CallbackReturn
PhidgetsStepperHardware::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("PhidgetsStepperHardware"), "Activating...");

  close_phidget();
  attached_ = false;

  PhidgetReturnCode rc = PhidgetStepper_create(&stepper_);
  if (rc != EPHIDGET_OK || stepper_ == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("PhidgetsStepperHardware"),
                 "PhidgetStepper_create failed: %d", rc);
    return hardware_interface::CallbackReturn::ERROR;
  }

  rc = Phidget_setDeviceSerialNumber((PhidgetHandle)stepper_, device_serial_);
  if (rc != EPHIDGET_OK) {
    RCLCPP_ERROR(rclcpp::get_logger("PhidgetsStepperHardware"),
                 "Phidget_setDeviceSerialNumber failed: %d", rc);
    close_phidget();
    return hardware_interface::CallbackReturn::ERROR;
  }

  rc = Phidget_setHubPort((PhidgetHandle)stepper_, hub_port_);
  if (rc != EPHIDGET_OK) {
    RCLCPP_ERROR(rclcpp::get_logger("PhidgetsStepperHardware"),
                 "Phidget_setHubPort failed: %d", rc);
    close_phidget();
    return hardware_interface::CallbackReturn::ERROR;
  }

  rc = Phidget_setChannel((PhidgetHandle)stepper_, channel_);
  if (rc != EPHIDGET_OK) {
    RCLCPP_ERROR(rclcpp::get_logger("PhidgetsStepperHardware"),
                 "Phidget_setChannel failed: %d", rc);
    close_phidget();
    return hardware_interface::CallbackReturn::ERROR;
  }

  rc = Phidget_openWaitForAttachment((PhidgetHandle)stepper_, 5000);
  if (rc != EPHIDGET_OK) {
    RCLCPP_ERROR(rclcpp::get_logger("PhidgetsStepperHardware"),
                 "Phidget_openWaitForAttachment failed: %d", rc);
    close_phidget();
    return hardware_interface::CallbackReturn::ERROR;
  }

  attached_ = true;

  // Match last year's working behavior
  rc = PhidgetStepper_setRescaleFactor(stepper_, rescale_factor_rot_);
  if (rc != EPHIDGET_OK) {
    RCLCPP_WARN(rclcpp::get_logger("PhidgetsStepperHardware"),
                "PhidgetStepper_setRescaleFactor(%.11f) failed: %d", rescale_factor_rot_, rc);
  }

  rc = PhidgetStepper_setAcceleration(stepper_, acceleration_);
  if (rc != EPHIDGET_OK) {
    const char * err = nullptr;
    Phidget_getErrorDescription(rc, &err);
    RCLCPP_WARN(
      rclcpp::get_logger("PhidgetsStepperHardware"),
      "Failed to set acceleration %.3f (rc=%d, %s)",
      acceleration_, rc, err ? err : "unknown"
    );
  }

  rc = PhidgetStepper_setVelocityLimit(stepper_, velocity_limit_);
  if (rc != EPHIDGET_OK) {
    const char * err = nullptr;
    Phidget_getErrorDescription(rc, &err);
    RCLCPP_WARN(
      rclcpp::get_logger("PhidgetsStepperHardware"),
      "Failed to set velocity_limit %.3f (rc=%d, %s)",
      velocity_limit_, rc, err ? err : "unknown"
    );
  }

  rc = PhidgetStepper_setCurrentLimit(stepper_, current_limit_);
  if (rc != EPHIDGET_OK) {
    RCLCPP_WARN(rclcpp::get_logger("PhidgetsStepperHardware"),
                "PhidgetStepper_setCurrentLimit(%.3f) failed: %d", current_limit_, rc);
  }

  rc = PhidgetStepper_setHoldingCurrentLimit(stepper_, holding_current_limit_);
  if (rc != EPHIDGET_OK) {
    RCLCPP_WARN(rclcpp::get_logger("PhidgetsStepperHardware"),
                "PhidgetStepper_setHoldingCurrentLimit(%.3f) failed: %d", holding_current_limit_, rc);
  }

  // Read current position (units == rotations after rescale), then start from there to avoid jumps
  double pos_units = 0.0;
  rc = PhidgetStepper_getPosition(stepper_, &pos_units);
  if (rc == EPHIDGET_OK) {
    pos_state_rad_ = units_to_rad(pos_units);
    cmd_pos_rad_ = pos_state_rad_;
    (void)PhidgetStepper_setTargetPosition(stepper_, pos_units);
  } else {
    cmd_pos_rad_ = 0.0;
    (void)PhidgetStepper_setTargetPosition(stepper_, 0.0);
  }

  // Engage last (so it holds the target)
  rc = PhidgetStepper_setEngaged(stepper_, 1);
  if (rc != EPHIDGET_OK) {
    RCLCPP_ERROR(rclcpp::get_logger("PhidgetsStepperHardware"),
                 "PhidgetStepper_setEngaged(true) failed: %d", rc);
    close_phidget();
    attached_ = false;
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("PhidgetsStepperHardware"), "Activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
PhidgetsStepperHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("PhidgetsStepperHardware"), "Deactivating...");

  if (stepper_) {
    (void)PhidgetStepper_setEngaged(stepper_, 0);
  }

  close_phidget();
  attached_ = false;

  RCLCPP_INFO(rclcpp::get_logger("PhidgetsStepperHardware"), "Deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
PhidgetsStepperHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!attached_ || !stepper_) {
    return hardware_interface::return_type::OK;
  }

  double pos_units = 0.0;
  double vel = 0.0;

  const auto rc_p = PhidgetStepper_getPosition(stepper_, &pos_units);
  if (rc_p == EPHIDGET_OK) {
    pos_state_rad_ = units_to_rad(pos_units);
  }

  const auto rc_v = PhidgetStepper_getVelocity(stepper_, &vel);
  if (rc_v == EPHIDGET_OK) {
    vel_state_ = vel;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
PhidgetsStepperHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!attached_ || !stepper_) {
    return hardware_interface::return_type::OK;
  }

  const double cmd_rad = clamp(cmd_pos_rad_, -command_limit_rad_, command_limit_rad_);
  const double target_units = rad_to_units(cmd_rad);  // units == rotations

  const auto rc = PhidgetStepper_setTargetPosition(stepper_, target_units);
  if (rc != EPHIDGET_OK) {
    RCLCPP_WARN(rclcpp::get_logger("PhidgetsStepperHardware"),
                "PhidgetStepper_setTargetPosition(%.6f units) failed: %d", target_units, rc);
  }

  return hardware_interface::return_type::OK;
}

void PhidgetsStepperHardware::close_phidget()
{
  if (stepper_) {
    (void)Phidget_close((PhidgetHandle)stepper_);
    (void)PhidgetStepper_delete(&stepper_);
    stepper_ = nullptr;
  }
}

}  // namespace phidgets_hardware

PLUGINLIB_EXPORT_CLASS(phidgets_hardware::PhidgetsStepperHardware, hardware_interface::ActuatorInterface)

