#pragma once

#include <string>
#include <vector>
#include <cstdint>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

// Phidgets C API (Phidget22)
#include <phidget22.h>

namespace phidgets_hardware
{

class PhidgetsStepperHardware : public hardware_interface::ActuatorInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PhidgetsStepperHardware)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters (read from URDF ros2_control params in on_init)
  int32_t device_serial_{766944};  // VINT hub serial
  int hub_port_{0};                // VINT hub port
  int channel_{0};                 // channel number

  // Motion config (Stepper API)
  double acceleration_{1000.0};    // PhidgetStepper_setAcceleration()
  double velocity_limit_{479.0};   // PhidgetStepper_setVelocityLimit()

  // Current limits
  double current_limit_{1.2};         // amps
  double holding_current_limit_{0.6}; // amps

  // Phidget rescale factor (set on device in on_activate)
  // Keep this as a parameter so it matches your old working script
  double rescale_factor_rot_{0.00416666667};

  // Optional command clamp (radians). Default: 2*pi
  double command_limit_rad_{6.283185307179586};

  // Phidgets handle
  PhidgetStepperHandle stepper_{nullptr};
  bool attached_{false};

  // ROS2 control values
  double pos_state_rad_{0.0};
  double vel_state_{0.0};
  double cmd_pos_rad_{0.0};

  static double clamp(double x, double lo, double hi)
  {
    return (x < lo) ? lo : ((x > hi) ? hi : x);
  }

  // Helpers
  // IMPORTANT: because we call PhidgetStepper_setRescaleFactor(stepper_, rescale_factor_rot_),
  // the Stepper API position units become OUTPUT rotations.
  double rad_to_units(double rad) const
  {
    return rad * (180.0 / 3.141592653589793);  // degrees
  }

  double units_to_rad(double units) const
  {
    return units * (3.141592653589793 / 180.0);  // radians
  }

  void close_phidget();
};

}  // namespace phidgets_hardware

