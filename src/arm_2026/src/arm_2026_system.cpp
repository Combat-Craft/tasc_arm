#include "arm_2026/arm_2026_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>
#include <utility>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arm_2026
{

hardware_interface::CallbackReturn Arm2026System::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_names_.clear();
  for (const auto & joint : info_.joints)
  {
    joint_names_.push_back(joint.name);
  }

  if (joint_names_.size() != NUM_JOINTS)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("Arm2026System"),
      "Expected %zu joints, got %zu",
      NUM_JOINTS,
      joint_names_.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (joint_names_[BASE_IDX] != "base_yaw" ||
      joint_names_[SHOULDER_IDX] != "shoulder_extension" ||
      joint_names_[ELBOW_IDX] != "elbow_extension")
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("Arm2026System"),
      "Joint order mismatch. Expected [base_yaw, shoulder_extension, elbow_extension].");
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_commands_.assign(NUM_JOINTS, 0.0);
  hw_states_.assign(NUM_JOINTS, 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Arm2026System::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(NUM_JOINTS);

  for (std::size_t i = 0; i < NUM_JOINTS; ++i)
  {
    state_interfaces.emplace_back(
      joint_names_[i],
      hardware_interface::HW_IF_POSITION,
      &hw_states_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Arm2026System::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(NUM_JOINTS);

  for (std::size_t i = 0; i < NUM_JOINTS; ++i)
  {
    command_interfaces.emplace_back(
      joint_names_[i],
      hardware_interface::HW_IF_POSITION,
      &hw_commands_[i]);
  }

  return command_interfaces;
}

double Arm2026System::clampf(double x, double lo, double hi) const
{
  return std::clamp(x, lo, hi);
}

bool Arm2026System::phidget_ok(PhidgetReturnCode code, const char * context) const
{
  if (code == EPHIDGET_OK)
  {
    return true;
  }

  const char * error_description = "Unknown error";
  Phidget_getErrorDescription(code, &error_description);

  RCLCPP_ERROR(
    rclcpp::get_logger("Arm2026System"),
    "Phidget error in %s: %s",
    context,
    error_description);

  return false;
}

hardware_interface::CallbackReturn Arm2026System::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("Arm2026System"), "Configuring Arm2026System");

  // ROS bridge to ESP32 for shoulder/elbow actuators
  comms_node_ = std::make_shared<rclcpp::Node>("arm_2026_hw_bridge");

  actuator_cmd_pub_ =
    comms_node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/arm_actuator_targets",
      10);

  actuator_state_sub_ =
    comms_node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/arm_actuator_states",
      10,
      std::bind(&Arm2026System::actuator_state_callback, this, std::placeholders::_1));

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(comms_node_);

  executor_running_.store(true);
  executor_thread_ = std::thread([this]()
  {
    while (executor_running_.load())
    {
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });

  // Base Phidget setup
  if (!phidget_ok(PhidgetStepper_create(&base_stepper_), "PhidgetStepper_create"))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!phidget_ok(
        Phidget_setDeviceSerialNumber(
          reinterpret_cast<PhidgetHandle>(base_stepper_), base_device_serial_),
        "Phidget_setDeviceSerialNumber"))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!phidget_ok(
        Phidget_setHubPort(
          reinterpret_cast<PhidgetHandle>(base_stepper_), base_hub_port_),
        "Phidget_setHubPort"))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!phidget_ok(
        Phidget_setChannel(
          reinterpret_cast<PhidgetHandle>(base_stepper_), base_channel_),
        "Phidget_setChannel"))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!phidget_ok(
        Phidget_openWaitForAttachment(
          reinterpret_cast<PhidgetHandle>(base_stepper_), 5000),
        "Phidget_openWaitForAttachment"))
  {
    RCLCPP_WARN(
      rclcpp::get_logger("Arm2026System"),
      "Base Phidget not attached. Continuing in fake-base mode.");

    base_stepper_attached_ = false;

    if (base_stepper_ != nullptr)
    {
      PhidgetStepper_delete(&base_stepper_);
      base_stepper_ = nullptr;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  if (!phidget_ok(
        PhidgetStepper_setRescaleFactor(base_stepper_, base_rescale_factor_deg_),
        "PhidgetStepper_setRescaleFactor"))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  base_stepper_attached_ = true;
  RCLCPP_INFO(
    rclcpp::get_logger("Arm2026System"),
    "Base Phidget attached on serial %d, hub port %d, channel %d",
    base_device_serial_,
    base_hub_port_,
    base_channel_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Arm2026System::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("Arm2026System"), "Activating Arm2026System");

  if (base_stepper_ && base_stepper_attached_)
  {
    if (!phidget_ok(
          PhidgetStepper_setAcceleration(base_stepper_, base_acceleration_deg_),
          "PhidgetStepper_setAcceleration"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!phidget_ok(
          PhidgetStepper_setVelocityLimit(base_stepper_, base_velocity_limit_deg_),
          "PhidgetStepper_setVelocityLimit"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!phidget_ok(
          PhidgetStepper_setEngaged(base_stepper_, 1),
          "PhidgetStepper_setEngaged"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    double base_pos_deg = 0.0;
    if (!phidget_ok(
          PhidgetStepper_getPosition(base_stepper_, &base_pos_deg),
          "PhidgetStepper_getPosition"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    hw_states_[BASE_IDX] = base_pos_deg * M_PI / 180.0;
    hw_commands_[BASE_IDX] = hw_states_[BASE_IDX];

    if (!phidget_ok(
          PhidgetStepper_setTargetPosition(base_stepper_, base_pos_deg),
          "PhidgetStepper_setTargetPosition"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  else
  {
    hw_states_[BASE_IDX] = hw_commands_[BASE_IDX];
  }

  hw_states_[SHOULDER_IDX] = 0.0;
  hw_states_[ELBOW_IDX] = 0.0;
  hw_commands_[SHOULDER_IDX] = 0.0;
  hw_commands_[ELBOW_IDX] = 0.0;

  actuator_command_shoulder_ = 0.0;
  actuator_command_elbow_ = 0.0;

  actuator_state_shoulder_ = 0.0;
  actuator_state_elbow_ = 0.0;
  actuator_state_received_.store(false);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Arm2026System::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("Arm2026System"), "Deactivating Arm2026System");

  if (base_stepper_)
  {
    phidget_ok(PhidgetStepper_setEngaged(base_stepper_, 0), "PhidgetStepper_setEngaged(0)");
    phidget_ok(Phidget_close(reinterpret_cast<PhidgetHandle>(base_stepper_)), "Phidget_close");
    phidget_ok(PhidgetStepper_delete(&base_stepper_), "PhidgetStepper_delete");
    base_stepper_ = nullptr;
    base_stepper_attached_ = false;
  }

  executor_running_.store(false);
  if (executor_thread_.joinable())
  {
    executor_thread_.join();
  }

  if (executor_ && comms_node_)
  {
    executor_->remove_node(comms_node_);
  }

  actuator_cmd_pub_.reset();
  actuator_state_sub_.reset();
  executor_.reset();
  comms_node_.reset();

  return hardware_interface::CallbackReturn::SUCCESS;
}

void Arm2026System::actuator_state_callback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 2)
  {
    RCLCPP_WARN(
      rclcpp::get_logger("Arm2026System"),
      "Actuator state message too short");
    return;
  }

  actuator_state_shoulder_ = msg->data[0];
  actuator_state_elbow_ = msg->data[1];
  actuator_state_received_.store(true);
}

hardware_interface::return_type Arm2026System::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  // Base joint
  if (base_stepper_ && base_stepper_attached_)
  {
    double base_pos_deg = 0.0;
    if (phidget_ok(
          PhidgetStepper_getPosition(base_stepper_, &base_pos_deg),
          "PhidgetStepper_getPosition"))
    {
      hw_states_[BASE_IDX] = base_pos_deg * M_PI / 180.0;
    }
  }
  else
  {
    // Fake-base mode when no Phidget is connected
    hw_states_[BASE_IDX] = hw_commands_[BASE_IDX];
  }

  // Shoulder and elbow
  if (actuator_state_received_.load())
  {
    hw_states_[SHOULDER_IDX] = actuator_state_shoulder_;
    hw_states_[ELBOW_IDX] = actuator_state_elbow_;
  }
  else
  {
    hw_states_[SHOULDER_IDX] = hw_commands_[SHOULDER_IDX];
    hw_states_[ELBOW_IDX] = hw_commands_[ELBOW_IDX];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Arm2026System::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  hw_commands_[BASE_IDX] =
    clampf(hw_commands_[BASE_IDX], base_min_, base_max_);

  hw_commands_[SHOULDER_IDX] =
    clampf(hw_commands_[SHOULDER_IDX], shoulder_min_, shoulder_max_);

  hw_commands_[ELBOW_IDX] =
    clampf(hw_commands_[ELBOW_IDX], elbow_min_, elbow_max_);

  // Base command to Phidget when attached
  if (base_stepper_ && base_stepper_attached_)
  {
    const double base_target_deg = hw_commands_[BASE_IDX] * 180.0 / M_PI;
    phidget_ok(
      PhidgetStepper_setTargetPosition(base_stepper_, base_target_deg),
      "PhidgetStepper_setTargetPosition");
  }

  // Shoulder + elbow targets to ESP32
  actuator_command_shoulder_ = hw_commands_[SHOULDER_IDX];
  actuator_command_elbow_ = hw_commands_[ELBOW_IDX];

  if (actuator_cmd_pub_)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {actuator_command_shoulder_, actuator_command_elbow_};
    actuator_cmd_pub_->publish(msg);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace arm_2026

PLUGINLIB_EXPORT_CLASS(
  arm_2026::Arm2026System,
  hardware_interface::SystemInterface)
