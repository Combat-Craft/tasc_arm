#include "arm_2026/arm_2026_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>
#include <utility>

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arm_2026
{

Arm2026System::~Arm2026System()
{
  cleanup_phidgets();
  cleanup_ros_bridge();
  close_actuator_serial();
}

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
      joint_names_[ELBOW_IDX] != "elbow_extension" ||
      joint_names_[WRIST_ROLL_IDX] != "wrist_roll" ||
      joint_names_[WRIST_TWIST_IDX] != "wrist_twist" ||
      joint_names_[CLAW_IDX] != "claw")
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("Arm2026System"),
      "Joint order mismatch. Expected [base_yaw, shoulder_extension, elbow_extension, wrist_roll, wrist_twist, claw].");
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

void Arm2026System::actuator_state_callback(
  const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 2)
  {
    RCLCPP_WARN(
      rclcpp::get_logger("Arm2026System"),
      "Actuator state message too short");
    return;
  }

  actuator_state_shoulder_ = static_cast<double>(msg->data[0]);
  actuator_state_elbow_ = static_cast<double>(msg->data[1]);
  actuator_state_received_.store(true);
}

void Arm2026System::debug_mode_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  debug_mode_enabled_.store(msg->data);
  RCLCPP_WARN(
    rclcpp::get_logger("Arm2026System"),
    "DEBUG MODE %s",
    msg->data ? "ON" : "OFF");
}

bool Arm2026System::open_actuator_serial()
{
  close_actuator_serial();

  actuator_serial_fd_ = ::open(
    actuator_serial_device_.c_str(),
    O_RDWR | O_NOCTTY | O_SYNC);

  if (actuator_serial_fd_ < 0)
  {
    RCLCPP_WARN(
      rclcpp::get_logger("Arm2026System"),
      "Failed to open actuator serial device %s: %s",
      actuator_serial_device_.c_str(),
      std::strerror(errno));
    return false;
  }

  termios tty{};
  if (tcgetattr(actuator_serial_fd_, &tty) != 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("Arm2026System"),
      "tcgetattr failed on %s: %s",
      actuator_serial_device_.c_str(),
      std::strerror(errno));
    close_actuator_serial();
    return false;
  }

  cfmakeraw(&tty);

  speed_t baud = B115200;
  if (actuator_serial_baud_ != 115200)
  {
    RCLCPP_WARN(
      rclcpp::get_logger("Arm2026System"),
      "Unsupported baud %d requested, defaulting to 115200",
      actuator_serial_baud_);
  }

  cfsetispeed(&tty, baud);
  cfsetospeed(&tty, baud);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(actuator_serial_fd_, TCSANOW, &tty) != 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("Arm2026System"),
      "tcsetattr failed on %s: %s",
      actuator_serial_device_.c_str(),
      std::strerror(errno));
    close_actuator_serial();
    return false;
  }

  tcflush(actuator_serial_fd_, TCIOFLUSH);

  RCLCPP_INFO(
    rclcpp::get_logger("Arm2026System"),
    "Opened actuator serial device %s at %d baud",
    actuator_serial_device_.c_str(),
    actuator_serial_baud_);

  return true;
}

void Arm2026System::close_actuator_serial()
{
  if (actuator_serial_fd_ >= 0)
  {
    ::close(actuator_serial_fd_);
    actuator_serial_fd_ = -1;
  }
}

uint8_t Arm2026System::command_to_byte(double value) const
{
  if (value > actuator_command_deadband_)
  {
    return ACT_EXTEND;
  }

  if (value < -actuator_command_deadband_)
  {
    return ACT_RETRACT;
  }

  return ACT_STOP;
}

bool Arm2026System::send_actuator_packet(uint8_t shoulder_cmd, uint8_t elbow_cmd)
{
  if (actuator_serial_fd_ < 0)
  {
    return false;
  }

  const uint8_t header = 0xAA;
  const uint8_t checksum = shoulder_cmd ^ elbow_cmd;
  const uint8_t packet[4] = {header, shoulder_cmd, elbow_cmd, checksum};

  const ssize_t written = ::write(actuator_serial_fd_, packet, sizeof(packet));

  if (written != static_cast<ssize_t>(sizeof(packet)))
  {
    RCLCPP_WARN(
      rclcpp::get_logger("Arm2026System"),
      "Failed to write full actuator packet to serial device: wrote %zd bytes",
      written);
    return false;
  }

  return true;
}

void Arm2026System::cleanup_phidgets()
{
  if (base_stepper_)
  {
    phidget_ok(PhidgetStepper_setEngaged(base_stepper_, 0), "PhidgetStepper_setEngaged(0) base");
    phidget_ok(Phidget_close(reinterpret_cast<PhidgetHandle>(base_stepper_)), "Phidget_close base");
    phidget_ok(PhidgetStepper_delete(&base_stepper_), "PhidgetStepper_delete base");
    base_stepper_ = nullptr;
    base_stepper_attached_ = false;
  }

  if (wrist_motor_1_)
  {
    phidget_ok(PhidgetStepper_setEngaged(wrist_motor_1_, 0), "PhidgetStepper_setEngaged(0) wrist1");
    phidget_ok(Phidget_close(reinterpret_cast<PhidgetHandle>(wrist_motor_1_)), "Phidget_close wrist1");
    phidget_ok(PhidgetStepper_delete(&wrist_motor_1_), "PhidgetStepper_delete wrist1");
    wrist_motor_1_ = nullptr;
    wrist_motor_1_attached_ = false;
  }

  if (wrist_motor_2_)
  {
    phidget_ok(PhidgetStepper_setEngaged(wrist_motor_2_, 0), "PhidgetStepper_setEngaged(0) wrist2");
    phidget_ok(Phidget_close(reinterpret_cast<PhidgetHandle>(wrist_motor_2_)), "Phidget_close wrist2");
    phidget_ok(PhidgetStepper_delete(&wrist_motor_2_), "PhidgetStepper_delete wrist2");
    wrist_motor_2_ = nullptr;
    wrist_motor_2_attached_ = false;
  }
  if (claw_stepper_)
  {
    phidget_ok(PhidgetStepper_setEngaged(claw_stepper_, 0), "PhidgetStepper_setEngaged(0) claw");
    phidget_ok(Phidget_close(reinterpret_cast<PhidgetHandle>(claw_stepper_)), "Phidget_close claw");
    phidget_ok(PhidgetStepper_delete(&claw_stepper_), "PhidgetStepper_delete claw");
    claw_stepper_ = nullptr;
    claw_stepper_attached_ = false;
  }
}

void Arm2026System::cleanup_ros_bridge()
{
  executor_running_.store(false);
  if (executor_thread_.joinable())
  {
    executor_thread_.join();
  }

  if (executor_ && comms_node_)
  {
    executor_->remove_node(comms_node_);
  }

  actuator_state_sub_.reset();
  debug_mode_sub_.reset();
  executor_.reset();
  comms_node_.reset();
}

hardware_interface::CallbackReturn Arm2026System::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("Arm2026System"), "Configuring Arm2026System");

  comms_node_ = std::make_shared<rclcpp::Node>("arm_2026_hw_bridge");

  actuator_state_sub_ =
    comms_node_->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/arm_actuator_states",
      10,
      std::bind(&Arm2026System::actuator_state_callback, this, std::placeholders::_1));
  
  debug_mode_sub_ =
    comms_node_->create_subscription<std_msgs::msg::Bool>(
      "/arm_2026/debug_mode",
      10,
      std::bind(&Arm2026System::debug_mode_callback, this, std::placeholders::_1));

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

  open_actuator_serial();

  if (!phidget_ok(PhidgetStepper_create(&base_stepper_), "PhidgetStepper_create base"))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!phidget_ok(
        Phidget_setDeviceSerialNumber(
          reinterpret_cast<PhidgetHandle>(base_stepper_), base_device_serial_),
        "Phidget_setDeviceSerialNumber base"))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!phidget_ok(
        Phidget_setHubPort(
          reinterpret_cast<PhidgetHandle>(base_stepper_), base_hub_port_),
        "Phidget_setHubPort base"))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!phidget_ok(
        Phidget_setChannel(
          reinterpret_cast<PhidgetHandle>(base_stepper_), base_channel_),
        "Phidget_setChannel base"))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!phidget_ok(
        Phidget_openWaitForAttachment(
          reinterpret_cast<PhidgetHandle>(base_stepper_), 5000),
        "Phidget_openWaitForAttachment base"))
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
  }
  else
  {
    if (!phidget_ok(
          PhidgetStepper_setRescaleFactor(base_stepper_, base_rescale_factor_deg_),
          "PhidgetStepper_setRescaleFactor base"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    base_stepper_attached_ = true;
    RCLCPP_INFO(
      rclcpp::get_logger("Arm2026System"),
      "Base Phidget attached on serial %d, hub port %d",
      base_device_serial_,
      base_hub_port_);
  }

  if (!phidget_ok(PhidgetStepper_create(&wrist_motor_1_), "PhidgetStepper_create wrist1"))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!phidget_ok(
        Phidget_setDeviceSerialNumber(
          reinterpret_cast<PhidgetHandle>(wrist_motor_1_), wrist_device_serial_),
        "Phidget_setDeviceSerialNumber wrist1"))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!phidget_ok(
        Phidget_setHubPort(
          reinterpret_cast<PhidgetHandle>(wrist_motor_1_), wrist_motor_1_hub_port_),
        "Phidget_setHubPort wrist1"))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!phidget_ok(
        Phidget_setChannel(
          reinterpret_cast<PhidgetHandle>(wrist_motor_1_), wrist_channel_),
        "Phidget_setChannel wrist1"))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!phidget_ok(
        Phidget_openWaitForAttachment(
          reinterpret_cast<PhidgetHandle>(wrist_motor_1_), 5000),
        "Phidget_openWaitForAttachment wrist1"))
  {
    RCLCPP_WARN(
      rclcpp::get_logger("Arm2026System"),
      "Wrist motor 1 not attached. Continuing in fake-wrist mode.");

    wrist_motor_1_attached_ = false;

    if (wrist_motor_1_ != nullptr)
    {
      PhidgetStepper_delete(&wrist_motor_1_);
      wrist_motor_1_ = nullptr;
    }
  }
  else
  {
    if (!phidget_ok(
          PhidgetStepper_setRescaleFactor(wrist_motor_1_, wrist_motor_1_rescale_factor_deg_),
          "PhidgetStepper_setRescaleFactor wrist1"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    wrist_motor_1_attached_ = true;
    RCLCPP_INFO(
      rclcpp::get_logger("Arm2026System"),
      "Wrist motor 1 attached on serial %d, hub port %d",
      wrist_device_serial_,
      wrist_motor_1_hub_port_);
  }

  if (!phidget_ok(PhidgetStepper_create(&wrist_motor_2_), "PhidgetStepper_create wrist2"))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!phidget_ok(
        Phidget_setDeviceSerialNumber(
          reinterpret_cast<PhidgetHandle>(wrist_motor_2_), wrist_device_serial_),
        "Phidget_setDeviceSerialNumber wrist2"))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!phidget_ok(
        Phidget_setHubPort(
          reinterpret_cast<PhidgetHandle>(wrist_motor_2_), wrist_motor_2_hub_port_),
        "Phidget_setHubPort wrist2"))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!phidget_ok(
        Phidget_setChannel(
          reinterpret_cast<PhidgetHandle>(wrist_motor_2_), wrist_channel_),
        "Phidget_setChannel wrist2"))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!phidget_ok(
        Phidget_openWaitForAttachment(
          reinterpret_cast<PhidgetHandle>(wrist_motor_2_), 5000),
        "Phidget_openWaitForAttachment wrist2"))
  {
    RCLCPP_WARN(
      rclcpp::get_logger("Arm2026System"),
      "Wrist motor 2 not attached. Continuing in fake-wrist mode.");

    wrist_motor_2_attached_ = false;

    if (wrist_motor_2_ != nullptr)
    {
      PhidgetStepper_delete(&wrist_motor_2_);
      wrist_motor_2_ = nullptr;
    }
  }
  else
  {
    if (!phidget_ok(
          PhidgetStepper_setRescaleFactor(wrist_motor_2_, wrist_motor_2_rescale_factor_deg_),
          "PhidgetStepper_setRescaleFactor wrist2"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    wrist_motor_2_attached_ = true;
    RCLCPP_INFO(
      rclcpp::get_logger("Arm2026System"),
      "Wrist motor 2 attached on serial %d, hub port %d",
      wrist_device_serial_,
      wrist_motor_2_hub_port_);
  }

  if (!phidget_ok(PhidgetStepper_create(&claw_stepper_), "PhidgetStepper_create claw"))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!phidget_ok(
        Phidget_setDeviceSerialNumber(
          reinterpret_cast<PhidgetHandle>(claw_stepper_), claw_device_serial_),
        "Phidget_setDeviceSerialNumber claw"))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!phidget_ok(
        Phidget_setHubPort(
          reinterpret_cast<PhidgetHandle>(claw_stepper_), claw_hub_port_),
        "Phidget_setHubPort claw"))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!phidget_ok(
        Phidget_setChannel(
          reinterpret_cast<PhidgetHandle>(claw_stepper_), claw_channel_),
        "Phidget_setChannel claw"))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!phidget_ok(
        Phidget_openWaitForAttachment(
          reinterpret_cast<PhidgetHandle>(claw_stepper_), 5000),
        "Phidget_openWaitForAttachment claw"))
  {
    RCLCPP_WARN(
      rclcpp::get_logger("Arm2026System"),
      "Claw Phidget not attached. Continuing in fake-claw mode.");

    claw_stepper_attached_ = false;

    if (claw_stepper_ != nullptr)
    {
      PhidgetStepper_delete(&claw_stepper_);
      claw_stepper_ = nullptr;
    }
  }
  else
  {
    if (!phidget_ok(
          PhidgetStepper_setRescaleFactor(claw_stepper_, claw_rescale_factor_deg_),
          "PhidgetStepper_setRescaleFactor claw"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    claw_stepper_attached_ = true;
    RCLCPP_INFO(
      rclcpp::get_logger("Arm2026System"),
      "Claw Phidget attached on serial %d, hub port %d",
      claw_device_serial_,
      claw_hub_port_);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Arm2026System::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("Arm2026System"), "Activating Arm2026System");

  if (actuator_serial_fd_ < 0)
  {
    open_actuator_serial();
  }

  if (base_stepper_ && base_stepper_attached_)
  {
    if (!phidget_ok(
          PhidgetStepper_setAcceleration(base_stepper_, base_acceleration_deg_),
          "PhidgetStepper_setAcceleration base"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!phidget_ok(
          PhidgetStepper_setVelocityLimit(base_stepper_, base_velocity_limit_deg_),
          "PhidgetStepper_setVelocityLimit base"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!phidget_ok(
          PhidgetStepper_setEngaged(base_stepper_, 1),
          "PhidgetStepper_setEngaged base"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    double base_pos_deg = 0.0;
    if (!phidget_ok(
          PhidgetStepper_getPosition(base_stepper_, &base_pos_deg),
          "PhidgetStepper_getPosition base"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    hw_states_[BASE_IDX] = base_pos_deg * M_PI / 180.0;
    hw_commands_[BASE_IDX] = hw_states_[BASE_IDX];

    if (!phidget_ok(
          PhidgetStepper_setTargetPosition(base_stepper_, base_pos_deg),
          "PhidgetStepper_setTargetPosition base"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  else
  {
    hw_states_[BASE_IDX] = hw_commands_[BASE_IDX];
  }

  if (wrist_motor_1_ && wrist_motor_1_attached_)
  {
    if (!phidget_ok(
          PhidgetStepper_setAcceleration(wrist_motor_1_, wrist_acceleration_deg_),
          "PhidgetStepper_setAcceleration wrist1"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!phidget_ok(
          PhidgetStepper_setVelocityLimit(wrist_motor_1_, wrist_velocity_limit_deg_),
          "PhidgetStepper_setVelocityLimit wrist1"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!phidget_ok(
          PhidgetStepper_setEngaged(wrist_motor_1_, 1),
          "PhidgetStepper_setEngaged wrist1"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  if (wrist_motor_2_ && wrist_motor_2_attached_)
  {
    if (!phidget_ok(
          PhidgetStepper_setAcceleration(wrist_motor_2_, wrist_acceleration_deg_),
          "PhidgetStepper_setAcceleration wrist2"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!phidget_ok(
          PhidgetStepper_setVelocityLimit(wrist_motor_2_, wrist_velocity_limit_deg_),
          "PhidgetStepper_setVelocityLimit wrist2"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!phidget_ok(
          PhidgetStepper_setEngaged(wrist_motor_2_, 1),
          "PhidgetStepper_setEngaged wrist2"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  hw_states_[SHOULDER_IDX] = 0.0;
  hw_states_[ELBOW_IDX] = 0.0;
  hw_states_[CLAW_IDX] = 0.0;
  hw_commands_[SHOULDER_IDX] = 0.0;
  hw_commands_[ELBOW_IDX] = 0.0;
  hw_commands_[CLAW_IDX] = 0.0;

  actuator_command_shoulder_ = 0.0;
  actuator_command_elbow_ = 0.0;
  actuator_state_shoulder_ = 0.0;
  actuator_state_elbow_ = 0.0;
  actuator_state_received_.store(false);

  actuator_estimated_shoulder_pos_ = 0.0;
  actuator_estimated_elbow_pos_ = 0.0;

  hw_states_[WRIST_ROLL_IDX] = 0.0;
  hw_states_[WRIST_TWIST_IDX] = 0.0;
  hw_commands_[WRIST_ROLL_IDX] = 0.0;
  hw_commands_[WRIST_TWIST_IDX] = 0.0;

  send_actuator_packet(ACT_STOP, ACT_STOP);
  
  if (claw_stepper_ && claw_stepper_attached_)
  {
    if (!phidget_ok(
          PhidgetStepper_setAcceleration(claw_stepper_, claw_acceleration_deg_),
          "PhidgetStepper_setAcceleration claw"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!phidget_ok(
          PhidgetStepper_setVelocityLimit(claw_stepper_, claw_velocity_limit_deg_),
          "PhidgetStepper_setVelocityLimit claw"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!phidget_ok(
          PhidgetStepper_setEngaged(claw_stepper_, 1),
          "PhidgetStepper_setEngaged claw"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Arm2026System::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("Arm2026System"), "Deactivating Arm2026System");

  send_actuator_packet(ACT_STOP, ACT_STOP);
  cleanup_phidgets();
  cleanup_ros_bridge();
  close_actuator_serial();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Arm2026System::read(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  if (base_stepper_ && base_stepper_attached_)
  {
    double base_pos_deg = 0.0;
    if (phidget_ok(
          PhidgetStepper_getPosition(base_stepper_, &base_pos_deg),
          "PhidgetStepper_getPosition base"))
    {
      hw_states_[BASE_IDX] = base_pos_deg * M_PI / 180.0;
    }
  }
  else
  {
    hw_states_[BASE_IDX] = hw_commands_[BASE_IDX];
  }

  if (actuator_state_received_.load())
  {
    hw_states_[SHOULDER_IDX] = actuator_state_shoulder_;
    hw_states_[ELBOW_IDX] = actuator_state_elbow_;
  }
  else
  {
    const double dt = period.seconds();

    if (actuator_command_shoulder_ > actuator_command_deadband_)
    {
      actuator_estimated_shoulder_pos_ += actuator_estimated_speed_rad_s_ * dt;
    }
    else if (actuator_command_shoulder_ < -actuator_command_deadband_)
    {
      actuator_estimated_shoulder_pos_ -= actuator_estimated_speed_rad_s_ * dt;
    }

    if (actuator_command_elbow_ > actuator_command_deadband_)
    {
      actuator_estimated_elbow_pos_ += actuator_estimated_speed_rad_s_ * dt;
    }
    else if (actuator_command_elbow_ < -actuator_command_deadband_)
    {
      actuator_estimated_elbow_pos_ -= actuator_estimated_speed_rad_s_ * dt;
    }

    actuator_estimated_shoulder_pos_ = clampf(
      actuator_estimated_shoulder_pos_, shoulder_min_, shoulder_max_);
    actuator_estimated_elbow_pos_ = clampf(
      actuator_estimated_elbow_pos_, elbow_min_, elbow_max_);

    hw_states_[SHOULDER_IDX] = actuator_estimated_shoulder_pos_;
    hw_states_[ELBOW_IDX] = actuator_estimated_elbow_pos_;
  }

  hw_states_[WRIST_ROLL_IDX] = hw_commands_[WRIST_ROLL_IDX];
  hw_states_[WRIST_TWIST_IDX] = hw_commands_[WRIST_TWIST_IDX];
  
  hw_states_[CLAW_IDX] = hw_commands_[CLAW_IDX];

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Arm2026System::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (!debug_mode_enabled_.load())
  {
    hw_commands_[BASE_IDX] =
      clampf(hw_commands_[BASE_IDX], base_min_, base_max_);

    hw_commands_[SHOULDER_IDX] =
      clampf(hw_commands_[SHOULDER_IDX], shoulder_min_, shoulder_max_);

    hw_commands_[ELBOW_IDX] =
      clampf(hw_commands_[ELBOW_IDX], elbow_min_, elbow_max_);

    hw_commands_[WRIST_ROLL_IDX] =
      clampf(hw_commands_[WRIST_ROLL_IDX], wrist_roll_min_, wrist_roll_max_);

    hw_commands_[WRIST_TWIST_IDX] =
      clampf(hw_commands_[WRIST_TWIST_IDX], wrist_twist_min_, wrist_twist_max_);

    hw_commands_[CLAW_IDX] =
      clampf(hw_commands_[CLAW_IDX], claw_min_, claw_max_);
  }

  actuator_command_shoulder_ = hw_commands_[SHOULDER_IDX];
  actuator_command_elbow_ = hw_commands_[ELBOW_IDX];

  const double base_target_deg = hw_commands_[BASE_IDX] * 180.0 / M_PI;
  const double wrist_roll_cmd = hw_commands_[WRIST_ROLL_IDX];
  const double wrist_twist_cmd = hw_commands_[WRIST_TWIST_IDX];

  const double wrist_motor_1_cmd = wrist_roll_cmd + wrist_twist_cmd;
  const double wrist_motor_2_cmd = wrist_roll_cmd - wrist_twist_cmd;

  const double wrist_motor_1_cmd_deg = wrist_motor_1_cmd * 180.0 / M_PI;
  const double wrist_motor_2_cmd_deg = wrist_motor_2_cmd * 180.0 / M_PI;
  
  const double claw_target_deg = hw_commands_[CLAW_IDX] * 180.0 / M_PI;

  const uint8_t shoulder_byte = command_to_byte(actuator_command_shoulder_);
  const uint8_t elbow_byte = command_to_byte(actuator_command_elbow_);

  if (comms_node_) {
    RCLCPP_INFO_THROTTLE(
      comms_node_->get_logger(),
      *comms_node_->get_clock(),
      1000,
      "write() | debug=%s | base_deg=%.3f | shoulder_cmd=%.3f -> %u | elbow_cmd=%.3f -> %u | wrist1_deg=%.3f | wrist2_deg=%.3f | claw_deg=%.3f | shoulder_est=%.3f | elbow_est=%.3f",
      debug_mode_enabled_.load() ? "ON" : "OFF",
      base_target_deg,
      actuator_command_shoulder_, static_cast<unsigned>(shoulder_byte),
      actuator_command_elbow_, static_cast<unsigned>(elbow_byte),
      wrist_motor_1_cmd_deg,
      wrist_motor_2_cmd_deg,
      claw_target_deg,
      actuator_estimated_shoulder_pos_,
      actuator_estimated_elbow_pos_);
  }

  if (base_stepper_ && base_stepper_attached_)
  {
    int engaged = 0;
    if (phidget_ok(
          PhidgetStepper_getEngaged(base_stepper_, &engaged),
          "PhidgetStepper_getEngaged base"))
    {
      if (!engaged)
      {
        RCLCPP_WARN(
          rclcpp::get_logger("Arm2026System"),
          "Base stepper was disengaged in write(); attempting to re-engage.");
        phidget_ok(
          PhidgetStepper_setEngaged(base_stepper_, 1),
          "PhidgetStepper_setEngaged base re-engage");
      }
    }

    phidget_ok(
      PhidgetStepper_setTargetPosition(base_stepper_, base_target_deg),
      "PhidgetStepper_setTargetPosition base");
  }

  if (actuator_serial_fd_ < 0)
  {
    open_actuator_serial();
  }
  if (actuator_serial_fd_ >= 0)
  {
    send_actuator_packet(shoulder_byte, elbow_byte);
  }

  if (wrist_motor_1_ && wrist_motor_1_attached_)
  {
    int engaged = 0;
    if (phidget_ok(
          PhidgetStepper_getEngaged(wrist_motor_1_, &engaged),
          "PhidgetStepper_getEngaged wrist1"))
    {
      if (!engaged)
      {
        RCLCPP_WARN(
          rclcpp::get_logger("Arm2026System"),
          "Wrist motor 1 was disengaged in write(); attempting to re-engage.");
        phidget_ok(
          PhidgetStepper_setEngaged(wrist_motor_1_, 1),
          "PhidgetStepper_setEngaged wrist1 re-engage");
      }
    }

    phidget_ok(
      PhidgetStepper_setTargetPosition(wrist_motor_1_, wrist_motor_1_cmd_deg),
      "PhidgetStepper_setTargetPosition wrist1");
  }

  if (wrist_motor_2_ && wrist_motor_2_attached_)
  {
    int engaged = 0;
    if (phidget_ok(
          PhidgetStepper_getEngaged(wrist_motor_2_, &engaged),
          "PhidgetStepper_getEngaged wrist2"))
    {
      if (!engaged)
      {
        RCLCPP_WARN(
          rclcpp::get_logger("Arm2026System"),
          "Wrist motor 2 was disengaged in write(); attempting to re-engage.");
        phidget_ok(
          PhidgetStepper_setEngaged(wrist_motor_2_, 1),
          "PhidgetStepper_setEngaged wrist2 re-engage");
      }
    }

    phidget_ok(
      PhidgetStepper_setTargetPosition(wrist_motor_2_, wrist_motor_2_cmd_deg),
      "PhidgetStepper_setTargetPosition wrist2");
  }
  
  if (claw_stepper_ && claw_stepper_attached_)
  {
    int engaged = 0;
    if (phidget_ok(
          PhidgetStepper_getEngaged(claw_stepper_, &engaged),
          "PhidgetStepper_getEngaged claw"))
    {
      if (!engaged)
      {
        RCLCPP_WARN(
          rclcpp::get_logger("Arm2026System"),
          "Claw stepper was disengaged in write(); attempting to re-engage.");
        phidget_ok(
          PhidgetStepper_setEngaged(claw_stepper_, 1),
          "PhidgetStepper_setEngaged claw re-engage");
      }
    }

    phidget_ok(
      PhidgetStepper_setTargetPosition(claw_stepper_, claw_target_deg),
      "PhidgetStepper_setTargetPosition claw");
  }

  return hardware_interface::return_type::OK;
}

}  // namespace arm_2026

PLUGINLIB_EXPORT_CLASS(
  arm_2026::Arm2026System,
  hardware_interface::SystemInterface)
