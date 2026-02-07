#include "arm_control/mini_arm_hardware.hpp"
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

namespace arm_control
{

static std::vector<double> parse_csv_doubles(const std::string & s)
{
  std::vector<double> out;
  std::stringstream ss(s);
  std::string item;

  while (std::getline(ss, item, ',')) {
    // trim spaces
    item.erase(0, item.find_first_not_of(" \t"));
    item.erase(item.find_last_not_of(" \t") + 1);

    if (!item.empty()) {
      out.push_back(std::stod(item));
    }
  }
  return out;
}

hardware_interface::CallbackReturn MG996RServoSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Optional params from URDF <hardware> <param name="...">
  if (info_.hardware_parameters.count("command_topic")) {
    command_topic_ = info_.hardware_parameters.at("command_topic");
  }
  if (info_.hardware_parameters.count("center_deg")) {
    center_deg_ = std::stod(info_.hardware_parameters.at("center_deg"));
  }
  if (info_.hardware_parameters.count("deg_min")) {
    deg_min_ = std::stod(info_.hardware_parameters.at("deg_min"));
  }
  if (info_.hardware_parameters.count("deg_max")) {
    deg_max_ = std::stod(info_.hardware_parameters.at("deg_max"));
  }

  // Future-proof: per-joint sign flips, example: "1,-1,1,1,1,1"
  
  // THIS MAY CAUSE ERRORS IF UNEXPECTED BEHAVIOUR IS OBSERVED
  
  joint_sign_.assign(info_.joints.size(), 1.0);
  if (info_.hardware_parameters.count("joint_signs")) {
    const auto vals = parse_csv_doubles(info_.hardware_parameters.at("joint_signs"));
    for (size_t i = 0; i < std::min(vals.size(), joint_sign_.size()); i++) {
      joint_sign_[i] = (vals[i] >= 0.0) ? 1.0 : -1.0;
    }
  }

  const size_t n = info_.joints.size();
  hw_states_pos_.assign(n, 0.0);
  hw_commands_pos_.assign(n, 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MG996RServoSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION,
        &hw_states_pos_[i]
      )
    );
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MG996RServoSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION,
        &hw_commands_pos_[i]
      )
    );
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn MG996RServoSystem::on_configure(const rclcpp_lifecycle::State &)
{
  if (!rclcpp::ok()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  node_ = std::make_shared<rclcpp::Node>("mg996r_servo_hw");
  pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(command_topic_, 10);

  exec_.add_node(node_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MG996RServoSystem::on_activate(const rclcpp_lifecycle::State &)
{
  active_ = true;

  // Publish a safe "center" pose on activate (0 rad maps to center_deg)
  std_msgs::msg::Float32MultiArray out;
  out.data.resize(info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); i++) {
    float deg = static_cast<float>(center_deg_);
    deg = clamp_f(deg, static_cast<float>(deg_min_), static_cast<float>(deg_max_));
    out.data[i] = deg;

    // Open-loop state initialization
    hw_states_pos_[i] = 0.0;
  }

  pub_->publish(out);
  exec_.spin_some();  // one-time spin is fine here

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MG996RServoSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  active_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MG996RServoSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Open loop: assume robot achieved commanded position
  for (size_t i = 0; i < hw_states_pos_.size(); i++) {
    hw_states_pos_[i] = hw_commands_pos_[i];
  }
  // Do not spin here (avoid jitter)
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MG996RServoSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_ || !pub_) {
    return hardware_interface::return_type::OK;
  }

  std_msgs::msg::Float32MultiArray out;
  out.data.resize(hw_commands_pos_.size());

  // Mapping: rad -> degrees with 0 rad = center_deg
  for (size_t i = 0; i < hw_commands_pos_.size(); i++) {
    const double rad = hw_commands_pos_[i] * joint_sign_[i];
    const double deg = center_deg_ + rad * rad_to_deg_scale_;

    const float deg_f = clamp_f(
      static_cast<float>(deg),
      static_cast<float>(deg_min_),
      static_cast<float>(deg_max_)
    );

    out.data[i] = deg_f;
  }

  pub_->publish(out);
  // Do not spin here (avoid jitter)
  return hardware_interface::return_type::OK;
}

}  // namespace arm_control

PLUGINLIB_EXPORT_CLASS(mini_arm_ros2::MG996RServoSystem, hardware_interface::SystemInterface)

