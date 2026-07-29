#ifndef ARM_DASHBOARD__ARM_DASHBOARD_PANEL_HPP_
#define ARM_DASHBOARD__ARM_DASHBOARD_PANEL_HPP_

#include <map>
#include <memory>
#include <string>

#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace arm_dashboard
{

struct JointDisplay
{
  QLabel * name_label{nullptr};
  QLabel * position_label{nullptr};
  QLabel * velocity_label{nullptr};
  QLabel * feedback_label{nullptr};
};

class ArmDashboardPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ArmDashboardPanel(QWidget * parent = nullptr);

  void onInitialize() override;

private:
  void build_ui();

  void create_joint_card(
    QVBoxLayout * parent_layout,
    const std::string & joint_name,
    const QString & display_name,
    const QString & feedback_description);

  void joint_state_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg);

  void joy_callback(
    const sensor_msgs::msg::Joy::SharedPtr msg);

  void command_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  QLabel * title_label_{nullptr};
  QLabel * status_label_{nullptr};
  QLabel * joint_state_note_label_{nullptr};

  QLabel * joystick_status_label_{nullptr};
  QLabel * joystick_axes_label_{nullptr};
  QLabel * joystick_buttons_label_{nullptr};

  QLabel * command_status_label_{nullptr};
  QLabel * command_values_label_{nullptr};

  QPushButton * software_stop_button_{nullptr};

  std::map<std::string, JointDisplay> joint_displays_;

  rclcpp::Node::SharedPtr raw_node_;

  rclcpp::Subscription<
    sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  rclcpp::Subscription<
    sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  rclcpp::Subscription<
    std_msgs::msg::Float64MultiArray>::SharedPtr command_sub_;
};

}  // namespace arm_dashboard

#endif  // ARM_DASHBOARD__ARM_DASHBOARD_PANEL_HPP_
