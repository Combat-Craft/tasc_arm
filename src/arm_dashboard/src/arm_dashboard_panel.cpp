#include "arm_dashboard/arm_dashboard_panel.hpp"

#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>

#include <QFrame>
#include <QHBoxLayout>
#include <QMetaObject>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace arm_dashboard
{

ArmDashboardPanel::ArmDashboardPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  build_ui();
}

void ArmDashboardPanel::build_ui()
{
  auto * main_layout = new QVBoxLayout(this);

  main_layout->setContentsMargins(12, 12, 12, 12);
  main_layout->setSpacing(10);

  // ===========================================================================
  // HEADER
  // ===========================================================================

  title_label_ = new QLabel("TASC Arm Dashboard", this);
  title_label_->setAlignment(Qt::AlignCenter);
  title_label_->setStyleSheet(
    "font-size: 20px;"
    "font-weight: bold;"
    "padding: 8px;");

  status_label_ = new QLabel(
    "ROS 2 status: waiting for RViz initialization",
    this);

  status_label_->setAlignment(Qt::AlignCenter);
  status_label_->setWordWrap(true);
  status_label_->setStyleSheet(
    "background-color: #374151;"
    "color: white;"
    "border-radius: 8px;"
    "padding: 8px;");

  main_layout->addWidget(title_label_);
  main_layout->addWidget(status_label_);

  // ===========================================================================
  // TWO-COLUMN CONTENT
  // ===========================================================================

  auto * content_layout = new QHBoxLayout();
  content_layout->setSpacing(12);

  auto * left_column_widget = new QWidget(this);
  auto * left_column_layout = new QVBoxLayout(left_column_widget);

  left_column_layout->setContentsMargins(0, 0, 0, 0);
  left_column_layout->setSpacing(8);

  auto * right_column_widget = new QWidget(this);
  auto * right_column_layout = new QVBoxLayout(right_column_widget);

  right_column_layout->setContentsMargins(0, 0, 0, 0);
  right_column_layout->setSpacing(8);

  // ===========================================================================
  // LEFT COLUMN: JOINT STATES
  // ===========================================================================

  auto * joint_heading =
    new QLabel("Joint Status", left_column_widget);

  joint_heading->setStyleSheet(
    "font-size: 16px;"
    "font-weight: bold;"
    "padding: 4px;");

  joint_state_note_label_ = new QLabel(
    "Positions are relative to startup unless absolute sensors "
    "or homing are available.",
    left_column_widget);

  joint_state_note_label_->setWordWrap(true);
  joint_state_note_label_->setStyleSheet(
    "background-color: #422006;"
    "color: #fde68a;"
    "border: 1px solid #92400e;"
    "border-radius: 8px;"
    "padding: 8px;");

  left_column_layout->addWidget(joint_heading);
  left_column_layout->addWidget(joint_state_note_label_);

  create_joint_card(
    left_column_layout,
    "base_yaw",
    "Base Yaw",
    "Relative stepper position since startup");

  create_joint_card(
    left_column_layout,
    "shoulder_extension",
    "Shoulder",
    "Estimated position, no physical encoder");

  create_joint_card(
    left_column_layout,
    "elbow_extension",
    "Elbow",
    "Estimated position, no physical encoder");

  create_joint_card(
    left_column_layout,
    "wrist_roll",
    "Wrist Roll",
    "Relative differential stepper position");

  create_joint_card(
    left_column_layout,
    "wrist_twist",
    "Wrist Twist",
    "Relative differential stepper position");

  create_joint_card(
    left_column_layout,
    "claw",
    "Claw",
    "Feedback availability depends on hardware");

  left_column_layout->addStretch();

  // ===========================================================================
  // RIGHT COLUMN: JOYSTICK
  // ===========================================================================

  auto * joystick_heading =
    new QLabel("Joystick Diagnostics", right_column_widget);

  joystick_heading->setStyleSheet(
    "font-size: 16px;"
    "font-weight: bold;"
    "padding: 4px;");

  joystick_status_label_ =
    new QLabel("Joystick: waiting for /arm/joy", right_column_widget);

  joystick_status_label_->setAlignment(Qt::AlignCenter);
  joystick_status_label_->setStyleSheet(
    "background-color: #374151;"
    "color: white;"
    "border-radius: 8px;"
    "font-weight: bold;"
    "padding: 8px;");

  joystick_axes_label_ =
    new QLabel("Axes: no data", right_column_widget);

  joystick_axes_label_->setWordWrap(true);
  joystick_axes_label_->setAlignment(
    Qt::AlignLeft | Qt::AlignTop);

  joystick_axes_label_->setStyleSheet(
    "background-color: #111827;"
    "color: white;"
    "border: 1px solid #374151;"
    "border-radius: 8px;"
    "font-family: monospace;"
    "padding: 8px;");

  joystick_buttons_label_ =
    new QLabel("Buttons: no data", right_column_widget);

  joystick_buttons_label_->setWordWrap(true);
  joystick_buttons_label_->setAlignment(
    Qt::AlignLeft | Qt::AlignTop);

  joystick_buttons_label_->setStyleSheet(
    "background-color: #111827;"
    "color: white;"
    "border: 1px solid #374151;"
    "border-radius: 8px;"
    "font-family: monospace;"
    "padding: 8px;");

  right_column_layout->addWidget(joystick_heading);
  right_column_layout->addWidget(joystick_status_label_);
  right_column_layout->addWidget(joystick_axes_label_);
  right_column_layout->addWidget(joystick_buttons_label_);

  // ===========================================================================
  // RIGHT COLUMN: CONTROLLER COMMANDS
  // ===========================================================================

  auto * command_heading =
    new QLabel("Controller Commands", right_column_widget);

  command_heading->setStyleSheet(
    "font-size: 16px;"
    "font-weight: bold;"
    "padding: 4px;");

  command_status_label_ =
    new QLabel(
      "Commands: waiting for /manual_controller/commands",
      right_column_widget);

  command_status_label_->setAlignment(Qt::AlignCenter);
  command_status_label_->setWordWrap(true);
  command_status_label_->setStyleSheet(
    "background-color: #374151;"
    "color: white;"
    "border-radius: 8px;"
    "font-weight: bold;"
    "padding: 8px;");

  command_values_label_ =
    new QLabel("No command data", right_column_widget);

  command_values_label_->setWordWrap(true);
  command_values_label_->setAlignment(
    Qt::AlignLeft | Qt::AlignTop);

  command_values_label_->setStyleSheet(
    "background-color: #111827;"
    "color: white;"
    "border: 1px solid #374151;"
    "border-radius: 8px;"
    "font-family: monospace;"
    "padding: 8px;");

  right_column_layout->addWidget(command_heading);
  right_column_layout->addWidget(command_status_label_);
  right_column_layout->addWidget(command_values_label_);

  // ===========================================================================
  // SOFTWARE STOP
  // ===========================================================================

  software_stop_button_ =
    new QPushButton("SOFTWARE STOP", right_column_widget);

  software_stop_button_->setMinimumHeight(44);
  software_stop_button_->setStyleSheet(
    "QPushButton {"
    "  background-color: #b91c1c;"
    "  color: white;"
    "  border: none;"
    "  border-radius: 8px;"
    "  font-weight: bold;"
    "  padding: 10px;"
    "}"
    "QPushButton:hover {"
    "  background-color: #dc2626;"
    "}"
    "QPushButton:pressed {"
    "  background-color: #7f1d1d;"
    "}");

  connect(
    software_stop_button_,
    &QPushButton::clicked,
    this,
    [this]()
    {
      status_label_->setText(
        "Software stop button pressed. "
        "ROS stop publishing will be added later.");

      status_label_->setStyleSheet(
        "background-color: #b91c1c;"
        "color: white;"
        "border-radius: 8px;"
        "padding: 8px;");
    });

  right_column_layout->addStretch();
  right_column_layout->addWidget(software_stop_button_);

  content_layout->addWidget(left_column_widget, 3);
  content_layout->addWidget(right_column_widget, 2);

  main_layout->addLayout(content_layout, 1);

  setLayout(main_layout);
}

void ArmDashboardPanel::create_joint_card(
  QVBoxLayout * parent_layout,
  const std::string & joint_name,
  const QString & display_name,
  const QString & feedback_description)
{
  auto * card = new QFrame(this);

  card->setStyleSheet(
    "QFrame {"
    "  background-color: #111827;"
    "  border: 1px solid #374151;"
    "  border-radius: 8px;"
    "}");

  auto * card_layout = new QVBoxLayout(card);
  card_layout->setContentsMargins(10, 8, 10, 8);
  card_layout->setSpacing(4);

  JointDisplay display;

  display.name_label = new QLabel(display_name, card);
  display.name_label->setStyleSheet(
    "font-size: 15px;"
    "font-weight: bold;"
    "color: white;"
    "border: none;");

  display.position_label =
    new QLabel("Position: waiting for data", card);

  display.position_label->setStyleSheet(
    "color: white;"
    "font-family: monospace;"
    "border: none;");

  display.velocity_label =
    new QLabel("Velocity: waiting for data", card);

  display.velocity_label->setStyleSheet(
    "color: white;"
    "font-family: monospace;"
    "border: none;");

  display.feedback_label =
    new QLabel(feedback_description, card);

  display.feedback_label->setWordWrap(true);
  display.feedback_label->setStyleSheet(
    "color: #fbbf24;"
    "font-size: 11px;"
    "border: none;");

  card_layout->addWidget(display.name_label);
  card_layout->addWidget(display.position_label);
  card_layout->addWidget(display.velocity_label);
  card_layout->addWidget(display.feedback_label);

  parent_layout->addWidget(card);

  joint_displays_[joint_name] = display;
}

void ArmDashboardPanel::onInitialize()
{
  auto node_abstraction =
    getDisplayContext()
    ->getRosNodeAbstraction()
    .lock();

  if (!node_abstraction)
  {
    status_label_->setText(
      "ROS 2 status: failed to access RViz node");

    status_label_->setStyleSheet(
      "background-color: #b91c1c;"
      "color: white;"
      "border-radius: 8px;"
      "padding: 8px;");

    return;
  }

  raw_node_ =
    node_abstraction->get_raw_node();

  if (!raw_node_)
  {
    status_label_->setText(
      "ROS 2 status: RViz node unavailable");

    status_label_->setStyleSheet(
      "background-color: #b91c1c;"
      "color: white;"
      "border-radius: 8px;"
      "padding: 8px;");

    return;
  }

  joint_state_sub_ =
    raw_node_->create_subscription<
      sensor_msgs::msg::JointState>(
        "/joint_states",
        10,
        std::bind(
          &ArmDashboardPanel::joint_state_callback,
          this,
          std::placeholders::_1));

  joy_sub_ =
    raw_node_->create_subscription<
      sensor_msgs::msg::Joy>(
        "/arm/joy",
        10,
        std::bind(
          &ArmDashboardPanel::joy_callback,
          this,
          std::placeholders::_1));

  command_sub_ =
    raw_node_->create_subscription<
      std_msgs::msg::Float64MultiArray>(
        "/manual_controller/commands",
        10,
        std::bind(
          &ArmDashboardPanel::command_callback,
          this,
          std::placeholders::_1));

  status_label_->setText(
    "ROS 2 status: connected through RViz");

  status_label_->setStyleSheet(
    "background-color: #15803d;"
    "color: white;"
    "border-radius: 8px;"
    "padding: 8px;");

  RCLCPP_INFO(
    raw_node_->get_logger(),
    "TASC Arm Dashboard panel initialized");
}

void ArmDashboardPanel::joint_state_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  for (std::size_t i = 0; i < msg->name.size(); ++i)
  {
    const auto display_iterator =
      joint_displays_.find(msg->name[i]);

    if (display_iterator == joint_displays_.end())
    {
      continue;
    }

    QString position_text = "Position: unavailable";
    QString velocity_text = "Velocity: unavailable";

    if (i < msg->position.size())
    {
      std::ostringstream stream;
      stream << std::fixed
             << std::setprecision(4)
             << msg->position[i];

      position_text =
        QString::fromStdString(
          "Position: " + stream.str() + " rad");
    }

    if (i < msg->velocity.size())
    {
      std::ostringstream stream;
      stream << std::fixed
             << std::setprecision(4)
             << msg->velocity[i];

      velocity_text =
        QString::fromStdString(
          "Velocity: " + stream.str() + " rad/s");
    }

    JointDisplay display =
      display_iterator->second;

    QMetaObject::invokeMethod(
      this,
      [display, position_text, velocity_text]()
      {
        if (display.position_label)
        {
          display.position_label->setText(position_text);
        }

        if (display.velocity_label)
        {
          display.velocity_label->setText(velocity_text);
        }
      },
      Qt::QueuedConnection);
  }
}

void ArmDashboardPanel::joy_callback(
  const sensor_msgs::msg::Joy::SharedPtr msg)
{
  const auto get_axis_value =
    [&msg](std::size_t index) -> double
    {
      if (index >= msg->axes.size())
      {
        return 0.0;
      }

      return static_cast<double>(msg->axes[index]);
    };

  const auto get_button_value =
    [&msg](std::size_t index) -> int
    {
      if (index >= msg->buttons.size())
      {
        return 0;
      }

      return msg->buttons[index];
    };

  const double base_axis =
    get_axis_value(2);

  const double shoulder_elbow_axis =
    get_axis_value(1);

  const double wrist_twist_axis =
    get_axis_value(4);

  const double wrist_roll_axis =
    get_axis_value(5);

  const bool deadman_pressed =
    get_button_value(0) != 0;

  const bool elbow_mode =
    get_button_value(1) != 0;

  const bool claw_close_pressed =
    get_button_value(4) != 0;

  const bool claw_open_pressed =
    get_button_value(5) != 0;

  std::ostringstream axes_stream;
  axes_stream << std::fixed << std::setprecision(3);

  axes_stream
    << "Base Joint\n"
    << "  Axis: " << base_axis << "\n\n"
    << (elbow_mode ? "Elbow Joint\n" : "Shoulder Joint\n")
    << "  Axis: " << shoulder_elbow_axis << "\n\n"
    << "Wrist Twist\n"
    << "  Axis: " << wrist_twist_axis << "\n\n"
    << "Wrist Roll\n"
    << "  Axis: " << wrist_roll_axis;

  std::ostringstream buttons_stream;

  buttons_stream
    << "Deadman\n"
    << "  " << (deadman_pressed ? "ON" : "OFF") << "\n\n"
    << "Shoulder / Elbow Control\n"
    << "  " << (elbow_mode ? "ELBOW" : "SHOULDER") << "\n\n"
    << "Claw Close\n"
    << "  " << (claw_close_pressed ? "PRESSED" : "RELEASED") << "\n\n"
    << "Claw Open\n"
    << "  " << (claw_open_pressed ? "PRESSED" : "RELEASED");

  const QString axes_text =
    QString::fromStdString(
      axes_stream.str());

  const QString buttons_text =
    QString::fromStdString(
      buttons_stream.str());

  QMetaObject::invokeMethod(
    this,
    [this, axes_text, buttons_text, deadman_pressed]()
    {
      joystick_status_label_->setText(
        deadman_pressed
        ? "Joystick: active"
        : "Joystick: connected, deadman released");

      joystick_status_label_->setStyleSheet(
        deadman_pressed
        ? "background-color: #15803d;"
          "color: white;"
          "border-radius: 8px;"
          "font-weight: bold;"
          "padding: 8px;"
        : "background-color: #b45309;"
          "color: white;"
          "border-radius: 8px;"
          "font-weight: bold;"
          "padding: 8px;");

      joystick_axes_label_->setText(
        axes_text);

      joystick_buttons_label_->setText(
        buttons_text);
    },
    Qt::QueuedConnection);
}

void ArmDashboardPanel::command_callback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  static const std::array<const char *, 6> command_names{
    "Base Yaw",
    "Shoulder",
    "Elbow",
    "Wrist Roll",
    "Wrist Twist",
    "Claw"
  };

  std::ostringstream output;
  output << std::fixed << std::setprecision(3);

  bool any_active = false;

  for (std::size_t i = 0; i < command_names.size(); ++i)
  {
    const double value =
      i < msg->data.size()
      ? msg->data[i]
      : 0.0;

    const bool active =
      std::fabs(value) > 0.0001;

    any_active =
      any_active || active;

    output
      << command_names[i]
      << ": "
      << value;

    if (active)
    {
      output << "  ACTIVE";
    }

    output << "\n";
  }

  const QString values_text =
    QString::fromStdString(
      output.str());

  QMetaObject::invokeMethod(
    this,
    [this, values_text, any_active]()
    {
      command_values_label_->setText(
        values_text);

      command_status_label_->setText(
        any_active
        ? "Commands: arm movement requested"
        : "Commands: all outputs zero");

      command_status_label_->setStyleSheet(
        any_active
        ? "background-color: #15803d;"
          "color: white;"
          "border-radius: 8px;"
          "font-weight: bold;"
          "padding: 8px;"
        : "background-color: #374151;"
          "color: white;"
          "border-radius: 8px;"
          "font-weight: bold;"
          "padding: 8px;");
    },
    Qt::QueuedConnection);
}

}  // namespace arm_dashboard

PLUGINLIB_EXPORT_CLASS(
  arm_dashboard::ArmDashboardPanel,
  rviz_common::Panel)
