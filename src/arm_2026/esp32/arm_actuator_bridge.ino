#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float64_multi_array.h>

#define LED_PIN 13

// -----------------------------------------------------------------------------
// Joint setup
// 0 = shoulder_extension
// 1 = elbow_extension
// -----------------------------------------------------------------------------
static const int NJ = 2;

// Joint limits in radians, matching your URDF
static const double RAD_MIN[NJ] = {-1.57, -1.57};
static const double RAD_MAX[NJ] = { 1.57,  1.57};

// Startup estimated position
static const double RAD_HOME[NJ] = {0.0, 0.0};

// If one actuator moves the wrong way, flip its sign to -1.0
static const double DIRECTION_SIGN[NJ] = {1.0, 1.0};

// Open-loop speed estimate in rad/s at the chosen PWM below
// Tune these after testing
static const double ACTUATOR_SPEED_RAD_PER_SEC[NJ] = {0.35, 0.35};

// Stop once estimated position is within this error band
static const double POSITION_DEADBAND_RAD = 0.03;

// If no new command arrives for this long, stop motors for safety
static const unsigned long COMMAND_TIMEOUT_MS = 1000;

// -----------------------------------------------------------------------------
// BTS7960 pin setup
// Shoulder actuator
// -----------------------------------------------------------------------------
const int SHOULDER_RPWM_PIN = 22;
const int SHOULDER_LPWM_PIN = 23;

// Elbow actuator
const int ELBOW_RPWM_PIN = 18;
const int ELBOW_LPWM_PIN = 19;

// PWM setup
const int PWM_FREQ = 1000;
const int PWM_RESOLUTION = 8;
const int DRIVE_PWM = 180;

// -----------------------------------------------------------------------------
// micro-ROS objects
// -----------------------------------------------------------------------------
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_subscription_t sub;
rcl_publisher_t pub;
rcl_timer_t timer;
rclc_executor_t executor;

std_msgs__msg__Float64MultiArray sub_msg;
std_msgs__msg__Float64MultiArray pub_msg;

// Backing storage for variable-length arrays
static double sub_data_buf[NJ];
static double pub_data_buf[NJ];

// Commanded targets and estimated states, in radians
static double target_rad[NJ] = {0.0, 0.0};
static double est_rad[NJ] = {0.0, 0.0};

// Timing
static unsigned long last_update_ms = 0;
static unsigned long last_command_ms = 0;

#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) error_loop(); }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

// -----------------------------------------------------------------------------
// Utility
// -----------------------------------------------------------------------------
void error_loop()
{
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

double clampf(double x, double lo, double hi)
{
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// -----------------------------------------------------------------------------
// Motor helpers
// -----------------------------------------------------------------------------
void stopMotorPins(int rpwm_pin, int lpwm_pin)
{
  ledcWrite(rpwm_pin, 0);
  ledcWrite(lpwm_pin, 0);
}

void moveForwardPins(int rpwm_pin, int lpwm_pin, int speed_val)
{
  ledcWrite(rpwm_pin, speed_val);
  ledcWrite(lpwm_pin, 0);
}

void moveReversePins(int rpwm_pin, int lpwm_pin, int speed_val)
{
  ledcWrite(rpwm_pin, 0);
  ledcWrite(lpwm_pin, speed_val);
}

void stopJoint(int joint_idx)
{
  if (joint_idx == 0) {
    stopMotorPins(SHOULDER_RPWM_PIN, SHOULDER_LPWM_PIN);
  } else {
    stopMotorPins(ELBOW_RPWM_PIN, ELBOW_LPWM_PIN);
  }
}

void driveJointForward(int joint_idx)
{
  if (joint_idx == 0) {
    moveForwardPins(SHOULDER_RPWM_PIN, SHOULDER_LPWM_PIN, DRIVE_PWM);
  } else {
    moveForwardPins(ELBOW_RPWM_PIN, ELBOW_LPWM_PIN, DRIVE_PWM);
  }
}

void driveJointReverse(int joint_idx)
{
  if (joint_idx == 0) {
    moveReversePins(SHOULDER_RPWM_PIN, SHOULDER_LPWM_PIN, DRIVE_PWM);
  } else {
    moveReversePins(ELBOW_RPWM_PIN, ELBOW_LPWM_PIN, DRIVE_PWM);
  }
}

void stopAllMotors()
{
  stopJoint(0);
  stopJoint(1);
}

// -----------------------------------------------------------------------------
// Open-loop controller
// -----------------------------------------------------------------------------
void updateJointOpenLoop(int joint_idx, double dt_sec)
{
  double error = target_rad[joint_idx] - est_rad[joint_idx];

  if (fabs(error) <= POSITION_DEADBAND_RAD) {
    stopJoint(joint_idx);
    return;
  }

  // Convert desired motion sign into actuator drive direction
  double signed_error = error * DIRECTION_SIGN[joint_idx];

  if (signed_error > 0.0) {
    driveJointForward(joint_idx);
    est_rad[joint_idx] += ACTUATOR_SPEED_RAD_PER_SEC[joint_idx] * dt_sec;
  } else {
    driveJointReverse(joint_idx);
    est_rad[joint_idx] -= ACTUATOR_SPEED_RAD_PER_SEC[joint_idx] * dt_sec;
  }

  est_rad[joint_idx] = clampf(est_rad[joint_idx], RAD_MIN[joint_idx], RAD_MAX[joint_idx]);
}

void updateController()
{
  unsigned long now_ms = millis();

  if (last_update_ms == 0) {
    last_update_ms = now_ms;
    return;
  }

  double dt_sec = (now_ms - last_update_ms) / 1000.0;
  last_update_ms = now_ms;

  // If commands stop arriving, stop everything
  if ((now_ms - last_command_ms) > COMMAND_TIMEOUT_MS) {
    stopAllMotors();
    return;
  }

  updateJointOpenLoop(0, dt_sec);
  updateJointOpenLoop(1, dt_sec);
}

// -----------------------------------------------------------------------------
// micro-ROS subscription callback
// Expects [shoulder_extension, elbow_extension] in radians
// -----------------------------------------------------------------------------
void sub_callback(const void * msgin)
{
  const std_msgs__msg__Float64MultiArray * m =
    (const std_msgs__msg__Float64MultiArray *)msgin;

  if (m->data.size < NJ) {
    return;
  }

  target_rad[0] = clampf(m->data.data[0], RAD_MIN[0], RAD_MAX[0]);
  target_rad[1] = clampf(m->data.data[1], RAD_MIN[1], RAD_MAX[1]);

  last_command_ms = millis();

  digitalWrite(LED_PIN, !digitalRead(LED_PIN));

  Serial.print("Received targets: shoulder=");
  Serial.print(target_rad[0], 4);
  Serial.print("  elbow=");
  Serial.println(target_rad[1], 4);
}

// -----------------------------------------------------------------------------
// Publish estimated joint states back to ROS2
// -----------------------------------------------------------------------------
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void)last_call_time;

  if (timer == NULL) {
    return;
  }

  pub_data_buf[0] = est_rad[0];
  pub_data_buf[1] = est_rad[1];

  pub_msg.data.size = NJ;
  RCSOFTCHECK(rcl_publish(&pub, &pub_msg, NULL));
}

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------
void setup()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  Serial.begin(115200);
  delay(2000);

  // Attach PWM outputs for both BTS7960 modules
  if (!ledcAttach(SHOULDER_RPWM_PIN, PWM_FREQ, PWM_RESOLUTION)) {
    Serial.println("Failed to attach SHOULDER_RPWM_PIN");
    while (true) {}
  }

  if (!ledcAttach(SHOULDER_LPWM_PIN, PWM_FREQ, PWM_RESOLUTION)) {
    Serial.println("Failed to attach SHOULDER_LPWM_PIN");
    while (true) {}
  }

  if (!ledcAttach(ELBOW_RPWM_PIN, PWM_FREQ, PWM_RESOLUTION)) {
    Serial.println("Failed to attach ELBOW_RPWM_PIN");
    while (true) {}
  }

  if (!ledcAttach(ELBOW_LPWM_PIN, PWM_FREQ, PWM_RESOLUTION)) {
    Serial.println("Failed to attach ELBOW_LPWM_PIN");
    while (true) {}
  }

  stopAllMotors();

  // Initialize estimated state
  for (int i = 0; i < NJ; i++) {
    est_rad[i] = RAD_HOME[i];
    target_rad[i] = RAD_HOME[i];
  }

  last_update_ms = millis();
  last_command_ms = millis();

  set_microros_transports();

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "arm_2026_esp32", "", &support));

  // Subscriber message setup
  sub_msg.data.data = sub_data_buf;
  sub_msg.data.size = 0;
  sub_msg.data.capacity = NJ;

  // Publisher message setup
  pub_msg.data.data = pub_data_buf;
  pub_msg.data.size = NJ;
  pub_msg.data.capacity = NJ;

  // Subscribe to desired actuator targets from the ROS2 hardware plugin
  RCCHECK(rclc_subscription_init_default(
    &sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    "/arm_actuator_targets"
  ));

  // Publish estimated actuator states back to the ROS2 hardware plugin
  RCCHECK(rclc_publisher_init_default(
    &pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    "/arm_actuator_states"
  ));

  // Publish state at 50 Hz
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(20),
    timer_callback
  ));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub, &sub_msg, &sub_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  Serial.println("micro-ROS actuator bridge ready");
}

// -----------------------------------------------------------------------------
// Main loop
// -----------------------------------------------------------------------------
void loop()
{
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  updateController();
  delay(10);
}
