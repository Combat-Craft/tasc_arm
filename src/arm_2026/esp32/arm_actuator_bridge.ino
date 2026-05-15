#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>

#define LED_PIN 13

// We have 2 actuator channels:
// [0] shoulder
// [1] elbow
static const int NJ = 2;

// BTS7960 pins
static const int SHOULDER_RPWM = 18;
static const int SHOULDER_LPWM = 19;

static const int ELBOW_RPWM = 23;
static const int ELBOW_LPWM = 22;

// PWM settings
static const int PWM_FREQ = 1000;
static const int PWM_RESOLUTION = 8;   // 0..255
static const int PWM_MAX_CMD = 180;    // cap speed a bit for safety
static const float DEADZONE = 0.10f;

// Stop actuators if teleop/agent stops sending fresh commands.
static const unsigned long CMD_TIMEOUT_MS = 300;

// micro-ROS objects
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_subscription_t sub;
rclc_executor_t executor;

std_msgs__msg__Float32MultiArray msg;

// Backing array for msg.data
static float msg_data_buf[NJ];

// Latest command values
static float actuator_cmd[NJ] = {0.0f, 0.0f};
static unsigned long last_cmd_ms = 0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if (temp_rc != RCL_RET_OK) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if (temp_rc != RCL_RET_OK) {} }

static void error_loop()
{
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

static float clampf(float x, float lo, float hi)
{
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static int cmdToPwm(float cmd)
{
  cmd = fabs(cmd);
  if (cmd < DEADZONE) return 0;

  int pwm = (int)(cmd * PWM_MAX_CMD);
  if (pwm < 80) pwm = 80;  // small minimum to overcome stiction
  if (pwm > 255) pwm = 255;
  return pwm;
}

static void stopMotor(int rpwm_pin, int lpwm_pin)
{
  ledcWrite(rpwm_pin, 0);
  ledcWrite(lpwm_pin, 0);
}

static void moveForward(int rpwm_pin, int lpwm_pin, int pwm_val)
{
  ledcWrite(rpwm_pin, pwm_val);
  ledcWrite(lpwm_pin, 0);
}

static void moveReverse(int rpwm_pin, int lpwm_pin, int pwm_val)
{
  ledcWrite(rpwm_pin, 0);
  ledcWrite(lpwm_pin, pwm_val);
}

static void driveMotorFromCmd(float cmd, int rpwm_pin, int lpwm_pin)
{
  cmd = clampf(cmd, -1.0f, 1.0f);

  int pwm_val = cmdToPwm(cmd);

  if (fabs(cmd) < DEADZONE) {
    stopMotor(rpwm_pin, lpwm_pin);
    return;
  }

  if (cmd > 0.0f) {
    moveForward(rpwm_pin, lpwm_pin, pwm_val);
  } else {
    moveReverse(rpwm_pin, lpwm_pin, pwm_val);
  }
}

static void applyCommand()
{
  driveMotorFromCmd(actuator_cmd[0], SHOULDER_RPWM, SHOULDER_LPWM);
  driveMotorFromCmd(actuator_cmd[1], ELBOW_RPWM, ELBOW_LPWM);
}

static void stopAllMotors()
{
  actuator_cmd[0] = 0.0f;
  actuator_cmd[1] = 0.0f;
  stopMotor(SHOULDER_RPWM, SHOULDER_LPWM);
  stopMotor(ELBOW_RPWM, ELBOW_LPWM);
}

// micro-ROS subscription callback
static void sub_callback(const void * msgin)
{
  const std_msgs__msg__Float32MultiArray * m =
    (const std_msgs__msg__Float32MultiArray *)msgin;

  // Expect at least 2 values: shoulder, elbow
  if (m->data.size < (size_t)NJ) {
    return;
  }

  for (int i = 0; i < NJ; i++) {
    actuator_cmd[i] = clampf(m->data.data[i], -1.0f, 1.0f);
  }

  last_cmd_ms = millis();
  applyCommand();

  digitalWrite(LED_PIN, !digitalRead(LED_PIN));

  Serial.print("Shoulder cmd: ");
  Serial.print(actuator_cmd[0], 3);
  Serial.print(" | Elbow cmd: ");
  Serial.println(actuator_cmd[1], 3);
}

void setup()
{
  // Use the same transport style as your old working code
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  Serial.begin(115200);
  delay(1000);

  // New ESP32 core 3.x style
  if (!ledcAttach(SHOULDER_RPWM, PWM_FREQ, PWM_RESOLUTION)) {
    Serial.println("Failed to attach SHOULDER_RPWM");
    while (true) {}
  }

  if (!ledcAttach(SHOULDER_LPWM, PWM_FREQ, PWM_RESOLUTION)) {
    Serial.println("Failed to attach SHOULDER_LPWM");
    while (true) {}
  }

  if (!ledcAttach(ELBOW_RPWM, PWM_FREQ, PWM_RESOLUTION)) {
    Serial.println("Failed to attach ELBOW_RPWM");
    while (true) {}
  }

  if (!ledcAttach(ELBOW_LPWM, PWM_FREQ, PWM_RESOLUTION)) {
    Serial.println("Failed to attach ELBOW_LPWM");
    while (true) {}
  }

  stopAllMotors();

  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "arm_2026_actuator_bridge", "", &support));

  // Prepare msg memory to avoid allocations
  msg.data.data = msg_data_buf;
  msg.data.size = 0;
  msg.data.capacity = NJ;

  // Subscriber: shoulder/elbow direct actuator commands from keyboard teleop.
  RCCHECK(rclc_subscription_init_default(
    &sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/arm_actuator_drive_cmd"
  ));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub, &msg, &sub_callback, ON_NEW_DATA));

  last_cmd_ms = millis();

  Serial.println("ESP32 actuator micro-ROS node ready.");
  Serial.println("Subscribed to /arm_actuator_drive_cmd");
}

void loop()
{
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

  if ((millis() - last_cmd_ms) > CMD_TIMEOUT_MS) {
    stopAllMotors();
    last_cmd_ms = millis();
  }

  delay(10);
}
