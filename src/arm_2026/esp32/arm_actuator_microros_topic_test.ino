#include <micro_ros_arduino.h>

#include <Arduino.h>
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float64_multi_array.h>

rcl_subscription_t cmd_subscriber;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
std_msgs__msg__Float64MultiArray cmd_msg;

#define LED_PIN 2

#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) {} }

// ===== BTS7960 pin map =====
// Motor A: LPWM=D23, RPWM=D22
constexpr int SHOULDER_RPWM_PIN = 22;
constexpr int SHOULDER_LPWM_PIN = 23;
// Motor B: LPWM=D19, RPWM=D18
constexpr int ELBOW_RPWM_PIN = 18;
constexpr int ELBOW_LPWM_PIN = 19;

// EN pins are hardwired to VCC in your setup, so no EN GPIO handling here.

// ===== PWM config =====
constexpr int PWM_FREQ_HZ = 20000;
constexpr int PWM_BITS = 8;
constexpr int PWM_MAX = (1 << PWM_BITS) - 1;  // 255
constexpr int PWM_CH_SH_R = 0;
constexpr int PWM_CH_SH_L = 1;
constexpr int PWM_CH_EL_R = 2;
constexpr int PWM_CH_EL_L = 3;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
#define USE_ESP32_LEDC_NEW_API 1
#else
#define USE_ESP32_LEDC_NEW_API 0
#endif

// Topic: /arm_actuator_test_cmd
// data[0] = shoulder command in [-1.0, 1.0]
// data[1] = elbow command in [-1.0, 1.0]
constexpr int MAX_TEST_DUTY = 220;
constexpr uint32_t CMD_TIMEOUT_MS = 500;
constexpr bool ENABLE_SERIAL_DEBUG = true;

double cmd_data_buf[2];
double shoulder_cmd_norm = 0.0;
double elbow_cmd_norm = 0.0;
uint32_t last_cmd_ms = 0;
bool got_cmd = false;
uint32_t last_dbg_ms = 0;

double clampd(double x, double lo, double hi)
{
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

void write_pwm(int channel, int duty)
{
#if USE_ESP32_LEDC_NEW_API
  ledcWriteChannel(channel, duty);
#else
  ledcWrite(channel, duty);
#endif
}

void set_motor_signed(int ch_r, int ch_l, int signed_duty)
{
  int duty = abs(signed_duty);
  if (duty > MAX_TEST_DUTY) duty = MAX_TEST_DUTY;

  if (signed_duty > 0) {
    write_pwm(ch_r, duty);
    write_pwm(ch_l, 0);
  } else if (signed_duty < 0) {
    write_pwm(ch_r, 0);
    write_pwm(ch_l, duty);
  } else {
    write_pwm(ch_r, 0);
    write_pwm(ch_l, 0);
  }
}

void stop_all()
{
  set_motor_signed(PWM_CH_SH_R, PWM_CH_SH_L, 0);
  set_motor_signed(PWM_CH_EL_R, PWM_CH_EL_L, 0);
}

void error_loop()
{
  stop_all();
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(120);
  }
}

void cmd_callback(const void * msg_in)
{
  const std_msgs__msg__Float64MultiArray * m =
    (const std_msgs__msg__Float64MultiArray *)msg_in;
  if (m == nullptr || m->data.size < 2) {
    return;
  }

  shoulder_cmd_norm = clampd(m->data.data[0], -1.0, 1.0);
  elbow_cmd_norm = clampd(m->data.data[1], -1.0, 1.0);
  got_cmd = true;
  last_cmd_ms = millis();
}

void setup_pwm()
{
#if USE_ESP32_LEDC_NEW_API
  const bool ok1 = ledcAttachChannel(SHOULDER_RPWM_PIN, PWM_FREQ_HZ, PWM_BITS, PWM_CH_SH_R);
  const bool ok2 = ledcAttachChannel(SHOULDER_LPWM_PIN, PWM_FREQ_HZ, PWM_BITS, PWM_CH_SH_L);
  const bool ok3 = ledcAttachChannel(ELBOW_RPWM_PIN, PWM_FREQ_HZ, PWM_BITS, PWM_CH_EL_R);
  const bool ok4 = ledcAttachChannel(ELBOW_LPWM_PIN, PWM_FREQ_HZ, PWM_BITS, PWM_CH_EL_L);
  if (!(ok1 && ok2 && ok3 && ok4)) {
    if (ENABLE_SERIAL_DEBUG) {
      Serial.println("[topic_test] ledcAttachChannel failed");
    }
    error_loop();
  }
#else
  ledcSetup(PWM_CH_SH_R, PWM_FREQ_HZ, PWM_BITS);
  ledcSetup(PWM_CH_SH_L, PWM_FREQ_HZ, PWM_BITS);
  ledcSetup(PWM_CH_EL_R, PWM_FREQ_HZ, PWM_BITS);
  ledcSetup(PWM_CH_EL_L, PWM_FREQ_HZ, PWM_BITS);
  ledcAttachPin(SHOULDER_RPWM_PIN, PWM_CH_SH_R);
  ledcAttachPin(SHOULDER_LPWM_PIN, PWM_CH_SH_L);
  ledcAttachPin(ELBOW_RPWM_PIN, PWM_CH_EL_R);
  ledcAttachPin(ELBOW_LPWM_PIN, PWM_CH_EL_L);
#endif
  stop_all();
}

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  if (ENABLE_SERIAL_DEBUG) {
    Serial.begin(115200);
    delay(200);
    Serial.println("[topic_test] boot");
  }

  setup_pwm();

  set_microros_transports();
  delay(1500);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "arm_2026_topic_test", "", &support));

  std_msgs__msg__Float64MultiArray__init(&cmd_msg);
  cmd_msg.data.data = cmd_data_buf;
  cmd_msg.data.size = 0;
  cmd_msg.data.capacity = 2;
  cmd_msg.layout.data_offset = 0;
  cmd_msg.layout.dim.data = NULL;
  cmd_msg.layout.dim.size = 0;
  cmd_msg.layout.dim.capacity = 0;

  RCCHECK(rclc_subscription_init_default(
    &cmd_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    "/arm_actuator_test_cmd"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &cmd_subscriber,
    &cmd_msg,
    &cmd_callback,
    ON_NEW_DATA));

  if (ENABLE_SERIAL_DEBUG) {
    Serial.println("[topic_test] ready: publish /arm_actuator_test_cmd [shoulder, elbow] in [-1,1]");
  }
}

void loop()
{
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));

  const uint32_t now = millis();
  bool fresh = got_cmd && ((now - last_cmd_ms) <= CMD_TIMEOUT_MS);

  if (!fresh) {
    stop_all();
  } else {
    int sh = (int)(shoulder_cmd_norm * (double)MAX_TEST_DUTY);
    int el = (int)(elbow_cmd_norm * (double)MAX_TEST_DUTY);
    set_motor_signed(PWM_CH_SH_R, PWM_CH_SH_L, sh);
    set_motor_signed(PWM_CH_EL_R, PWM_CH_EL_L, el);
  }

  if (ENABLE_SERIAL_DEBUG && (now - last_dbg_ms) > 250) {
    last_dbg_ms = now;
    Serial.print("[topic_test] fresh=");
    Serial.print(fresh ? "1" : "0");
    Serial.print(" cmd=(");
    Serial.print(shoulder_cmd_norm, 2);
    Serial.print(", ");
    Serial.print(elbow_cmd_norm, 2);
    Serial.println(")");
  }

  delay(5);
}
