const int LED_PIN = 13;

// Shoulder BTS7960 pins
const int SHOULDER_RPWM = 18;
const int SHOULDER_LPWM = 19;

// Elbow BTS7960 pins
const int ELBOW_RPWM = 23;
const int ELBOW_LPWM = 22;

// PWM settings
const int PWM_FREQ = 1000;
const int PWM_RESOLUTION = 8;

// Ramp settings
const int PWM_START = 0;          // starting PWM when motion begins
const int PWM_TARGET = 255;         // max PWM
const int PWM_RAMP_STEP = 4;        // how much PWM increases each step
const unsigned long PWM_RAMP_INTERVAL_MS = 20;

// Protocol
const uint8_t PACKET_HEADER = 0xAA;

const uint8_t CMD_STOP = 0;
const uint8_t CMD_EXTEND = 1;
const uint8_t CMD_RETRACT = 2;

// Timeout safety
const unsigned long CMD_TIMEOUT_MS = 300;
unsigned long last_cmd_ms = 0;

enum ParseState
{
  WAIT_HEADER,
  WAIT_SHOULDER,
  WAIT_ELBOW,
  WAIT_CHECKSUM
};

ParseState parse_state = WAIT_HEADER;
uint8_t rx_shoulder = CMD_STOP;
uint8_t rx_elbow = CMD_STOP;

// Desired commands from serial
uint8_t shoulder_target_cmd = CMD_STOP;
uint8_t elbow_target_cmd = CMD_STOP;

// Current applied commands
uint8_t shoulder_active_cmd = CMD_STOP;
uint8_t elbow_active_cmd = CMD_STOP;

// Per-motor PWM state
int shoulder_pwm = 0;
int elbow_pwm = 0;

unsigned long last_shoulder_ramp_ms = 0;
unsigned long last_elbow_ramp_ms = 0;

void stopMotor(int rpwm_pin, int lpwm_pin)
{
  ledcWrite(rpwm_pin, 0);
  ledcWrite(lpwm_pin, 0);
}

void driveMotor(int cmd, int pwm_val, int rpwm_pin, int lpwm_pin)
{
  if (cmd == CMD_EXTEND)
  {
    ledcWrite(rpwm_pin, pwm_val);
    ledcWrite(lpwm_pin, 0);
  }
  else if (cmd == CMD_RETRACT)
  {
    ledcWrite(rpwm_pin, 0);
    ledcWrite(lpwm_pin, pwm_val);
  }
  else
  {
    stopMotor(rpwm_pin, lpwm_pin);
  }
}

void stopAllMotors()
{
  shoulder_target_cmd = CMD_STOP;
  elbow_target_cmd = CMD_STOP;
  shoulder_active_cmd = CMD_STOP;
  elbow_active_cmd = CMD_STOP;
  shoulder_pwm = 0;
  elbow_pwm = 0;
  stopMotor(SHOULDER_RPWM, SHOULDER_LPWM);
  stopMotor(ELBOW_RPWM, ELBOW_LPWM);
}

void updateMotorRamp(
  uint8_t target_cmd,
  uint8_t & active_cmd,
  int & current_pwm,
  unsigned long & last_ramp_ms,
  int rpwm_pin,
  int lpwm_pin)
{
  unsigned long now = millis();

  if (target_cmd == CMD_STOP)
  {
    active_cmd = CMD_STOP;
    if (current_pwm > 0 && (now - last_ramp_ms) >= PWM_RAMP_INTERVAL_MS)
    {
      current_pwm -= PWM_RAMP_STEP;
      if (current_pwm < 0) {
        current_pwm = 0;
      }
      last_ramp_ms = now;
    }

    if (current_pwm == 0)
    {
      stopMotor(rpwm_pin, lpwm_pin);
    }
    else
    {
      driveMotor(active_cmd, current_pwm, rpwm_pin, lpwm_pin);
    }
    return;
  }

  // If direction changes, force a stop before reversing
  if (active_cmd != CMD_STOP && active_cmd != target_cmd)
  {
    if ((now - last_ramp_ms) >= PWM_RAMP_INTERVAL_MS)
    {
      current_pwm -= PWM_RAMP_STEP;
      if (current_pwm < 0) {
        current_pwm = 0;
      }
      last_ramp_ms = now;
    }

    if (current_pwm == 0)
    {
      active_cmd = CMD_STOP;
      stopMotor(rpwm_pin, lpwm_pin);
    }
    else
    {
      driveMotor(active_cmd, current_pwm, rpwm_pin, lpwm_pin);
    }
    return;
  }

  // Start motion
  if (active_cmd == CMD_STOP)
  {
    active_cmd = target_cmd;
    if (current_pwm < PWM_START)
    {
      current_pwm = PWM_START;
    }
    driveMotor(active_cmd, current_pwm, rpwm_pin, lpwm_pin);
    last_ramp_ms = now;
    return;
  }

  // Continue ramping toward target
  if ((now - last_ramp_ms) >= PWM_RAMP_INTERVAL_MS)
  {
    current_pwm += PWM_RAMP_STEP;
    if (current_pwm > PWM_TARGET) {
      current_pwm = PWM_TARGET;
    }
    last_ramp_ms = now;
  }

  driveMotor(active_cmd, current_pwm, rpwm_pin, lpwm_pin);
}

void handleIncomingByte(uint8_t b)
{
  switch (parse_state)
  {
    case WAIT_HEADER:
      if (b == PACKET_HEADER)
      {
        parse_state = WAIT_SHOULDER;
      }
      break;

    case WAIT_SHOULDER:
      rx_shoulder = b;
      parse_state = WAIT_ELBOW;
      break;

    case WAIT_ELBOW:
      rx_elbow = b;
      parse_state = WAIT_CHECKSUM;
      break;

    case WAIT_CHECKSUM:
    {
      uint8_t expected_checksum = rx_shoulder ^ rx_elbow;

      if (b == expected_checksum &&
          rx_shoulder <= CMD_RETRACT &&
          rx_elbow <= CMD_RETRACT)
      {
        shoulder_target_cmd = rx_shoulder;
        elbow_target_cmd = rx_elbow;
        last_cmd_ms = millis();
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      }

      parse_state = WAIT_HEADER;
      break;
    }

    default:
      parse_state = WAIT_HEADER;
      break;
  }
}

void setup()
{
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  if (!ledcAttach(SHOULDER_RPWM, PWM_FREQ, PWM_RESOLUTION)) {
    while (true) {}
  }

  if (!ledcAttach(SHOULDER_LPWM, PWM_FREQ, PWM_RESOLUTION)) {
    while (true) {}
  }

  if (!ledcAttach(ELBOW_RPWM, PWM_FREQ, PWM_RESOLUTION)) {
    while (true) {}
  }

  if (!ledcAttach(ELBOW_LPWM, PWM_FREQ, PWM_RESOLUTION)) {
    while (true) {}
  }

  stopAllMotors();
  last_cmd_ms = millis();
}

void loop()
{
  while (Serial.available() > 0)
  {
    uint8_t b = static_cast<uint8_t>(Serial.read());
    handleIncomingByte(b);
  }

  if ((millis() - last_cmd_ms) > CMD_TIMEOUT_MS)
  {
    shoulder_target_cmd = CMD_STOP;
    elbow_target_cmd = CMD_STOP;
  }

  updateMotorRamp(
    shoulder_target_cmd,
    shoulder_active_cmd,
    shoulder_pwm,
    last_shoulder_ramp_ms,
    SHOULDER_RPWM,
    SHOULDER_LPWM);

  updateMotorRamp(
    elbow_target_cmd,
    elbow_active_cmd,
    elbow_pwm,
    last_elbow_ramp_ms,
    ELBOW_RPWM,
    ELBOW_LPWM);

  delay(5);
}
