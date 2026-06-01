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
const int PWM_VALUE = 255;

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

void stopMotor(int rpwm_pin, int lpwm_pin)
{
  ledcWrite(rpwm_pin, 0);
  ledcWrite(lpwm_pin, 0);
}

void driveExtend(int rpwm_pin, int lpwm_pin)
{
  ledcWrite(rpwm_pin, PWM_VALUE);
  ledcWrite(lpwm_pin, 0);
}

void driveRetract(int rpwm_pin, int lpwm_pin)
{
  ledcWrite(rpwm_pin, 0);
  ledcWrite(lpwm_pin, PWM_VALUE);
}

void applyMotorCommand(uint8_t cmd, int rpwm_pin, int lpwm_pin)
{
  switch (cmd)
  {
    case CMD_EXTEND:
      driveExtend(rpwm_pin, lpwm_pin);
      break;

    case CMD_RETRACT:
      driveRetract(rpwm_pin, lpwm_pin);
      break;

    case CMD_STOP:
    default:
      stopMotor(rpwm_pin, lpwm_pin);
      break;
  }
}

void stopAllMotors()
{
  stopMotor(SHOULDER_RPWM, SHOULDER_LPWM);
  stopMotor(ELBOW_RPWM, ELBOW_LPWM);
}

void applyCommands(uint8_t shoulder_cmd, uint8_t elbow_cmd)
{
  applyMotorCommand(shoulder_cmd, SHOULDER_RPWM, SHOULDER_LPWM);
  applyMotorCommand(elbow_cmd, ELBOW_RPWM, ELBOW_LPWM);
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
        applyCommands(rx_shoulder, rx_elbow);
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
    stopAllMotors();
    last_cmd_ms = millis();
  }

  delay(5);
}
