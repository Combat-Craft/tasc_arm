#include <ESP32Servo.h>
#include <map>

Servo myservo;
static const int servoPin = 13;

// Timing (in ms)
const int dotTime = 150;
const int dashTime = 400;
const int symbolGap = 150;
const int letterGap = 400;
const int wordGap = 800;

// character -> morse code, just like the Python dict but inverted
std::map<char, String> morseMap = {
  {'A', ".-"},    {'B', "-..."},  {'C', "-.-."},  {'D', "-.."},
  {'E', "."},     {'F', "..-."},  {'G', "--."},   {'H', "...."},
  {'I', ".."},    {'J', ".---"},  {'K', "-.-"},   {'L', ".-.."},
  {'M', "--"},    {'N', "-."},    {'O', "---"},   {'P', ".--."},
  {'Q', "--.-"},  {'R', ".-."},   {'S', "..."},   {'T', "-"},
  {'U', "..-"},   {'V', "...-"},  {'W', ".--"},   {'X', "-..-"},
  {'Y', "-.--"},  {'Z', "--.."},

  {'0', "-----"}, {'1', ".----"}, {'2', "..---"}, {'3', "...--"},
  {'4', "....-"}, {'5', "....."}, {'6', "-...."}, {'7', "--..."},
  {'8', "---.."}, {'9', "----."},

  {'.', ".-.-.-"}, {',', "--..--"}, {'?', "..--.."},
  {'\'', ".----."}, {'!', "-.-.--"}, {'/', "-..-."},
  {'(', "-.--."},  {')', "-.--.-"},
  {'&', ".-..."},  {':', "---..."},
  {';', "-.-.-."}, {'=', "-...-"},
  {'+', ".-.-."},  {'-', "-....-"},
  {'_', "..--.-"}, {'"', ".-..-."},
  {'@', ".--.-."}
};

void setup() {
  Serial.begin(115200);
  myservo.setPeriodHertz(50);
  myservo.attach(servoPin, 500, 2400);
  myservo.write(0);
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.toUpperCase();

    for (int i = 0; i < input.length(); i++) {
      char c = input[i];

      if (c == ' ') {
        delay(wordGap);
        continue;
      }

      // dict-style lookup, just like Python's `if c in morseMap`
      if (morseMap.count(c)) {
        playMorse(morseMap[c]);
        delay(letterGap);
      }
      // unknown characters are silently skipped
    }
  }
}

void playMorse(const String &morse) {
  for (int i = 0; i < morse.length(); i++) {
    char symbol = morse[i];

    if (symbol == '.') {
      dotPulse();
    } else if (symbol == '-') {
      dashPulse();
    }

    delay(symbolGap);
  }
}

void dotPulse() {
  myservo.write(45);
  delay(dotTime);
  myservo.write(0);
  delay(100);
}

void dashPulse() {
  myservo.write(45);
  delay(dashTime);
  myservo.write(0);
  delay(100);
}
