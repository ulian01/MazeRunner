#include <Arduino.h>

// --- Line Sensor Setup ---
const int LINE_SENSOR_COUNT = 8;
const int sensorPins[LINE_SENSOR_COUNT] = {
  A7, A6, A5, A4, A3, A2, A1, A0
};

// Tune this depending on your sensors & surface:
// For many IR line sensors: black line = LOWER value.
const int LINE_THRESHOLD = 500;  // try 400–700 if needed

// --- Motor Speed Constants ---
const int BASE_SPEED = 150;   // forward speed
const int MAX_SPEED  = 255;   // max PWM
const int TURN_GAIN  = 35;    // how strong to turn

// --- Motor Pins ---
#define LEFT_BWD 9
#define LEFT_FWD 6
#define RIGHT_BWD 5
#define RIGHT_FWD 3

// --- Ultrasonic (optional, not used in line follow now) ---
#define ECHO 12
#define TRIG 4

// --- Prototypes ---
void stopMotors();
void applyWheelSpeeds(int leftSpeed, int rightSpeed);
float readDistance();
bool sensorOnLine(int index);

// Remember last side where line was seen
float lastError = 0;

void setup() {
  Serial.begin(9600);

  // Motor pins
  pinMode(LEFT_FWD, OUTPUT);
  pinMode(RIGHT_FWD, OUTPUT);
  pinMode(LEFT_BWD, OUTPUT);
  pinMode(RIGHT_BWD, OUTPUT);

  // Ultrasonic
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  // Sensor pins (for analogRead, pinMode isn't mandatory, but it's fine)
  for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  stopMotors();
  delay(1000); // small pause at start
}

void loop() {
  // 1. Read all sensors and compute how far the line is from the center
  int numOnLine = 0;
  float weightedSum = 0;

  float centerIndex = (LINE_SENSOR_COUNT - 1) / 2.0; // e.g. 3.5 for 0..7

  for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
    bool onLine = sensorOnLine(i);

    if (onLine) {
      numOnLine++;
      float pos = i - centerIndex;  // negative = left, positive = right
      weightedSum += pos;
    }

    // Debug print:
    // Serial.print(onLine ? "1" : "0");
  }
  // Serial.println();

  // 2. Decide what to do with motors
  if (numOnLine > 0) {
    // We see the line somewhere
    float error = weightedSum / numOnLine; // average offset
    lastError = error;

    // Convert error into speed difference
    int correction = (int)(TURN_GAIN * error);

    int leftSpeed  = BASE_SPEED - correction;
    int rightSpeed = BASE_SPEED + correction;

    leftSpeed  = constrain(leftSpeed, 0, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, 0, MAX_SPEED);

    applyWheelSpeeds(leftSpeed, rightSpeed);
  } else {
    // 3. Line lost → search using last direction
    int searchSpeed = BASE_SPEED;

    if (lastError < 0) {
      // Line was on the left → turn left to find it
      applyWheelSpeeds(0, searchSpeed);
    } else {
      // Line was on the right → turn right to find it
      applyWheelSpeeds(searchSpeed, 0);
    }
  }

  delay(10); // smooth motion
}

// --- Sensors ---

bool sensorOnLine(int index) {
  int raw = analogRead(sensorPins[index]);
  // For most sensors: black = lower value, white = higher.
  // If yours behaves opposite, flip the comparison:
  //   return raw > LINE_THRESHOLD;
  bool onLine = (raw < LINE_THRESHOLD);

  // Optional: debug each sensor’s raw value
  // Serial.print("S"); Serial.print(index);
  // Serial.print("="); Serial.println(raw);

  return onLine;
}

// --- Motors ---

void applyWheelSpeeds(int leftSpeed, int rightSpeed) {
  leftSpeed  = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  analogWrite(LEFT_FWD, leftSpeed);
  analogWrite(RIGHT_FWD, rightSpeed);
  analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_BWD, 0);
}

void stopMotors() {
  analogWrite(LEFT_FWD, 0);
  analogWrite(RIGHT_FWD, 0);
  analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_BWD, 0);
}

// --- Ultrasonic (optional) ---

float readDistance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long duration = pulseIn(ECHO, HIGH, 30000); // 30 ms timeout

  if (duration == 0) {
    return 999.0;
  }

  return duration * 0.034 / 2.0;
}
