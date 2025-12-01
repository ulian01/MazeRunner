#include <Arduino.h>

// --- Line Sensor Setup ---
const int LINE_SENSOR_COUNT = 8;

// LEFT → RIGHT physical order
const int sensorPins[LINE_SENSOR_COUNT] = {
  A7, A6, A5, A4, A3, A2, A1, A0
};

// Tune this after reading raw values
const int LINE_THRESHOLD = 500;

// --- Motor Speed Constants ---
const int BASE_SPEED    = 135;  // forward speed (normal curves)
const int MAX_SPEED     = 255;
const int TURN_GAIN     = 45;   // for normal steering
const int PIVOT_SPEED   = 250;  // speed when pivoting on sharp corners

// --- Motor Pins ---
#define LEFT_BWD 9
#define LEFT_FWD 6
#define RIGHT_BWD 5
#define RIGHT_FWD 3

// --- Ultrasonic (optional) ---
#define ECHO 12
#define TRIG 4

// --- Prototypes ---
void stopMotors();
void applyWheelSpeeds(int leftSpeed, int rightSpeed);
void pivotLeft();
void pivotRight();
float readDistance();
bool sensorOnLine(int index);

float lastError = 0;

void setup() {
  Serial.begin(9600);

  pinMode(LEFT_FWD, OUTPUT);
  pinMode(RIGHT_FWD, OUTPUT);
  pinMode(LEFT_BWD, OUTPUT);
  pinMode(RIGHT_BWD, OUTPUT);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  stopMotors();
  delay(1000);
}

void loop() {
  int numOnLine = 0;
  float weightedSum = 0;

  int firstOn = -1;
  int lastOn  = -1;

  float centerIndex = (LINE_SENSOR_COUNT - 1) / 2.0; // 3.5

  // ----- Read sensors -----
  for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
    bool onLine = sensorOnLine(i);

    if (onLine) {
      if (firstOn == -1) firstOn = i;
      lastOn = i;

      numOnLine++;
      float pos = i - centerIndex;
      weightedSum += pos;
    }
  }

  // ----- Decide movement -----
  if (numOnLine == 0) {
    // *** CHANGE STARTS HERE ***

    // If we just had the line more or less centered,
    // treat this as a small gap/crossing and KEEP GOING STRAIGHT.
    float absErr = lastError;
    if (absErr < 0) absErr = -absErr;

    if (absErr < 0.4) {
      // lastError ~ 0 → go straight through crossing
      applyWheelSpeeds(BASE_SPEED, BASE_SPEED);
    } else {
      // Real line loss → search in last direction
      int searchSpeed = BASE_SPEED;

      if (lastError < 0) {
        applyWheelSpeeds(0, searchSpeed);   // search left
      } else {
        applyWheelSpeeds(searchSpeed, 0);   // search right
      }
    }
    // *** CHANGE ENDS HERE ***
  }
  else if (numOnLine <= 2 &&
           (firstOn == 0 || lastOn == LINE_SENSOR_COUNT - 1)) {
    // ----- SHARP CORNER detection -----
    if (firstOn == 0) {
      pivotLeft();
    } else {
      pivotRight();
    }
  }
  else {
    // Normal line following (curves, slight bends, crossings)
    float error = weightedSum / numOnLine;
    lastError = error;

    int correction = (int)(TURN_GAIN * error);

    int leftSpeed  = BASE_SPEED - correction;
    int rightSpeed = BASE_SPEED + correction;

    leftSpeed  = constrain(leftSpeed, 0, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, 0, MAX_SPEED);

    applyWheelSpeeds(leftSpeed, rightSpeed);
  }

  delay(10);
}

// --- Sensors ---
bool sensorOnLine(int index) {
  int raw = analogRead(sensorPins[index]);
  bool onLine = (raw < LINE_THRESHOLD);  // black < threshold
  return onLine;
}

// --- Motors ---
void applyWheelSpeeds(int leftSpeed, int rightSpeed) {
  leftSpeed  = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // LEFT motor
  if (leftSpeed >= 0) {
    analogWrite(LEFT_FWD, leftSpeed);
    analogWrite(LEFT_BWD, 0);
  } else {
    analogWrite(LEFT_FWD, 0);
    analogWrite(LEFT_BWD, -leftSpeed);
  }

  // RIGHT motor
  if (rightSpeed >= 0) {
    analogWrite(RIGHT_FWD, rightSpeed);
    analogWrite(RIGHT_BWD, 0);
  } else {
    analogWrite(RIGHT_FWD, 0);
    analogWrite(RIGHT_BWD, -rightSpeed);
  }
}

void stopMotors() {
  analogWrite(LEFT_FWD, 0);
  analogWrite(RIGHT_FWD, 0);
  analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_BWD, 0);
}

// Pivot in place to handle sharp turns
void pivotLeft() {
  applyWheelSpeeds(-PIVOT_SPEED, PIVOT_SPEED);
  delay(350);  // fixed duration pivot
  stopMotors();
}

void pivotRight() {
  applyWheelSpeeds(PIVOT_SPEED, -PIVOT_SPEED);
  delay(420);   // slightly longer right pivot
  stopMotors();
}

// --- Ultrasonic (unused) ---
float readDistance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long duration = pulseIn(ECHO, HIGH, 30000);

  if (duration == 0) return 999.0;
  return duration * 0.034 / 2.0;
}
