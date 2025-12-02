#include <Arduino.h>

// 1. SENSOR LOGIC (FLIPPED)
// We set this to FALSE. Now Black (High Value) = Line.
const bool BLACK_LINE_IS_LOW = false; 

// 2. STEERING DIRECTION
// We keep this TRUE because you said it was turning the wrong way.
const bool INVERT_STEERING = true; 

// 3. SPEED SETTINGS (HIGH POWER)
const int BASE_SPEED = 230;   
const int SEARCH_SPEED = 200; 

// ================================================================

const int sensorPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
#define LEFT_FWD 5
#define LEFT_BWD 6
#define RIGHT_FWD 9
#define RIGHT_BWD 10

int minValues[8];
int maxValues[8];
int lastKnownDirection = 1; // 1 = Right, -1 = Left

// ================= MOTOR CONTROL =================
void setMotors(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  if (left >= 0) {
    analogWrite(LEFT_FWD, left);
    analogWrite(LEFT_BWD, 0);
  } else {
    analogWrite(LEFT_FWD, 0);
    analogWrite(LEFT_BWD, abs(left));
  }

  if (right >= 0) {
    analogWrite(RIGHT_FWD, right);
    analogWrite(RIGHT_BWD, 0);
  } else {
    analogWrite(RIGHT_FWD, 0);
    analogWrite(RIGHT_BWD, abs(right));
  }
}

// ================= CALIBRATION =================
void calibrate() {
  Serial.println("Calibrating... Scan the line!");
  setMotors(200, -200); // Spin fast to calibrate
  
  for(int i=0; i<8; i++) {
    minValues[i] = 1023;
    maxValues[i] = 0;
  }

  for (int i = 0; i < 5000; i++) { 
    for (int j = 0; j < 8; j++) {
      int val = analogRead(sensorPins[j]);
      if (val < minValues[j]) minValues[j] = val;
      if (val > maxValues[j]) maxValues[j] = val;
    }
  }
  setMotors(0, 0);
  delay(1000);
}

// ================= SETUP =================
void setup() {
  Serial.begin(9600);
  pinMode(LEFT_FWD, OUTPUT);
  pinMode(LEFT_BWD, OUTPUT);
  pinMode(RIGHT_FWD, OUTPUT);
  pinMode(RIGHT_BWD, OUTPUT);
  
  delay(1000);
  calibrate();
}

// ================= LOOP =================
void loop() {
  long weightedSum = 0;
  long sum = 0;
  int activeSensors = 0;

  // 1. Read Sensors
  for(int i=0; i<8; i++) {
    int val = analogRead(sensorPins[i]);
    
    // Normalize to 0-1000 range
    int calibratedVal = map(val, minValues[i], maxValues[i], 0, 1000);
    
    // IF BLACK IS HIGH (Standard): 0=White, 1000=Black
    if (BLACK_LINE_IS_LOW) {
       // Invert if your specific sensor logic requires it
       calibratedVal = 1000 - calibratedVal;
    }
    
    calibratedVal = constrain(calibratedVal, 0, 1000);

    // Filter noise: Only count if > 300 (Dark enough to be a line)
    if(calibratedVal > 300) {
        activeSensors++;
    }

    weightedSum += (long)calibratedVal * (i * 1000);
    sum += calibratedVal;
  }

  // 2. CHECK IF LOST (SEARCH MODE)
  if (activeSensors == 0) {
    // If we lost the line, spin in the direction we last saw it
    if (lastKnownDirection == -1) {
       setMotors(-SEARCH_SPEED, SEARCH_SPEED); // Spin Left
    } else {
       setMotors(SEARCH_SPEED, -SEARCH_SPEED); // Spin Right
    }
    return;
  }

  // 3. CALCULATE POSITION & ERROR
  int position = weightedSum / sum; // Center is 3500
  int error = position - 3500;

  // Remember direction for when we get lost next time
  if (error < -500) lastKnownDirection = -1; // Line is Left
  else if (error > 500) lastKnownDirection = 1; // Line is Right

  // 4. STEERING
  int turnAdjustment = error * 0.12; // Gain (Kp)

  if (INVERT_STEERING) {
    turnAdjustment = -turnAdjustment;
  }

  int leftSpeed = BASE_SPEED + turnAdjustment;
  int rightSpeed = BASE_SPEED - turnAdjustment;

  setMotors(leftSpeed, rightSpeed);
}
