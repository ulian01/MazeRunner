#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

// ---------------- LINE SENSORS ----------------
#define NUM_SENSORS 8

const int sensorPins[NUM_SENSORS] = { A0, A1, A2, A3, A4, A5, A6, A7 };
int sensorValues[NUM_SENSORS];

// ---------------- MOTOR PINS ----------------
const int MOTOR_A_1 = 6;   // Left forward
const int MOTOR_A_2 = 9;   // Left backward
const int MOTOR_B_1 = 3;   // Right forward
const int MOTOR_B_2 = 5;   // Right backward

// --- PID control variables ---
int error = 0;
int lastError = 0;
float integral = 0;
float derivative = 0;

float Kp, Ki, Kd;
int correction;

// ---------- CROSSING DETECTION ----------
const int BLACK_THRESHOLD = 500;
const unsigned long CROSSING_IGNORE_TIME = 250;

bool allBlackActive = false;
unsigned long allBlackStartTime = 0;
int lastPosition = (NUM_SENSORS - 1) * 1000 / 2;

// ---------- FUNCTION HEADERS ----------
void readSensors();
int  calculateLinePosition();
void followLine(int position);
void moveForward(int _leftSpeed, int _rightSpeed);
void stopMotors();
bool allSensorsBlack();

void setup() {
    Serial.begin(9600);

    for (int i = 0; i < NUM_SENSORS; i++)
        pinMode(sensorPins[i], INPUT);

    pinMode(MOTOR_A_1, OUTPUT);
    pinMode(MOTOR_A_2, OUTPUT);
    pinMode(MOTOR_B_1, OUTPUT);
    pinMode(MOTOR_B_2, OUTPUT);

    stopMotors();
}

void loop() {
    readSensors();

    unsigned long now = millis();
    bool nowAllBlack = allSensorsBlack();
    int position;

    if (nowAllBlack) {
        if (!allBlackActive) {
            allBlackActive = true;
            allBlackStartTime = now;
        }

        unsigned long duration = now - allBlackStartTime;

        if (duration < CROSSING_IGNORE_TIME) {
            position = lastPosition;
        } else {
            position = calculateLinePosition();
        }
    }
    else {
        allBlackActive = false;
        position = calculateLinePosition();
    }

    followLine(position);
    lastPosition = position;

    delay(50);
}

// ------------ Read sensors ------------
void readSensors() {
    for (int i = 0; i < NUM_SENSORS; i++)
        sensorValues[i] = analogRead(sensorPins[i]);
}

// ------------ Check if all sensors see black ------------
bool allSensorsBlack() {
    for (int i = 0; i < NUM_SENSORS; i++)
        if (sensorValues[i] > BLACK_THRESHOLD)
            return false;
    return true;
}

// ------------ Weighted average for line position ------------
int calculateLinePosition() {
    long weightedSum = 0;
    long sum = 0;

    for (int i = 0; i < NUM_SENSORS; i++) {
        int value = 1023 - sensorValues[i];
        weightedSum += (long)value * i * 1000;
        sum += value;
    }

    return weightedSum / sum;
}

// ------------ PID line following ------------
void followLine(int position) {

    Kp = 1;
    Ki = 0.0002;
    Kd = 1;

    int center = (NUM_SENSORS - 1) * 1000 / 2;

    error = (position - center) / 2;
    integral += error;
    derivative = error - lastError;

    correction =
        (Kp * error) +
        (Ki * integral) +
        (Kd * derivative);

    int baseSpeed = 240;
    int _leftSpeed  = constrain(baseSpeed - correction, 0, 255);
    int _rightSpeed = constrain(baseSpeed + correction, 0, 255);

    moveForward(_leftSpeed, _rightSpeed);

    lastError = error;
}

// ------------ Move forward ------------
void moveForward(int _leftSpeed, int _rightSpeed) {
    analogWrite(MOTOR_A_1, _leftSpeed);
    analogWrite(MOTOR_A_2, 0);

    analogWrite(MOTOR_B_1, _rightSpeed);
    analogWrite(MOTOR_B_2, 0);
}

// ------------ Stop motors ------------
void stopMotors() {
    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_A_2, 0);
    analogWrite(MOTOR_B_1, 0);
    analogWrite(MOTOR_B_2, 0);
}
