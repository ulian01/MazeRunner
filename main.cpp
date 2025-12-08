#include <Arduino.h>                         // Include core Arduino functions (pinMode, analogRead, etc.)

// ---------------- LINE SENSORS ----------------
#define NUM_SENSORS 8                        // We have 8 line sensors in total

int sensorPins[NUM_SENSORS] = {              // Which Arduino pin each sensor is connected to
  A7, A6, A5, A4, A3, A2, A1, A0              // D1 = A0, D2 = A1, ... , D8 = A7
};
int sensorValues[NUM_SENSORS];           // Latest analog value from each sensor

// ---------------- MOTOR PINS (YOUR PINS) ----------------
//  - 11 & 10 = left motor
//  - 6  & 5  = right motors
const int MOTOR_A_1 = 6;                    // Left motor forward (PWM)
const int MOTOR_A_2 = 9;                    // Left motor backward
const int MOTOR_B_1 = 3;                     // Right motor forward (PWM)
const int MOTOR_B_2 = 5;                     // Right motor backward

// --- PID control variables ---
int error = 0;                               // Current error (line position - center)
int lastError = 0;                           // Error from previous loop It’s used to see how fast the error is changing
float integral = 0;                          // Sum of errors, how long you’ve been wrong.
float derivative = 0;                        // Change in error, how fast things are changing.

// PID gains
float Kp;                                    // Proportional gain controls how strongly the robot reacts to the current error.
float Ki;                                    // Integral gain, If the robot is always slightly to one side, Ki helps push it back.
float Kd;                                    // Derivative gain, Kd controls how much the derivative (change in error) matters.
int correction;                              // Final correction applied to motor speeds

// ---------- CROSSING DETECTION CONFIG ----------
// Treat analog values BELOW this as "black".
const int BLACK_THRESHOLD = 500;             // Example: <500 = black, >500 = white

const unsigned long CROSSING_IGNORE_TIME = 250; // 0.25 s in milliseconds

bool allBlackActive = false;                 // Are we currently in an "all sensors black" period?
unsigned long allBlackStartTime = 0;         // When that period started
int lastPosition = (NUM_SENSORS - 1) * 1000 / 2; // Last valid line position (start at center)

// ---------- FUNCTION HEADERS ----------
void readSensors();
int  calculateLinePosition();
void followLine(int position);
void moveForward(int _leftSpeed, int _rightSpeed);
void stopMotors();
bool allSensorsBlack();

void setup() {
    Serial.begin(9600);                      // Serial for debugging

    pinMode(MOTOR_A_1, OUTPUT);
    pinMode(MOTOR_A_2, OUTPUT);
    pinMode(MOTOR_B_1, OUTPUT);
    pinMode(MOTOR_B_2, OUTPUT);
    //TODO: initialize line sensors here to prevent errors

    stopMotors();                            // Make sure motors are off at startup
}

void loop() {
    readSensors();                           // Read all 8 sensors

    unsigned long now = millis();            // Current time in ms

    bool nowAllBlack = allSensorsBlack();    // Check if all sensors see black
    int position;                            // Line position for this loop

    if (nowAllBlack) {
        // Just entered all-black region
        if (!allBlackActive) {
            allBlackActive = true;
            allBlackStartTime = now;         // Start timing this all-black period
        }

        unsigned long duration = now - allBlackStartTime;

        if (duration < CROSSING_IGNORE_TIME) {
            // Use the last good line position instead of recalculating
            position = lastPosition;
        } else {
            // All-black for >= 0.5 s → treat as normal (or later you can add special behaviour)
            position = calculateLinePosition();
        }
    } else {
        // We are NOT in all-black now
        if (allBlackActive) {
            // We just left an all-black period (could log duration if needed)
            allBlackActive = false;
        }

        // Normal situation: some sensors see black, some white
        position = calculateLinePosition();
    }

    Serial.print("Line Position: ");
    Serial.println(position);

    followLine(position);                    // PID line following using selected position
    lastPosition = position;                 // Remember this position for the next loop

    delay(50);                               // Loop delay
}

// ------------ Read all sensors ------------
void readSensors() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensorValues[i] = analogRead(sensorPins[i]); // 0–1023 per sensor
    }
}

// ------------ Check if ALL sensors see black ------------
bool allSensorsBlack() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        // Assuming: lower value = darker = black.
        // If your sensors are inverted, change '<' to '>'.
        if (sensorValues[i] > BLACK_THRESHOLD) {
            return false;                    // This sensor is not black enough → not all black
        }
    }
    return true;                             // All sensors are under threshold → all black
}

// ------------ Calculate line position (weighted average) ------------
int calculateLinePosition() {
    long weightedSum = 0;
    long sum = 0;

    for (int i = 0; i < NUM_SENSORS; i++) {
        int value = 1023 - sensorValues[i];     // now BLACK = high value

        weightedSum += (long)value * i * 1000;  // Weighted by index (0..7) and scaled by 1000
        sum += value;
    }

    // We assume sum != 0 (otherwise no signal).
    return weightedSum / sum;                // 0..7000, center ≈ 3500
}

// ------------ PID line following ------------
void followLine(int position) {

    // PID tuning values (yours)
   Kp = 1;
   Ki = 0.0002;
   Kd = 0.6;

                                // Derivative gain

    int center = (NUM_SENSORS - 1) * 1000 / 2; // Center of sensor array (≈3500)

    error = (position - center)/2;               // Negative: line left, positive: line right
    integral += error;                       // Sum of errors over time
    derivative = error - lastError;          // Change in error since last loop

    correction = (Kp * error) +              // P part
                 (Ki * integral) +           // I part
                 (Kd * derivative);          // D part

    Serial.println(error);                   // For debugging/tuning

    int baseSpeed = 240;                     // Base speed for both motors
    int _leftSpeed  = baseSpeed - correction;// Left motor gets less when we turn left
    int _rightSpeed = baseSpeed + correction;// Right motor gets more when we turn left

    _leftSpeed  = constrain(_leftSpeed,  0, 255); // constrain is an Arduino function that forces a value to stay inside a range.
    _rightSpeed = constrain(_rightSpeed, 0, 255);

    moveForward(_leftSpeed, _rightSpeed);    // Command motors

    lastError = error;                       // Save for next derivative calculation
}

// ------------ Move forward with given speeds ------------
void moveForward(int _leftSpeed, int _rightSpeed) {
    analogWrite(MOTOR_A_1, _leftSpeed);      // Left motor forward PWM
    analogWrite(MOTOR_A_2, 0);               // Left motor backward off

    analogWrite(MOTOR_B_1, _rightSpeed);     // Right motor forward PWM
    analogWrite(MOTOR_B_2, 0);               // Right motor backward off
}

// ------------ Stop both motors ------------
void stopMotors() {
    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_A_2, 0);
    analogWrite(MOTOR_B_1, 0);
    analogWrite(MOTOR_B_2, 0);
}
