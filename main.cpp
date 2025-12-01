#include <Arduino.h>
#include "lineDetect.h"

// --- Constants ---
const int SPEED_FAST = 180;
const int WEAK_WHEEL = 185;
const int SPEED_SLOW = 80;

// --- Pin Definitions ---
#define LEFT_BWD 9
#define LEFT_FWD 6
#define RIGHT_BWD 5
#define RIGHT_FWD 3
#define ECHO 12
#define TRIG 4
#define SERVO_PIN 8

// If LINE_SENSOR_COUNT isn't in your .h file, uncomment the line below:
// #define LINE_SENSOR_COUNT 8 

// --- Function Prototypes ---
void stopMotors();
void curveRight();
void curveLeft();
void curveRightCorrect();
void forwardMove();
void applyWheelSpeeds(int leftSpeed, int rightSpeed);
float readDistance();

void setup() {
    Serial.begin(9600);
    
    // Initialize pins (Good practice to explicitly set them)
    pinMode(LEFT_FWD, OUTPUT);
    pinMode(RIGHT_FWD, OUTPUT);
    pinMode(LEFT_BWD, OUTPUT);
    pinMode(RIGHT_BWD, OUTPUT);
    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);

    stopMotors();
    initLineSensors();
}

void loop() {
    // 1. Start by moving forward (default state)
    // Note: If you want smooth following, you might remove this 
    // and let the logic below decide the speed.
    forwardMove();

    // 2. Scan from center outwards
    int leftIndex = 3;  // Start at center-left
    int rightIndex = 4; // Start at center-right
    
    bool isLineFounded = false;

    // Loop through sensors from center to outside
    do {
        // Read sensors
        byte leftValue = readLineSensor(leftIndex);
        byte rightValue = readLineSensor(rightIndex);
        
        bool leftOnLine = isValueOnLine(leftValue);
        bool rightOnLine = isValueOnLine(rightValue);

        if (leftOnLine || rightOnLine) {
            isLineFounded = true;

            if (leftOnLine && !rightOnLine) {
                // LINE IS ON THE LEFT -> Turn Left
                // Calculate dynamic speed based on how far from center we are
                int speedFactor = (LINE_SENSOR_COUNT - 1 - leftIndex); 
                // Example: If index is 0 (far left), factor is high.
                
                applyWheelSpeeds(SPEED_SLOW, speedFactor * 50); 
            } 
            else if (!leftOnLine && rightOnLine) {
                // LINE IS ON THE RIGHT -> Turn Right
                int speedFactor = (rightIndex); // Further right = higher speed
                
                applyWheelSpeeds(speedFactor * 50, SPEED_SLOW);
            } 
            else {
                // Both on line (Center) -> Go Forward
                forwardMove();
            }
        } 
        else {
            // Not found at this index, widen the search
            leftIndex--;
            rightIndex++;
        }

    } while (!isLineFounded && leftIndex >= 0 && rightIndex < LINE_SENSOR_COUNT);
    
    // Optional: Small delay to prevent motor jitter
    // delay(10); 
}

// --- Motor Functions ---

void forwardMove() {
    analogWrite(LEFT_FWD, WEAK_WHEEL);
    analogWrite(RIGHT_FWD, SPEED_FAST);
    analogWrite(LEFT_BWD, 0);
    analogWrite(RIGHT_BWD, 0);
}

void applyWheelSpeeds(int leftSpeed, int rightSpeed) {
    // constrain values to 0-255 to prevent errors
    leftSpeed = constrain(leftSpeed, 0, 255);
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

void curveRight() {
    analogWrite(LEFT_FWD, SPEED_FAST);
    analogWrite(RIGHT_FWD, SPEED_SLOW);
    analogWrite(LEFT_BWD, 0);
    analogWrite(RIGHT_BWD, 0);
}

void curveLeft() {
    analogWrite(RIGHT_FWD, SPEED_FAST);
    analogWrite(LEFT_FWD, SPEED_SLOW);
    analogWrite(LEFT_BWD, 0);
    analogWrite(RIGHT_BWD, 0);
}

void curveRightCorrect() {
    analogWrite(LEFT_FWD, SPEED_FAST);
    analogWrite(RIGHT_FWD, SPEED_FAST - 40); 
    analogWrite(LEFT_BWD, 0);
    analogWrite(RIGHT_BWD, 0);
}

// --- Sensor Functions ---

float readDistance() {
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    long duration = pulseIn(ECHO, HIGH, 30000); // 30ms timeout
    
    if (duration == 0) {
        return 999.0; // Return explicit float
    }

    return duration * 0.034 / 2.0;
}
