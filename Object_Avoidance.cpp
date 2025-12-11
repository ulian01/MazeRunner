#include <Arduino.h>

// ==========================================
// 1. HARDWARE CONFIGURATION
// ==========================================

// --- Motor Pins ---
// If a wheel spins backward when it should go forward, swap these numbers!
const int LEFT_FWD  = 5; 
const int LEFT_BWD  = 6;
const int RIGHT_FWD = 9;
const int RIGHT_BWD = 10;

// --- Speed Settings (0 - 255) ---
// If the robot pulls to the RIGHT, lower LEFT_SPEED.
// If the robot pulls to the LEFT, lower RIGHT_SPEED.
const int LEFT_SPEED  = 200; 
const int RIGHT_SPEED = 200; 
const int TURN_SPEED  = 200; // Speed during turns

// --- Ultrasonic Sensor ---
// CHECK WIRING: VCC->5V, GND->GND, Trig->12, Echo->4
const int TRIG_PIN = 2;
const int ECHO_PIN = 3;

// --- Distances & Timing ---
const int OBSTACLE_DIST_CM  = 20;   // Stop if closer than this
const int SENSE_INTERVAL    = 60;   // INCREASED to 60ms to prevent ghost echoes

// --- Avoidance Routine Durations (ms) ---
const unsigned long TURN_LEFT_TIME  = 500;
const unsigned long TURN_RIGHT_TIME = 500;
const unsigned long FWD_SHORT_TIME  = 1000;
const unsigned long FWD_LONG_TIME   = 2000;

// ==========================================
// 2. STATE MACHINE SETUP
// ==========================================

enum RobotState {
  DRIVE_FORWARD,
  STOP_AND_THINK,      // Tiny pause before reacting
  AVOID_TURN_LEFT1,
  AVOID_FORWARD1,       
  AVOID_TURN_RIGHT1,
  AVOID_FORWARD2,       
  AVOID_TURN_RIGHT2,
  AVOID_FORWARD3,       
  AVOID_TURN_LEFT2
};

RobotState currentState = DRIVE_FORWARD;
unsigned long stateStartTime = 0;
unsigned long lastSenseTime  = 0;
long currentDistance = 999;

// ==========================================
// 3. MOVEMENT FUNCTIONS
// ==========================================

void stopMotors() {
  analogWrite(LEFT_FWD, 0);
  analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_FWD, 0);
  analogWrite(RIGHT_BWD, 0);
}

void moveForward() {
  analogWrite(LEFT_FWD, LEFT_SPEED);
  analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_FWD, RIGHT_SPEED);
  analogWrite(RIGHT_BWD, 0);
}

void turnLeft() {
  // Left wheel back, Right wheel forward (Pivot Turn)
  analogWrite(LEFT_FWD, 0);
  analogWrite(LEFT_BWD, TURN_SPEED);
  analogWrite(RIGHT_FWD, TURN_SPEED);
  analogWrite(RIGHT_BWD, 0);
}

void turnRight() {
  // Left wheel forward, Right wheel back (Pivot Turn)
  analogWrite(LEFT_FWD, TURN_SPEED);
  analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_FWD, 0);
  analogWrite(RIGHT_BWD, TURN_SPEED);
}

// ==========================================
// 4. SENSOR FUNCTION
// ==========================================
long getDistance() {
  // Ensure clear signal
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // Send ping
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for echo (Timeout 30ms = ~5 meters)
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); 

  if (duration == 0) return 999; // No echo received (too far or disconnected)
  
  return (duration * 0.034 / 2);
}

// ==========================================
// 5. MAIN LOOPS
// ==========================================

void setup() {
  Serial.begin(9600);
  
  pinMode(LEFT_FWD, OUTPUT);
  pinMode(LEFT_BWD, OUTPUT);
  pinMode(RIGHT_FWD, OUTPUT);
  pinMode(RIGHT_BWD, OUTPUT);
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW); // Initialize Low
  
  delay(1000); // Wait 1 sec before starting
  Serial.println("Robot Initialized.");
}

void loop() {
  unsigned long now = millis();

  // --- 1. Read Sensor ---
  if (now - lastSenseTime >= SENSE_INTERVAL) {
    long newDist = getDistance();
    
    // Only update if reading seems valid (filter out 0 noise if needed)
    currentDistance = newDist;
    
    lastSenseTime = now;
    
    // Debugging (Check your Serial Monitor!)
    // If this prints "999" constantly, check wiring (Trig/Echo swap?)
    Serial.print("State: ");
    Serial.print(currentState);
    Serial.print(" | Dist: ");
    Serial.println(currentDistance);
  }

  // --- 2. State Machine ---
  
  switch (currentState) {
    
    // STATE: DRIVING NORMALLY
    case DRIVE_FORWARD:
      moveForward();
      // Check collision
      if (currentDistance < OBSTACLE_DIST_CM) {
        stopMotors();
        currentState = STOP_AND_THINK;
        stateStartTime = now;
      }
      break;

    // STATE: PAUSE BEFORE MANEUVER (Helps stability)
    case STOP_AND_THINK:
      if (now - stateStartTime > 200) { // 200ms pause
        currentState = AVOID_TURN_LEFT1;
        stateStartTime = now;
      }
      break;

    // --- MANEUVER SEQUENCE ---
    
    case AVOID_TURN_LEFT1:
      turnLeft();
      if (now - stateStartTime >= TURN_LEFT_TIME) {
        stopMotors(); // Brief stop prevents momentum drift
        currentState = AVOID_FORWARD1;
        stateStartTime = now;
      }
      break;

    case AVOID_FORWARD1:
      moveForward();
      if (now - stateStartTime >= FWD_SHORT_TIME) {
        stopMotors();
        currentState = AVOID_TURN_RIGHT1;
        stateStartTime = now;
      }
      break;

    case AVOID_TURN_RIGHT1:
      turnRight();
      if (now - stateStartTime >= TURN_RIGHT_TIME) {
        stopMotors();
        currentState = AVOID_FORWARD2;
        stateStartTime = now;
      }
      break;

    case AVOID_FORWARD2:
      moveForward();
      if (now - stateStartTime >= FWD_LONG_TIME) {
        stopMotors();
        currentState = AVOID_TURN_RIGHT2;
        stateStartTime = now;
      }
      break;

    case AVOID_TURN_RIGHT2:
      turnRight();
      if (now - stateStartTime >= TURN_RIGHT_TIME) {
        stopMotors();
        currentState = AVOID_FORWARD3;
        stateStartTime = now;
      }
      break;

    case AVOID_FORWARD3:
      moveForward();
      if (now - stateStartTime >= FWD_SHORT_TIME) {
        stopMotors();
        currentState = AVOID_TURN_LEFT2;
        stateStartTime = now;
      }
      break;

    case AVOID_TURN_LEFT2:
      turnLeft();
      if (now - stateStartTime >= TURN_LEFT_TIME) {
        stopMotors();
        currentState = DRIVE_FORWARD; // Reset to normal behavior
      }
      break;
  }
}