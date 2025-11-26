#include <Arduino.h>

// --- PIN DEFINITIONS ---
const int MOTORA1 = 6;  // Left wheel backward (PWM)
const int MOTORA2 = 5;  // Left wheel forward (PWM)
const int MOTORB1 = 10; // Right wheel backward (PWM)
const int MOTORB2 = 9;  // Right wheel forward (PWM)
const int TRIGPIN = 2;  // Trigger Pin (Ultrasonic Sensor)
const int ECHOPIN = 3;  // Echo Pin (Ultrasonic Sensor)

// --- ULTRASONIC & TIMING CONSTANTS ---
const float US_PER_CM = 58.0;      
const long TIMEOUT_US = 23200;     
const float AVOID_DISTANCE = 15.0; 
const float MIN_READ_CM = 1.0;     

// --- TIMING FOR ACTIONS (in milliseconds) ---
const unsigned long BACKUP_TIME_MS = 1000; // Original MoveBackward time
const unsigned long TURN_R_TIME_MS = 1160;  // Original TurnR time

// --- ROBOT STATE DEFINITIONS ---
enum RobotState { 
    DRIVING, 
    INITIATE_AVOID, 
    BACKING_UP, 
    TURNING_RIGHT, 
    READY_TO_DRIVE 
};

// Global State Variables
RobotState currentState = DRIVING;
unsigned long actionStartTime = 0; // Tracks when a time-based action began

// --- FUNCTION PROTOTYPES ---
void RunMotors(int A1, int A2, int B1, int B2);
void StopMotors_NonBlocking();
void MoveForward_NonBlocking(); 
float getDistanceCM();         
void UpdateAvoidanceLogic();    

// -------------------------------------------------------------------

void setup() {
  pinMode(MOTORA1, OUTPUT);
  pinMode(MOTORA2, OUTPUT);
  pinMode(MOTORB1, OUTPUT);
  pinMode(MOTORB2, OUTPUT);
  pinMode(TRIGPIN, OUTPUT);
  pinMode(ECHOPIN, INPUT);
  Serial.begin(9600);
  Serial.println("Robot Initialized: DRIVING (Non-Blocking)");
}

// -------------------------------------------------------------------
// --- LOOP (The Brain) ---
// -------------------------------------------------------------------
void loop() {
  // 1. Always check the sensor and update the state variable.
  UpdateAvoidanceLogic();
  
  // 2. Based on the state, run the motor commands.
  switch (currentState) {
    
    case DRIVING:
      MoveForward_NonBlocking();
      break;

    case INITIATE_AVOID:
      // Stop and set the action start time for the next step.
      StopMotors_NonBlocking();
      Serial.println("-> Obstacle Detected. BACKING UP.");
      actionStartTime = millis(); // Record the start time
      currentState = BACKING_UP;
      break; 
      
    case BACKING_UP:
      RunMotors(210, 0, 235); // Start moving backward
      
      // Check if the required time has elapsed
      if (millis() - actionStartTime >= BACKUP_TIME_MS) {
        StopMotors_NonBlocking();
        Serial.println("-> Backup complete. TURNING RIGHT.");
        actionStartTime = millis(); // Reset timer for the next action
        currentState = TURNING_RIGHT;
      }
      break;

    case TURNING_RIGHT:
      RunMotors(0, 250, 250); // Start turning right
      
      // Check if the required time has elapsed
      if (millis() - actionStartTime >= TURN_R_TIME_MS) {
        StopMotors_NonBlocking();
        Serial.println("-> Turn complete. Resuming DRIVING.");
        currentState = DRIVING; // Go back to driving
      }
      break;
      
    case READY_TO_DRIVE: // Used briefly as a stop point after a maneuver
      StopMotors_NonBlocking();
      currentState = DRIVING;
      break;
  }
}

// -------------------------------------------------------------------
// --- MOVEMENT HELPER FUNCTIONS (NON-BLOCKING) ---
// -------------------------------------------------------------------

void RunMotors(int A1, int A2, int B1, int B2){
  analogWrite(MOTORA1, A1);
  analogWrite(MOTORA2, A2);
  analogWrite(MOTORB1, B1);
  analogWrite(MOTORB2, B2);
}

void StopMotors_NonBlocking(){
  RunMotors(0, 0, 0, 0);
}

void MoveForward_NonBlocking(){
  RunMotors(0, 236, 0, 243);
}

// -------------------------------------------------------------------
// --- SENSOR READING & LOGIC ---
// -------------------------------------------------------------------

float getDistanceCM() {
  // Send 10µs pulse
  digitalWrite(TRIGPIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGPIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGPIN, LOW);

  // Measure the duration of the echo pulse
  long duration_us = pulseIn(ECHOPIN, HIGH, TIMEOUT_US);
  
  // Calculate and return distance in cm
  return duration_us / US_PER_CM;
}

// This function ONLY checks the sensor and updates the state variable.
void UpdateAvoidanceLogic() {
  float distance_cm = getDistanceCM();

  // If currently DRIVING AND obstacle is detected (within threshold)
  if (currentState == DRIVING && distance_cm > MIN_READ_CM && distance_cm <= AVOID_DISTANCE) {
    currentState = INITIATE_AVOID;
  }
  
  // If we were in an avoidance state, but the path is now clear (> threshold), 
  // and we are NOT in the middle of a time-critical maneuver, we can abort the maneuver and resume driving.
  else if (currentState != DRIVING && distance_cm > AVOID_DISTANCE && currentState != INITIATE_AVOID) {
    // If the path clears during a turn/backup, we can go straight back to driving.
    // However, for clean turns, it's usually better to let the action finish,
    // so we only reset if the path clears after the sequence.
    if (currentState == READY_TO_DRIVE) {
      Serial.println("-> Path clear. Resuming DRIVING.");
      currentState = DRIVING;
    }
  }
}
