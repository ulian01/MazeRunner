#include <Arduino.h>
#include "Config.h"
#include "NeoPixels.h"
#include "LightSensors.h"
#include "UltrasonicSensor.h"
#include "MotorControl.h"
#include "GripperControl.h"
#include "LinePosition.h"

void setup() {
  Serial.begin(9600);
  
  setupNeoPixels();
  setupUltrasonicSensor();
  setupMotors();
  setupGripper();
  
  // Attach Interrupts for Encoders
  attachInterrupt(digitalPinToInterrupt(MOTOR_R1), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_R2), rightEncoderISR, CHANGE);

  Serial.println("R2 Robot Online");
}

void loop() {
  while(!otherRobotDetected){
    unsigned long currentTime = millis();
    
    // Heartbeat for web connectivity check (every 2 seconds)
    static unsigned long lastHeartbeat = 0;
    if (currentTime - lastHeartbeat >= 2000) {
      Serial.println("R2 Waiting for start signal..."); 
      lastHeartbeat = currentTime;
    }

    // Check for Server Start Signal
    if (Serial.available() > 0) {
      String income = Serial.readStringUntil('\n');
      income.trim(); // Remove whitespace/newlines
      
      // Check for Start commands
      if (income.equalsIgnoreCase("START") || 
          income.equalsIgnoreCase("START_R2") || 
          income.equalsIgnoreCase("R2:START_R2") || 
          income.equalsIgnoreCase("R2:START")) {
        Serial.println("R2 Start Signal Received!");
        Serial.println("R2 I started the race");
        otherRobotDetected = true; // Breaks the wait loop
      }
    }


  }

  unsigned long currentTime = millis();
  if(conePickedUp) {
    if (currentTime - previousTime >= gripperInterval) {
      previousTime = currentTime;
      gripper(GRIPPER_CLOSE);
    }
  } else {
    if (currentTime - previousTime >= gripperInterval) {
      previousTime = currentTime;
      gripper(GRIPPER_OPEN);
    }
  }
  
  updateNeoPixels();
  
  if(coneInSquare && !sensorsCalibrated){
    calibrateSensors();
  }

  if (sensorsCalibrated && !conePickedUp) {
    conePickedUp = true;
    return;
  }

  if (sensorsCalibrated && !gameStarted && conePickedUp) {
    turnLeftMillis(90);
    if(robotState != FOLLOW_LINE) return;
    gameStarted = true;
  }

  if (gameStarted && !gameEnded) {
    // Handle active turns to prevent re-entering line logic
    if (robotState == TURNING_LEFT) {
      turnLeftMillis(90);
      updateNeoPixels();
      return;
    }
    if (robotState == TURNING_AROUND) {
      turnAroundMillis();
      updateNeoPixels();
      return;
    }
    
    // Status Report: Motor Speed
    static unsigned long lastSpeedReport = 0;
    if (millis() - lastSpeedReport > 1000) {
       Serial.print("R2 Motor Speed: ");
       Serial.println(baseSpeed); 
       lastSpeedReport = millis();
    }

    getLinePosition();
    
    float dist = getDistance();
    
    if (dist != -1) {  
      if (dist < OBSTACLE_THRESHOLD) {
        Serial.println("R2 OBJECT FOUND!!!");
        Serial.print("R2 Distance: ");
        Serial.println(dist);
        
        stopMotors();
        delay(100);
        resetTicks();
        
        // Calculate ticks for 180 degree tank turn
        // Arc length = (Pi * Diameter) / 2
        float turnDist = (3.14 * DISTANCE_BETWEEN_WHEELS) / 2.0;
        int targetTicks = (turnDist / WHEEL_CIRCUMFERENCE) * PULSE_PER_REVOLUTION;
        
        // Start tank turn (spin in place)
        analogWrite(PIN_LEFT_FWD, 220);     // Left Forward
        digitalWrite(PIN_LEFT_BWD, LOW);
        
        digitalWrite(PIN_RIGHT_FWD, LOW);
        analogWrite(PIN_RIGHT_BWD, 220);    // Right Backward
        
        // Block until turn is complete using encoder ticks
        while(abs(_leftTicks) < targetTicks) {
          // Wait for turn to complete
        }
        
        stopMotors();
        robotState = FOLLOW_LINE;
        linePosition = CENTER_LINE; 
        return;
      }
    }
    
    switch (linePosition) {
      case T_JUNCTION:
        // Drive forward 1/3 of a wheel rotation to check if it's the finish line
        resetTicks();
        {
          int checkTicks = PULSE_PER_REVOLUTION / 3;
          moveForward(150, 150);
          while(_leftTicks < checkTicks && _rightTicks < checkTicks) {
            // Wait for movement
          }
        }
        stopMotors();
        
        readSensors();
        
        {
          bool allBlack = true;
          for (int i = 0; i < NUM_SENSORS; i++) {
            if (sensorValues[i] < sensorThreshold[i]) {
              allBlack = false;
              break;
            }
          }
          
          if (allBlack) {
            // Still all black -> Finish Line
            Serial.println("R2 Dropping Cone");
            coneDroppedOff = true;
            gripper(GRIPPER_OPEN);
            delay(500);
            
            // Reverse 45cm
            resetTicks();
            int targetTicks = 45; // Approx 45cm
            
            // Set motors to reverse (increased power)
            analogWrite(PIN_LEFT_BWD, 200);
            digitalWrite(PIN_LEFT_FWD, LOW);
            analogWrite(PIN_RIGHT_BWD, 200);
            digitalWrite(PIN_RIGHT_FWD, LOW);
            
            while(abs(_leftTicks) < targetTicks && abs(_rightTicks) < targetTicks) {
              delay(10);
            }
            
            stopMotors();
            Serial.println("R2 Race Finished");
            gameEnded = true;
          } else {
            // Not all black -> Just a crossing
            Serial.println("R2 Turning Left at Junction");
            turnLeftMillis(120);
          }
        }
        break;
        
      case LEFT_LINE:
        Serial.println("R2 Turning Left");
        turnLeftMillis(140);
        readSensors();
        {
          bool lineDetected = false;
          for (int i = 0; i < NUM_SENSORS; i++) {
            if (sensorValues[i] > sensorThreshold[i]) {
              lineDetected = true;
              break;
            }
          }
          
          if (!lineDetected) {
            updateNeoPixels();
            return;
          }
        }
        break;
        
      case NO_LINE:
        Serial.println("R2 I lost the line And now I try searching for it");
        turnAroundMillis();
        break;
        
      case RIGHT_LINE:
        linePosition = CENTER_LINE;
        robotState = FOLLOW_LINE;
        moveForwardPID(baseSpeed, baseSpeed, false, true);
        break;
        
      case CENTER_LINE:
      default:
        moveForwardPID(baseSpeed, baseSpeed, false, true);
        break;
    }
  }

  updateNeoPixels();
} 