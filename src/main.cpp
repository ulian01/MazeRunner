#include <Arduino.h>
#include "lineDetect.h"

#define LEFT_BWD 9
#define LEFT_FWD 6
#define RIGHT_BWD 5
#define RIGHT_FWD 3
#define ECHO 12
#define TRIG 4
#define SERVO_PIN 8

void stopMotors();
void curveRight();
void curveLeft();
void curveRightCorrect();
void applyWheelSpeeds(int leftSpeed, int rightSpeed);



void setup(){

  Serial.begin(9600);
  stopMotors();

  initLineSensors();
}


void loop(){
    return;
  

  // drives forwards normally
  forwardMove();

  // get middle sensor values (3,4)
  int leftIndex = 3;
  int rightIndex = 4;
  byte leftMiddleSensor = readLineSensor(leftIndex);
  byte rightMiddleSensor = readLineSensor(rightIndex);

  bool leftOnLine = isValueOnLine(leftMiddleSensor);
  bool rightOnLine = isValueOnLine(rightMiddleSensor);
  bool isLineFounded = false;

  do
  {
    if (leftOnLine || rightOnLine)
    {
      isLineFounded = true;
      // calculate wheel speeds based on shift from center
      // which side is on line
      if (leftOnLine && !rightOnLine)
      {
        // calculate wheel speeds to curve right
        int leftSpeed = LINE_SENSOR_COUNT - 1 - leftIndex;
        int rightSpeed = leftIndex + 1;

        // use leftSpeed to adjust the speed of the left wheel
        // bigger value is faster wheel
        applyWheelSpeeds(leftSpeed * 50, rightSpeed * 50);
      }
      else if (!leftOnLine && rightOnLine)
      {
        // calculate wheel speeds to curve left
        int rightSpeed = LINE_SENSOR_COUNT - 1 - rightIndex;
        int leftSpeed = rightIndex + 1;

        // use rightSpeed to adjust the speed of the right wheel
        applyWheelSpeeds(leftSpeed * 50, rightSpeed * 50);
      }
      else
      {
        // both on line, go forward
        forwardMove();
      }
    }
    else
    {
      // shift left and right to find the line again
      leftIndex--;
      rightIndex++;
      leftOnLine = isValueOnLine(readLineSensor(leftIndex));
      rightOnLine = isValueOnLine(readLineSensor(rightIndex));
    }
  } while (!isLineFounded && leftIndex >= 0 && rightIndex < LINE_SENSOR_COUNT);
}
  // motorfunctions

  void forwardMove()
  {
    analogWrite(LEFT_FWD, WEAK_WHEEL);
    analogWrite(RIGHT_FWD, SPEED_FAST);
    analogWrite(LEFT_BWD, 0);
    analogWrite(RIGHT_BWD, 0);
  }

// motorfunctions
  void applyWheelSpeeds(int leftSpeed, int rightSpeed)
  {
    // TODO fix this?
    analogWrite(LEFT_FWD, leftSpeed);
    analogWrite(RIGHT_FWD, rightSpeed);
    analogWrite(LEFT_BWD, 0);
    analogWrite(RIGHT_BWD, 0);
  }

void forwardMove()
{
  analogWrite(LEFT_FWD, WEAK_WHEEL);
  analogWrite(RIGHT_FWD, SPEED_FAST);
  analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_BWD, 0);
}
  void stopMotors()
  {
    analogWrite(LEFT_FWD, 0);
    analogWrite(RIGHT_FWD, 0);
    analogWrite(LEFT_BWD, 0);
    analogWrite(RIGHT_BWD, 0);
  }

void stopMotors()
{
  analogWrite(LEFT_FWD, 0);
  analogWrite(RIGHT_FWD, 0);
  analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_BWD, 0);
}
  void curveRight()
  {
    analogWrite(LEFT_FWD, SPEED_FAST);
    analogWrite(RIGHT_FWD, SPEED_SLOW);
    analogWrite(LEFT_BWD, 0);
    analogWrite(RIGHT_BWD, 0);
  }

void curveRight()
{
  analogWrite(LEFT_FWD, SPEED_FAST);
  analogWrite(RIGHT_FWD, SPEED_SLOW);
  analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_BWD, 0);
}
  void curveLeft()
  {
    analogWrite(RIGHT_FWD, SPEED_FAST);
    analogWrite(LEFT_FWD, SPEED_SLOW);
    analogWrite(LEFT_BWD, 0);
    analogWrite(RIGHT_BWD, 0);
  }

void curveLeft()
{
  analogWrite(RIGHT_FWD, SPEED_FAST);
  analogWrite(LEFT_FWD, SPEED_SLOW);
  analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_BWD, 0);
}
  void curveRightCorrect()
  {
    analogWrite(LEFT_FWD, SPEED_FAST);
    analogWrite(RIGHT_FWD, SPEED_FAST - 40); // abit more to the right
    analogWrite(LEFT_BWD, 0);
    analogWrite(RIGHT_BWD, 0);
  }

void curveRightCorrect()
{
  analogWrite(LEFT_FWD, SPEED_FAST);
  analogWrite(RIGHT_FWD, SPEED_FAST - 40); // abit more to the right
  analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_BWD, 0);
}
  // sensor

// sensor
  float readDistance()
  {
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

float readDistance()
{
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long duration = pulseIn(ECHO, HIGH, 30000);
  if (duration == 0)
    return 999;

  return duration * 0.034 / 2.0;
}
    long duration = pulseIn(ECHO, HIGH, 30000);
    if (duration == 0)
      return 999;

    return duration * 0.034 / 2.0;
  }






