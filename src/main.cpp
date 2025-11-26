#include <Arduino.h>
#include "lineDetect.h"

#define LEFT_FWD 6
#define LEFT_BWD 9
#define RIGHT_FWD 3
#define RIGHT_BWD 5
#define TRIG 4
#define ECHO 12

// Speeds
const int SPEED_FAST = 180;
const int WEAK_WHEEL = 185;
const int SPEED_SLOW = 80;

// Detection distance
const float WALL_DIST = 18;

// Bocht- en correctietijden
const unsigned long TURN_RIGHT_FIRST = 1000;   // first turn to the right
const unsigned long TURN_LEFT_SECOND = 1400;   // second turn to the left
const unsigned long TURN_RIGHT_CORRECT = 1800; // correction to the right

// going forwards
const unsigned long FORWARD_LONG = 812;

float readDistance();
void forwardMove();
void stopMotors();
void curveRight();
void curveLeft();
void curveRightCorrect();
void applyWheelSpeeds(int leftSpeed, int rightSpeed);

void setup()
{
  pinMode(LEFT_FWD, OUTPUT);
  pinMode(LEFT_BWD, OUTPUT);
  pinMode(RIGHT_FWD, OUTPUT);
  pinMode(RIGHT_BWD, OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  Serial.begin(9600);
  stopMotors();

  initLineSensors();
}

void loop()
{

  float d = readDistance();

  static int confirm = 0;
  static bool avoidActive = false;

  if (!avoidActive)
  { // if there is an object
    if (d > 0 && d < WALL_DIST)
      confirm++;
    else
      confirm = 0; // sets that theres no object
  }

  if (!avoidActive && confirm >= 2)
  { // checks if theres an object

    avoidActive = true;
    confirm = 0;

    stopMotors();
    delay(80);

    // first turn to the right
    curveRight();
    delay(TURN_RIGHT_FIRST);
    forwardMove();
    delay(FORWARD_LONG);
    stopMotors();
    delay(100);

    // second turn to the left and going forward
    curveLeft();
    delay(TURN_LEFT_SECOND);
    forwardMove();
    delay(430); // going forward just a little

    // correction to the right
    curveRightCorrect();
    delay(TURN_RIGHT_CORRECT);
    forwardMove();
    stopMotors();
    delay(100);
    forwardMove();
    delay(1000);

    avoidActive = false; // sets it on false again
    return;
  }

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

  // motorfunctions

  void forwardMove()
  {
    analogWrite(LEFT_FWD, WEAK_WHEEL);
    analogWrite(RIGHT_FWD, SPEED_FAST);
    analogWrite(LEFT_BWD, 0);
    analogWrite(RIGHT_BWD, 0);
  }

  void applyWheelSpeeds(int leftSpeed, int rightSpeed)
  {
    // TODO fix this?
    analogWrite(LEFT_FWD, leftSpeed);
    analogWrite(RIGHT_FWD, rightSpeed);
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

  // sensor

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