#include <Arduino.h>

#define LEFT_BWD 9
#define LEFT_FWD 6
#define RIGHT_BWD 5
#define RIGHT_FWD 3
#define ECHO 12
#define TRIG 4
#define SERVO_PIN 8   

const int FORWARD_SPEED = 150;
const int TURN_SPEED = 170;

const float WALL_DIST  = 18.0;
const float CLEAR_DIST = 30.0;

const unsigned long TURN_90   = 350;
const unsigned long BACK_TIME = 250;

//functions for the motor
float readDistance();
void forwardMove();
void backwardMove();
void stopMotors();
void turnLeft();
void turnRight();

//functions for the servo
void servoWriteMicroseconds(int us);
void servoWrite(int angle);
void lookCenter();
void lookLeft();
void lookRight();

bool scanDirectionRight();
bool scanDirectionLeft();

void setup() {
  pinMode(LEFT_FWD, OUTPUT);
  pinMode(LEFT_BWD, OUTPUT);
  pinMode(RIGHT_FWD, OUTPUT);
  pinMode(RIGHT_BWD, OUTPUT);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  pinMode(SERVO_PIN, OUTPUT);   // SERVO OUTPUT
  lookCenter();                 // sets the neck forwards when started

  Serial.begin(9600);
  stopMotors();
}

void loop() {

  float d = readDistance();

  //avoids walls
  if (d > 0 && d < WALL_DIST) {

    stopMotors();
    delay(50);

    // little backup
    backwardMove();
    delay(BACK_TIME);
    stopMotors();
    delay(80);

    // servo looks to the right
    if (scanDirectionRight()) {
      turnRight();
      delay(TURN_90);
      stopMotors();
      delay(100);
      return;
    }

    // move forwards
    if (readDistance() > CLEAR_DIST) {
      forwardMove();
      return;
    }

    // servo checks left
    if (scanDirectionLeft()) {
      turnLeft();
      delay(TURN_90);
      stopMotors();
      delay(100);
      return;
    }

    // options left full turn
    turnRight();
    delay(TURN_90 * 2);
    stopMotors();
    delay(100);
    return;
  }

  forwardMove();

  // checks if BDP is stuck and tries to get itself out of its current situation
  static float lastDistance = 0;
  static unsigned long lastCheck = 0;
  static int stuckCounter = 0;

  if (millis() - lastCheck > 120) {
    lastCheck = millis();
    float distNow = readDistance();

    if (abs(distNow - lastDistance) < 1.5) stuckCounter++;
    else stuckCounter = 0;

    lastDistance = distNow;

    if (stuckCounter > 6) {
      stopMotors();
      delay(80);

      backwardMove();
      delay(350);
      stopMotors();
      delay(80);

      if (millis() % 2 == 0) turnRight();
      else turnLeft();

      delay(TURN_90);
      stopMotors();
      delay(120);

      stuckCounter = 0;
      return;
    }
  }
}


//function for the servo 
void servoWriteMicroseconds(int us) {
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(us);
  digitalWrite(SERVO_PIN, LOW);
  delay(20); // servo refresh
}

void servoWrite(int angle) { //puts the angle to the right direction
  int pulse = map(angle, 0, 180, 1000, 2000);
  servoWriteMicroseconds(pulse);
}

//turns back to the front to check
void lookCenter() {
  for (int i = 0; i < 5; i++) servoWrite(50);
}

//turns the neck to the left to check
void lookLeft() {
  for (int i = 0; i < 5; i++) servoWrite(200);
}

//turns the neck to the right to check
void lookRight() {
  for (int i = 0; i < 5; i++) servoWrite(-100);
}

//scanning the right side
bool scanDirectionRight() {
  lookRight();
  float d = readDistance();
  lookCenter();
  return d > CLEAR_DIST;
}

//scanning the left side
bool scanDirectionLeft() {
  lookLeft();
  float d = readDistance();
  lookCenter();
  return d > CLEAR_DIST;
}

//functions for all the movements
void forwardMove() {
  analogWrite(LEFT_FWD, 147);
  analogWrite(RIGHT_FWD, FORWARD_SPEED);
  analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_BWD, 0);
}

void backwardMove() {
  analogWrite(LEFT_BWD, 147);
  analogWrite(RIGHT_BWD, 150);
  analogWrite(LEFT_FWD, 0);
  analogWrite(RIGHT_FWD, 0);
}

void stopMotors() {
  analogWrite(LEFT_FWD, 0);
  analogWrite(RIGHT_FWD, 0);
  analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_BWD, 0);
}

void turnRight() {
  analogWrite(LEFT_FWD, 167);
  analogWrite(RIGHT_BWD, TURN_SPEED);
  analogWrite(RIGHT_FWD, 0);
  analogWrite(LEFT_BWD, 0);
}

void turnLeft() {
  analogWrite(RIGHT_FWD, TURN_SPEED);
  analogWrite(LEFT_BWD, 167);
  analogWrite(LEFT_FWD, 0);
  analogWrite(RIGHT_BWD, 0);
}

//reading the distance with the ultrasonic sensor
float readDistance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long duration = pulseIn(ECHO, HIGH, 30000);
  if (duration == 0) return 0;

  return duration * 0.034 / 2.0;
}






