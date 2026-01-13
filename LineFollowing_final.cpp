#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

int start = 0;

// ================= MOTOR PINS =================
const int LEFT_FWD  = 5;
const int LEFT_BWD  = 6;
const int RIGHT_FWD = 9;
const int RIGHT_BWD = 10;

// ================= LINE SENSORS =================
int sensorPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

// ================= ULTRASONIC =================
const int TRIG_PIN = 2;
const int ECHO_PIN = 3;

// ================= SPEED SETTINGS =================
const int BASE_SPEED = 255;
const int TURN_SPEED = 200;
const int LINE_TH    = 750;

// ================= OBSTACLE SETTINGS =================
const int OBSTACLE_DIST  = 20;
const int MIN_VALID_DIST = 3;
const int CONFIRM_COUNT  = 3;
const int SENSE_INTERVAL = 60;

// ================= AVOIDANCE TIMING =================
const unsigned long T0 = 400;
const unsigned long T1 = 800;
const unsigned long T2 = 600;

// ================= STATE =================
bool avoiding = false;
int avoidStage = 0;
unsigned long stageStartTime = 0;
unsigned long lastSenseTime = 0;
int obstacleCount = 0;
int lastTurn = 0;

#define GRIPPER 13
#define gripper_close 1000
#define gripper_open 1700


#define LED_PIN   4
#define LED_COUNT 4

#define FRONT_L 0
#define FRONT_R 1
#define BACK_L  2
#define BACK_R  3

Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// ================= LED HELPERS =================
void ledsOff() {
  pixels.clear();
  pixels.show();
}

void forwardLED() {
  pixels.clear();
  pixels.setPixelColor(FRONT_L, pixels.Color(0,255,0));
  pixels.setPixelColor(FRONT_R, pixels.Color(0,255,0));
  pixels.show();
}

void backwardLED() {
  pixels.clear();
  pixels.setPixelColor(BACK_L, pixels.Color(255,0,0));
  pixels.setPixelColor(BACK_R, pixels.Color(255,0,0));
  pixels.show();
}

void turnBlinkGreen() {
  static unsigned long lastBlink = 0;
  static bool state = false;

  if (millis() - lastBlink > 200) {
    lastBlink = millis();
    state = !state;
    pixels.clear();
    if (state) {
      for (int i = 0; i < LED_COUNT; i++)
        pixels.setPixelColor(i, pixels.Color(0,255,0));
    }
    pixels.show();
  }
}

void stopMotors() {
  ledsOff();
  analogWrite(LEFT_FWD, 0);
  analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_FWD, 0);
  analogWrite(RIGHT_BWD, 0);
}

void forward(int spd) {
  forwardLED();
  analogWrite(LEFT_FWD, spd);
  analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_FWD, spd);
  analogWrite(RIGHT_BWD, 0);
}

void backward(int spd){
  backwardLED();
  analogWrite(LEFT_BWD, spd);
  analogWrite(RIGHT_BWD, spd);
  analogWrite(LEFT_FWD, 0);
  analogWrite(RIGHT_FWD, 0);
}

void leftFwd_rightBwd() {
  turnBlinkGreen();
  analogWrite(LEFT_FWD, TURN_SPEED);
  analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_FWD, 0);
  analogWrite(RIGHT_BWD, TURN_SPEED);
}

void rightFwd_leftBwd() {
  turnBlinkGreen();
  analogWrite(LEFT_FWD, 0);
  analogWrite(LEFT_BWD, TURN_SPEED);
  analogWrite(RIGHT_FWD, TURN_SPEED);
  analogWrite(RIGHT_BWD, 0);
}


long getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return 999;
  return duration * 0.034 / 2;
}


bool lineDetected() {
  for (int i = 0; i < 8; i++)
    if (analogRead(sensorPins[i]) > LINE_TH) return true;
  return false;
}

void lineFollow() {
  bool leftSeen = false;
  bool rightSeen = false;

  for (int i = 0; i < 8; i++) {
    int v = analogRead(sensorPins[i]);
    if (i <= 3 && v > LINE_TH) leftSeen = true;
    if (i >= 4 && v > LINE_TH) rightSeen = true;
  }

  if (leftSeen && rightSeen) {
    lastTurn = 0;
    forward(BASE_SPEED);
  } else if (rightSeen) {
    lastTurn = -1;
    rightFwd_leftBwd();
  } else if (leftSeen) {
    lastTurn = 1;
    leftFwd_rightBwd();
  } else {
    if (lastTurn == -1) rightFwd_leftBwd();
    else if (lastTurn == 1) leftFwd_rightBwd();
    else forward(120);
  }
}


void gripper(int pulse){
  static long last_pulse;
  static long timer;
  if (millis() > timer){
    if(pulse > 0) last_pulse = pulse;
    digitalWrite(GRIPPER, 1);
    delayMicroseconds(last_pulse);
    digitalWrite(GRIPPER, 0);
    timer = millis() + 20;
  }
}


void startingSequence() {

  while (true) {
    long d = getDistance();
    if (d > 10 || d == 999) break;
    stopMotors();
    delay(30);
  }

  unsigned long t = millis();
  while (millis() - t < 1000) gripper(gripper_open);

  unsigned long startTime = millis();
  while (millis() - startTime < 1200) forward(BASE_SPEED);

  stopMotors();

  t = millis();
  while (millis() - t < 1000) gripper(gripper_close);

  unsigned long rot = millis();
  while (millis() - rot < 900) rightFwd_leftBwd();

  stopMotors();

  while (!lineDetected()) forward(BASE_SPEED);
  stopMotors();
}


bool endingSequence() {

  bool allBlack = true;

  for (int i = 0; i < 8; i++) {
    if (analogRead(sensorPins[i]) < 900) {
      allBlack = false;
      break;
    }
  }

  if (allBlack) {
    unsigned long t = millis();
    while (millis() - t < 100) forward(BASE_SPEED);

    for (int i = 0; i < 8; i++) {
      if (analogRead(sensorPins[i]) < 900) return false;
    }

    stopMotors();

    t = millis();
    while (millis() - t < 1000) gripper(gripper_open);

    t = millis();
    while (millis() - t < 800) backward(BASE_SPEED);

    stopMotors();

    while (true); // END PROGRAM
  }

  return false;
}


void setup() {
  for (int i = 0; i < 8; i++) pinMode(sensorPins[i], INPUT);

  pinMode(LEFT_FWD, OUTPUT);
  pinMode(LEFT_BWD, OUTPUT);
  pinMode(RIGHT_FWD, OUTPUT);
  pinMode(RIGHT_BWD, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(GRIPPER, OUTPUT);

  pixels.begin();
  pixels.clear();
  pixels.show();
}

// ================= LOOP =================
void loop() {

  if (start == 0) {
    start++;
    startingSequence();
  }

  if (millis() - lastSenseTime >= SENSE_INTERVAL) {
    long d = getDistance();
    lastSenseTime = millis();
    if (d >= MIN_VALID_DIST && d < OBSTACLE_DIST) obstacleCount++;
    else obstacleCount = 0;
  }

  if (!avoiding && obstacleCount >= CONFIRM_COUNT) {
    avoiding = true;
    avoidStage = 0;
    stageStartTime = millis();
    obstacleCount = 0;
    stopMotors();
    return;
  }

  if (avoiding) {
    if (avoidStage == 0) {
      rightFwd_leftBwd();
      if (millis() - stageStartTime >= T0) {
        avoidStage = 1;
        stageStartTime = millis();
      }
      return;
    }
    if (avoidStage == 1) {
      forward(BASE_SPEED);
      if (millis() - stageStartTime >= T1) {
        avoidStage = 2;
        stageStartTime = millis();
      }
      return;
    }
    if (avoidStage == 2) {
      leftFwd_rightBwd();
      if (millis() - stageStartTime >= T2) {
        avoidStage = 3;
      }
      return;
    }
    if (avoidStage == 3) {
      forward(BASE_SPEED);
      if (lineDetected()) avoiding = false;
      return;
    }
  }

  lineFollow();
  endingSequence();
}
