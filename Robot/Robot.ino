#include <Arduino.h>

#define TRIG 12
#define ECHO 13

const float DETECT_DIST = 10.0; // cm (same wall distance logic)

float readDistance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long duration = pulseIn(ECHO, HIGH, 18000);
  if (duration == 0) return 0;
  return duration * 0.034 / 2.0;
}

void setup() {
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  Serial.begin(9600); // HC-12
}

void loop() {
  float distance = readDistance();

  if (distance > 0 && distance < DETECT_DIST) {
    Serial.println("OBJECT FOUND!!!");
    delay(500); // prevent spam
  }
}
