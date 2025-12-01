#include "lineDetect.h"
#include <Arduino.h>

void initLineSensors()
{
  for (int i = 0; i < LINE_SENSOR_COUNT; i++)
  {
    // Assuming LINE_SENSOR_START + i maps to A0, A1, etc.
    pinMode(LINE_SENSOR_START + i, INPUT);
  }
}

// FIXED: Changed return type from byte to int
int readLineSensor(int index)
{
  // analogRead returns 0-1023. 'byte' (0-255) would overflow!
  return analogRead(LINE_SENSOR_START + index);
}

// FIXED: Changed parameter type from byte to int
bool isValueOnLine(int value)
{
  // Threshold for line detection (adjust based on your surface)
  // Black line on white surface usually reads HIGH (close to 1023)
  // White line on black surface usually reads LOW (close to 0)
  const int ON_LINE_THRESHOLD = 700; 

  return value > ON_LINE_THRESHOLD;
}
