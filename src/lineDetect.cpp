#include "lineDetect.h"
#include <Arduino.h>

void initLineSensors()
{
  for (int i = 0; i < LINE_SENSOR_COUNT; i++)
  {
    pinMode(LINE_SENSOR_START + i, INPUT);
  }
}
// int values vary from 0 to 1023
// index
byte readLineSensor(int index)
{
  // get data of the indexed line sensor
  return analogRead(LINE_SENSOR_START + index);
}

bool isValueOnLine(byte value)
{
  // threshold for line detection
  const byte ON_LINE_THRESHOLD = 900;
  // TODO solve jittering values with hysteresis if needed
  return value > ON_LINE_THRESHOLD;
}
