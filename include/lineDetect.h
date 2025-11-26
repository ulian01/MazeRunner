// a0 to a7
// 8 data points for line detection

#pragma once

#include <Arduino.h>

#define LINE_SENSOR_COUNT 8
#define LINE_SENSOR_START A0

void initLineSensors();

// int values vary from 0 to 1023
byte readLineSensor(int index);

bool isValueOnLine(byte value);