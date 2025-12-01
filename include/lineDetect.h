#ifndef LINEDETECT_H
#define LINEDETECT_H

#include <Arduino.h>

// Change these to match your actual wiring!
#define LINE_SENSOR_START A0  // The first sensor pin
#define LINE_SENSOR_COUNT 8   // Total number of sensors

void initLineSensors();
int readLineSensor(int index); // Prototype must match 'int'
bool isValueOnLine(int value); // Prototype must match 'int'

#endif
