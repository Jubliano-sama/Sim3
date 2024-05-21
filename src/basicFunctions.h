#pragma once

// 0 = No Debug, 1 = Only special cases(pause,stop,overshoot), 2 = everyhting
#define DEBUG 2

const double Kp = 0.12;
const double Ki = 0.000002;
const double Kd = -2;
const int switchPin = 53;

const int analogSensors[] = {A0, A1, A2, A3, A5, A4, A7, A8, A9}; // Update with actual analog sensor pins
const int analogSensorsCount = 9; // Update with the actual number of analog sensors
const int analogSensorThresholds[] = {150, 150, 150, 150, 150, 150, 150, 150, 150}; // Update with actual threshold values

// Add array index to this to disable them as input
const int disabledSensors[] = {};
const int disabledSensorsCount = 0;

const int motorsForwardPins[]={51,49,47,45}; //{lv,la,rv,ra}
const int motorsBackwardPins[]={50,48,46,44};
const int motorsENAPins[]={6,3,4,5};
const int stopPauseDelay = 700;

const int displayBarsRefreshTime = 100;

extern int lastDirection;

void setupBasicFunctions();
void driveMotors(double left, double right);
double mapDouble(double x, double inMin, double inMax, double outMin, double outMax);
bool* getAnalogSensorValues();