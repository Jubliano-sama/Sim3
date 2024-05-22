#pragma once

// 0 = No Debug, 1 = Only special cases(pause,stop,overshoot), 2 = everyhting
#define DEBUG 2

const double Kp = 0.15;
const double Ki = 0.00000;
const double Kd = 0;
const int switchPin = 53;

const int analogSensors[] = { A0,A1, A2, A3, A5, A4, A7, A8, A9}; // Update with actual analog sensor pins
const int analogSensorsCount = 9; // Update with the actual number of analog sensors
const int analogSensorThresholds[] = {50,60, 40, 120, 120, 300, 70, 100,50}; // Update with actual threshold values

// Add array index to this to disable them as input
const int disabledSensors[] = {};
const int disabledSensorsCount = 0;

const int motorsBackwardPins[]={51,49,46,44}; //{lv,la,rv,ra}
const int motorsForwardPins[]={50,48,47,45};
const int motorsENAPins[]={3,4,6,5};
const int stopPauseDelay = 700;


extern int lastDirection;

void setupBasicFunctions();
void driveMotors(double left, double right);
double mapDouble(double x, double inMin, double inMax, double outMin, double outMax);
bool* getAnalogSensorValues();