#pragma once

// 0 = No Debug, 1 = Only special cases(pause,stop,overshoot), 2 = everyhting
#define DEBUG 2

const double Kp = 0.03;
const double Ki = 0.0;
const double Kd = 0;

const int switchPin = 53;
const int IRSensors[] ={22,23,24,25,26,27,28,29,30,31,32,33,34,35,36};
const int IRSensorsCount = 7;
const int frontIRSensorPin=52;

const int analogSensors[] = {A0, A1, A2}; // Update with actual analog sensor pins
const int analogSensorsCount = 3; // Update with the actual number of analog sensors
const int analogSensorThresholds[] = {500, 500, 500}; // Update with actual threshold values

// Add array index to this to disable them as input
const int disabledSensors[] = {};
const int disabledSensorsCount = 0;

const int motorsBackwardPins[]={43,49,47,45}; //{lv,la,rv,ra}
const int motorsForwardPins[]={50,48,46,44};
const int motorsENAPins[]={6,3,4,5};
const int stopPauseDelay = 500;

extern bool lastDirection;

void setupBasicFunctions();
bool readFrontIRSensor();
void driveMotors(double left, double right);
double mapDouble(double x, double inMin, double inMax, double outMin, double outMax);
bool * getSensorValues();
bool* getAnalogSensorValues();