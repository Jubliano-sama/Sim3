#pragma once
#include <Arduino.h>
// 0 = No Debug, 1 = Only special cases(pause,stop,overshoot), 2 = everyhting
#define DEBUG 2

const double Kp = 0.15;
const double Ki = 0.00000;
const double Kd = 0;
const int switchPin = 53;

const int analogSensors[] = { A9, A8, A7, A6, A5, A4, A3}; // Update with actual analog sensor pins
const int analogSensorsCount = 7; // Update with the actual number of analog sensors
const int analogSensorThresholds[] = {250, 200, 150, 250, 250, 150, 200}; // Update with actual threshold values

// Add array index to this to disable them as input
const int disabledSensors[] = {};
const int disabledSensorsCount = 0;

const int motorsForwardPins[]={51,48,45,34}; //{lv,la,rv,ra}
const int motorsBackwardPins[]={50,49,44,38};
const int motorsENAPins[]={3,4,2,5};
const int stopPauseDelay = 700;
const int beginPauseDriveForwardTime = 500;

extern int lastDirection;

namespace car {
    void setupCarHardware();
    void driveMotors(double left, double right);
    double mapDouble(double x, double inMin, double inMax, double outMin, double outMax);
    bool* getAnalogSensorValues();
}
