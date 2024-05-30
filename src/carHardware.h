#pragma once
#include <Arduino.h>
// 0 = No Debug, 1 = Only special cases(pause,stop,overshoot), 2 = everyhting
#define DEBUG 2

const double Kp = 0.07;
const double Ki = 0.000015;
const double Kd = 0;
const int switchPin = 53;

const int analogSensors[] = { A9, A8, A7, A6, A5, A4, A3}; // Update with actual analog sensor pins
const int analogSensorsCount = 7; // Update with the actual number of analog sensors
const int analogSensorThresholds[] = {100, 100, 100, 100, 100, 100, 100}; // Update with actual threshold values

// Add array index to this to disable them as input
const int disabledSensors[] = {};
const int disabledSensorsCount = 0;

const int motorsBackwardPins[]={51,49,46,44}; //{lv,la,rv,ra}
const int motorsForwardPins[]={50,48,47,45};
const int motorsENAPins[]={3,4,6,5};
const int stopPauseDelay = 700;
const int beginPauseDriveForwardTime = 500;

extern int lastDirection;

namespace car {
    void setupCarHardware();
    void driveMotors(double left, double right);
    double mapDouble(double x, double inMin, double inMax, double outMin, double outMax);
    bool* getAnalogSensorValues();
}
