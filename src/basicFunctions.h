#pragma once
#define DEBUG 1
const double Kp = 0.05;
const double Ki = 0;
const double Kd = 0;

const int switchPin = 53;
const int IRSensors[] ={22,23,24,25,26};
const int IRSensorsCount = 5;
const int frontIRSensorPin=52;
const int startSwitch=53;

const int motorsFowardPins[]={51,49,47,45};
const int motorsBackwardPins[]={50,48,46,44};
const int motorsENAPins[]={2,3,4,5};

void setupBasicFunctions();
bool readFrontIRSensor();
void driveMotors(double left, double right);
bool * getSensorValues();
