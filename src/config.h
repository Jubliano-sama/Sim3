#pragma once

#include <Arduino.h>

// Define motor interface type and pins used for the A4988 driver
#define motorInterfaceType 1

#define dirPin A0 
#define stepPin 6 // 6 7 or 8!!
const int stepperAcceleration = 1000;
const int stepperMaxSpeed = 500;
const float stepperGearRatio = 2.5; // x rotations per shoulder rotation
const int stepsPerRotation = 200;
const float shoulderRotationSteps = (float)stepsPerRotation * stepperGearRatio;

// Servo control pins MAKE SURE THESE ARE PWM PINS
#define ELBOW_SERVO_PIN 9
#define SHOULDER_SERVO_PIN 10
#define GRIP_SERVO_PIN 11