#include <Arduino.h>

// Define motor interface type and pins used for the A4988 driver
#define motorInterfaceType 1
#define dirPin A0 // Direction pin
#define stepPin A1 // Step pin
const int stepperAcceleration = 1000;
const int stepperMaxSpeed = 500;
const float stepperGearRatio = 2.5; // x rotations per shoulder rotation
const int stepsPerRotation = 200;

// Servo control pins (adjust these to your setup)
#define ELBOW_SERVO_PIN 9
#define SHOULDER_SERVO_PIN 10
#define GRIP_SERVO_PIN 11

