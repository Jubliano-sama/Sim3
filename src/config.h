#pragma once

#include <Arduino.h>


// Define motor interface type and pins used for the A4988 driver
#define motorInterfaceType 1
#define dirPin A0
#define stepPin 8 // 6 7 or 8!!

// TODO SET THIS PIN
const int objectDetectionPin = 1;

#define ELBOW_SERVO_PIN 9
#define SHOULDER_SERVO_PIN 10
#define GRIP_SERVO_PIN A5
#define WRIST_SERVO_PIN 12

const int stepperAcceleration = 400;
const int stepperMaxSpeed = 5000;
const float stepperGearRatio = 1.0f+(38.0f/14.0f); // x rotations per shoulder rotation
const int stepsPerRotation = 200;
const float shoulderRotationSteps = (float)stepsPerRotation * stepperGearRatio;
const int scanningSpeed = shoulderRotationSteps / 10; // Scanning takes 10 seconds

struct ArmConfiguration {
    int shoulderAngle;
    int elbowAngle;
    int wristAngle;

    ArmConfiguration(float shoulder, float elbow, float wrist) : shoulderAngle(shoulder), elbowAngle(elbow), wristAngle(wrist) {}
};

const int gripClosingAngle = 110;
const int gripOpenAngle = 30;

// TODO Change for actual angles
const ArmConfiguration scanningPosition(90, 90, 90);
const ArmConfiguration carryingPosition(90, 90, 90);
const ArmConfiguration placingPosition(90, 90, 90);
const ArmConfiguration grabbingPosition(90, 90, 90);

// field values going clockwise from starting position
const int fieldValues[] = {0, -1, 2, -9, 12, -15, 20, -21, 14, -17, 6, -7, 0, -3, 4, -11, 10, -13, 18, -22, 16, -19, 8, -5};
const int amountOfFieldValues = sizeof(fieldValues)/sizeof(int);

