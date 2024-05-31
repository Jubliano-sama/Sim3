#pragma once
#include <Arduino.h>

// TODO SET THIS PIN
#define OBJECT_DETECTION_PIN 1

// The scanning function output will be offset by this amount, with respect to the scanning direction
#define SCANNING_OFFSET_ANGLE 15
#define GRIP_CLOSING_ANGLE 60
#define GRIP_OPEN_ANGLE 135

struct ArmConfiguration {
    int shoulderAngle;
    int elbowAngle;
    int wristAngle;

    ArmConfiguration(float shoulder, float elbow, float wrist) : shoulderAngle(shoulder), elbowAngle(elbow), wristAngle(wrist) {}
};



// TODO Change for actual angles
const ArmConfiguration scanningPosition(90, 90, 90);
const ArmConfiguration carryingPosition(90, 90, 90);
const ArmConfiguration placingPosition(90, 90, 90);
const ArmConfiguration grabbingPosition(90, 90, 90);
const ArmConfiguration pushingObjectPosition(90, 90, 90);

