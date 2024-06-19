#pragma once
#include <Arduino.h>
#include "motor.h"

#define END_PAUSE_DRIVEFORWARD_MS 2000
#define BEGIN_PAUSE_DRIVEFORWARD_MS 2000

// The scanning function output will be offset by this amount, with respect to the scanning direction
#define SCANNING_OFFSET_ANGLE 0
#define SCANNING_OBJECT_ANGLE_TOLERANCE 6 // This value will be used to make sure we dont hit a object that has already been scanned
#define SCANNING_THRESHOLD_MM 9;

#define GRIP_CLOSING_ANGLE 125
#define GRIP_OPEN_ANGLE 40
#define SWITCHPIN 53

// TODO Change for actual angles
const ArmConfiguration scanningPosition(34, 163, 125, 36);
const ArmConfiguration carryingPosition(50, 163, 130);
const ArmConfiguration placingPosition(90, 90, 90);
const ArmConfiguration grabbingPosition(0, 123, 149);
const ArmConfiguration pushingObjectPosition(31, 153, 90, 115);
const ArmConfiguration pushingObjectPosition2(31, 136, 90, 115);
const ArmConfiguration homePosition(35, 140, 156);

const float driveSpeedMultiplier = 0.7;

const int secondsPerFullScan = 30;
const float Kp = 0.13;
const float Ki = 0.00001;
const float Kd = 0;
