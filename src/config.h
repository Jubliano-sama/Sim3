#pragma once
#include <Arduino.h>
#include "motor.h"

#define END_PAUSE_DRIVEFORWARD_MS 500
#define BEGIN_PAUSE_DRIVEFORWARD_MS 1300
#define STOP_DRIVE_FORWARD_MS 2000

// The scanning function output will be offset by this amount, with respect to the scanning direction
#define SCANNING_OFFSET_ANGLE -2.5
#define SCANNING_OBJECT_ANGLE_TOLERANCE 18 // This value will be used to make sure we dont hit a object that has already been scanned
#define SCANNING_THRESHOLD_MM 5.5;

#define GRIP_CLOSING_ANGLE 125
#define GRIP_OPEN_ANGLE 70
#define SWITCHPIN 53

// TODO Change for actual angles
const ArmConfiguration scanningPosition(56, 178, 122, 33);
const ArmConfiguration carryingPosition(90, 173, 140);
const ArmConfiguration placingPosition(1, 123, 173);
const ArmConfiguration grabbingPosition(2, 123, 180);
const ArmConfiguration pushingObjectPosition(46, 177, 90, 115);
const ArmConfiguration pushingObjectPosition2(46, 150, 90, 115);
const ArmConfiguration homePosition(35, 140, 156);

const float driveSpeedMultiplier = 0.7;

const int secondsPerFullScan = 30;
const float Kp = 0.11;
const float Ki = 0.000013;
const float Kd = 0;
