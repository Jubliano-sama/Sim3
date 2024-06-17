#pragma once
#include <Arduino.h>
#include "motor.h"

#define END_PAUSE_DRIVEFORWARD_MS 1500
#define BEGIN_PAUSE_DRIVEFORWARD_MS 1000

// The scanning function output will be offset by this amount, with respect to the scanning direction
#define SCANNING_OFFSET_ANGLE 15
#define SCANNING_OBJECT_ANGLE_TOLERANCE 6 // This value will be used to make sure we dont hit a object that has already been scanned
#define SCANNING_THRESHOLD_MM 5;

#define GRIP_CLOSING_ANGLE 50
#define GRIP_OPEN_ANGLE 135

// TODO Change for actual angles
const ArmConfiguration scanningPosition(55, 160, 90, 160);
const ArmConfiguration carryingPosition(90, 90, 90);
const ArmConfiguration placingPosition(90, 90, 90);
const ArmConfiguration grabbingPosition(90, 90, 90);
const ArmConfiguration pushingObjectPosition(90, 90, 90);
const ArmConfiguration homePosition(35, 140, 156);

const float driveSpeedMultiplier = 0.75;

const int secondsPerFullScan = 20;