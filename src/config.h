#pragma once
#include <Arduino.h>
#include "motor.h"

// TODO SET THIS PIN
#define OBJECT_DETECTION_PIN 1

// The scanning function output will be offset by this amount, with respect to the scanning direction
#define SCANNING_OFFSET_ANGLE 15
#define SCANNING_TOLERANCE 6 // This value will be used to make sure we dont hit a object that has already been scanned
#define GRIP_CLOSING_ANGLE 60
#define GRIP_OPEN_ANGLE 135

// TODO Change for actual angles
const ArmConfiguration scanningPosition(55, 160, 90);
const ArmConfiguration carryingPosition(90, 90, 90);
const ArmConfiguration placingPosition(90, 90, 90);
const ArmConfiguration grabbingPosition(90, 90, 90);
const ArmConfiguration pushingObjectPosition(90, 90, 90);
const ArmConfiguration homePosition(35, 140, 156);

const int secondsPerFullScan = 20;