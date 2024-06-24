#pragma once
#include <Arduino.h>
#include "motor.h"

#define END_PAUSE_DRIVEFORWARD_MS 500
#define BEGIN_PAUSE_DRIVEFORWARD_MS 1200
#define STOP_DRIVE_FORWARD_MS 2000

// The scanning function output will be offset by this amount, with respect to the scanning direction
#define SCANNING_OFFSET_ANGLE -6
#define SCANNING_OBJECT_ANGLE_TOLERANCE 18 // This value will be used to make sure we dont hit a object that has already been scanned
#define SCANNING_THRESHOLD_MM 5.5
#define SCANNING_ROLLING_AVERAGE_ANGLE 3 // the angle over which the rolling average is taken
#define SCANNING_POSITIONS_ARRAY_SIZE 155 // This is the resolution the scan is broken into before calculating the object positions
#define SCANNING_BEGIN_END_ANGLE 32 // The scan will start and end at 0 degrees +- this value

#define GRIP_CLOSING_ANGLE 125
#define GRIP_OPEN_ANGLE 65
#define SWITCHPIN 53

// TODO Change for actual angles
const ArmConfiguration scanningPosition(59, 176, 90, 32);
const ArmConfiguration scanningPosition2(46, 167, 123, 32);
const ArmConfiguration carryingPosition(90, 173, 140);
const ArmConfiguration placingPosition(44, 151, 95);
const ArmConfiguration grabbingPosition(0, 121, 180);
const ArmConfiguration pushingObjectPosition(46, 177, 90, 110);
const ArmConfiguration pushingObjectPosition2(46, 150, 90, 110);
const ArmConfiguration homePosition(35, 140, 156);

const float driveSpeedMultiplier = 0.7;

const int secondsPerFullScan = 20;


const float Kp = 0.11;
const float Ki = 0.000013;
const float Kd = 0;
