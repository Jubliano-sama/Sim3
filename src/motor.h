#pragma once

#include <FastAccelStepper.h>
#include <Arduino.h>
#include <Servo.h>

// Servo objects
Servo elbowServo;
Servo shoulderServo;
Servo gripServo;

void rotateShoulderAbsoluteAngle(float angle);
bool hasStepperReachedPosition();

void moveServo(Servo servo, int angle);
void moveElbowServo(int angle);
void moveShoulderServo(int angle);
void moveGripServo(int angle);
void zeroStepperPosition();