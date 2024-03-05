#pragma once

#include <FastAccelStepper.h>
#include <Arduino.h>
#include <PWMServo.h>

// Servo objects
PWMServo elbowServo;
PWMServo shoulderServo;
PWMServo gripServo;

void rotateShoulderAbsoluteAngle(float angle);
bool hasStepperReachedPosition();
void zeroStepperPosition();
void stopShoulder();

void moveServo(PWMServo, int angle);
void moveElbowServo(int angle);
void moveShoulderServo(int angle);
void moveGripServo(int angle);