#pragma once
#include <FastAccelStepper.h>
#include <Arduino.h>
#include <Servo.h>
#include "config.h"

void rotateShoulderAbsoluteAngle(float angle);
bool hasStepperReachedPosition();
void zeroStepperPosition();
void stopShoulder();
void setupMotors();
void testMotors();
void rotateShoulderRelativeAngle(float angle);
float getShoulderAngle();
void setStepperSpeed(int stepsPerSecond);

void moveWristServo(int angle);
void moveServo(Servo, int angle);
void moveElbowServo(int angle);
void moveShoulderServo(int angle);
void moveGripServo(int angle);
void moveToArmConfiguration(ArmConfiguration configuration);