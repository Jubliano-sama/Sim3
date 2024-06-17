#pragma once
#include <FastAccelStepper.h>
#include <Arduino.h>
#include <Servo.h>

// Define motor interface type and pins used for the A4988 driver
#define DIR_PIN A0
#define STEP_PIN 8 // 6 7 or 8!!

#define ELBOW_SERVO_PIN 12
#define SHOULDER_SERVO_PIN 10
#define GRIP_SERVO_PIN 11
#define WRIST_SERVO_PIN 9

const int stepperAcceleration = 400;
const int stepperMaxSpeed = 5000;
const float stepperGearRatio = 1.0f + (38.0f / 14.0f); // x rotations per shoulder rotation
const int stepsPerRotation = 200;
const float shoulderRotationSteps = (float)stepsPerRotation * stepperGearRatio;

struct ArmConfiguration
{
    float shoulderAngle;
    float elbowAngle;
    float wristAngle;
    float gripAngle;

    // Constructor with default argument for gripAngle
    ArmConfiguration(float shoulder, float elbow, float wrist, float grip = -1)
        : shoulderAngle(shoulder), elbowAngle(elbow), wristAngle(wrist), gripAngle(grip) {}
};

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