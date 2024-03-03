#include "motor.h"
#include "config.h"
#include <AccelStepper.h>
#include <Arduino.h>
#include <Servo.h>

// Initialize AccelStepper for the stepper motor
AccelStepper stepper(motorInterfaceType, stepPin, dirPin);

// Servo objects
Servo elbowServo;
Servo shoulderServo;
Servo gripServo;

// Enumeration for servo selection
enum ServoType {
    ELBOW,
    SHOULDER,
    GRIP
};

void setupMotors() {
    // Stepper motor setup
    stepper.setMaxSpeed(stepperMaxSpeed);
    stepper.setAcceleration(stepperAcceleration);

    // Servo setup
    elbowServo.attach(ELBOW_SERVO_PIN);
    shoulderServo.attach(SHOULDER_SERVO_PIN);
    gripServo.attach(GRIP_SERVO_PIN);
}

void moveStepper(int steps) {
    stepper.moveTo(steps);
    stepper.runToPosition();
}

void stopShoulder(){
    stepper.stop();
}

//assume we start at angle 0
void rotateShoulderAbsoluteAngle(float angle) {
    const float currentAngle = positionToAngle(stepper.currentPosition());
    stepper.move(angleToSteps(angle - currentAngle));
}

float positionToAngle(int position){
    float angle = (static_cast<float>(position) / static_cast<float>(stepsPerRotation) / stepperGearRatio) * 360.0f;
    angle = fmod(angle, 360.0f); // Use fmod for floating-point modulo operation
    if (angle < 0) { // Ensure the angle is positive
        angle += 360.0f;
    }
    return angle;
}

int angleToSteps(float angle) {
    // Normalize angle to the range [0, 360)
    angle = fmod(angle, 360.0f);
    if (angle < 0) angle += 360.0f;

    float shoulderRotationSteps = stepsPerRotation * stepperGearRatio;

    // Calculate steps for the given angle considering gear ratio
    float steps = (angle / 360.0f) * shoulderRotationSteps;

    // Determine the shortest path (clockwise or counter-clockwise)
    float stepsNormalized = fmod(steps, shoulderRotationSteps);
    if (stepsNormalized > shoulderRotationSteps / 2) {
        // If going clockwise is longer, go counter-clockwise
        stepsNormalized -= shoulderRotationSteps;
    }

    return static_cast<int>(round(stepsNormalized));
}

void moveServo(ServoType servo, int angle) {
    switch (servo) {
        case ELBOW:
            elbowServo.write(angle);
            break;
        case SHOULDER:
            shoulderServo.write(angle);
            break;
        case GRIP:
            gripServo.write(angle);
            break;
    }
}

void loopMotor() {
    // Example stepper motor usage
    moveStepper(200);
    delay(1000);
    moveStepper(-200);
    delay(1000);

    // Example servo movements
    moveServo(ELBOW, 90); // Move elbow servo to 90 degrees
    delay(1000);
    moveServo(SHOULDER, 45); // Move shoulder servo to 45 degrees
    delay(1000);
    moveServo(GRIP, 10); // Close grip slightly (10 degrees)
    delay(1000);
}