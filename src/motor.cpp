#include "motor.h"

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

Servo shoulderServo;
Servo elbowServo;
Servo wristServo;
Servo gripServo;

int currentElbowAngle = 90;
int currentShoulderAngle = 90;
int currentWristAngle = 90;
int currentGripAngle = 90;

void setupMotors();
void moveStepper(int steps);
void stopShoulder();
void rotateShoulderAbsoluteAngle(float angle);
float positionToAngle(int position);
int angleToSteps(float angle);
void testMotors();
void moveWristServo(int angle);
void moveToArmConfiguration(ArmConfiguration configuration);

void setupMotors()
{
	engine.init();
	stepper = engine.stepperConnectToPin(STEP_PIN);
	if (stepper)
	{
		stepper->setDirectionPin(DIR_PIN);
		stepper->setAcceleration(stepperAcceleration);
		stepper->setSpeedInHz(stepperMaxSpeed);
	}

	// Servo setup
	elbowServo.attach(ELBOW_SERVO_PIN);
	shoulderServo.attach(SHOULDER_SERVO_PIN);
	gripServo.attach(GRIP_SERVO_PIN);
	wristServo.attach(WRIST_SERVO_PIN);
}

void moveStepper(int steps)
{
	Serial.print("Moving stepper ");
	Serial.print(steps);
	Serial.println(" steps.");
	if((stepper->getCurrentPosition() + steps) > maxRotationsOneDirection * shoulderRotationSteps){
		stepper->move(steps-shoulderRotationSteps);
		return;
	} else if(((stepper->getCurrentPosition() + steps) < -maxRotationsOneDirection * shoulderRotationSteps)) {
		stepper->move(steps+shoulderRotationSteps);
		return;
	}
	stepper->move(steps);
}

void stopShoulder()
{
	Serial.println("Stopping stepper motor immediately.");
	stepper->stopMove();
}

float getShoulderAngle()
{
	return positionToAngle(stepper->getCurrentPosition());
}

void rotateShoulderRelativeAngle(float angle)
{
	Serial.print("Rotating shoulder to relative angle: ");
	Serial.println(angle);
	moveStepper(static_cast<int>((angle / 360.0f) * shoulderRotationSteps));
}
// assume we start at angle 0
void rotateShoulderAbsoluteAngle(float angle)
{
	Serial.print("Rotating shoulder to absolute angle: ");
	Serial.println(angle);
	const float currentAngle = positionToAngle(stepper->getCurrentPosition());
	moveStepper(angleToSteps(angle - currentAngle));
}

void zeroStepperPosition()
{
	// Reset the stepper motor position to zero
	stepper->setCurrentPosition(0);
	Serial.println("Stepper position reset to 0.");
}

bool hasStepperReachedPosition()
{
	return !stepper->isRunning();
}

// function assumes stepper starts at 0 steps at degree 0
float positionToAngle(int position)
{
	float angle = (static_cast<float>(position) / shoulderRotationSteps) * 360.0f;
	angle = fmod(angle, 360.0f);
	if (angle < 0)
	{ // Ensure the angle is positive
		angle += 360.0f;
	}
	/*
	Serial.print("Converted position ");
	Serial.print(position);
	Serial.print(" to angle: ");
	Serial.println(angle);*/
	return angle;
}

int angleToSteps(float angle)
{
	// Normalize angle to the range [0, 360)
	angle = fmod(angle, 360.0f);
	if (angle < 0)
		angle += 360.0f;

	// Calculate steps for the given angle considering gear ratio
	float steps = (angle / 360.0f) * shoulderRotationSteps;

	// Determine the shortest path (clockwise or counter-clockwise)
	float stepsNormalized = fmod(steps, shoulderRotationSteps);
	if (stepsNormalized > shoulderRotationSteps / 2)
	{
		// If going clockwise is longer, go counter-clockwise
		stepsNormalized -= shoulderRotationSteps;
	}

	Serial.print("Converted angle ");
	Serial.print(angle);
	Serial.print(" to steps: ");
	Serial.println(stepsNormalized);

	return static_cast<int>(round(stepsNormalized));
}

void moveServo(Servo servo, int angle)
{
	servo.write(angle);
}

// Directly control each servo without using enum
void moveElbowServo(int angle)
{
	Serial.print("Moving elbow servo to angle: ");
	Serial.println(angle);
	currentElbowAngle = angle;
	moveServo(elbowServo, angle);
}

void moveShoulderServo(int angle)
{
	Serial.print("Moving shoulder servo to angle: ");
	Serial.println(angle);
	currentShoulderAngle = angle;
	moveServo(shoulderServo, angle);
}

void moveGripServo(int angle)
{
	Serial.print("Moving grip servo to angle: ");
	Serial.println(angle);
	currentGripAngle = angle;
	moveServo(gripServo, angle);
}

void moveWristServo(int angle)
{
	Serial.print("Moving wrist servo to angle: ");
	Serial.println(angle);
	currentWristAngle = angle;
	moveServo(wristServo, angle);
}

void moveToArmConfiguration(ArmConfiguration configuration)
{
	moveShoulderServo(configuration.shoulderAngle);
	moveElbowServo(configuration.elbowAngle);
	moveWristServo(configuration.wristAngle);
	if (configuration.gripAngle > 0) moveGripServo(configuration.gripAngle);
}

// Watch out, this function is blocking
void interpolateToArmConfiguration(ArmConfiguration configuration, unsigned long delayTime){
	int iterations = delayTime/10;

	int beginningElbowAngle = currentElbowAngle;
	int beginningShoulderAngle = currentShoulderAngle;
	int beginningWristAngle = currentWristAngle;
	int beginningGripAngle = currentGripAngle;

	for (int i = 0; i <= iterations; i++)
    {
        // Interpolate between pushing position and grabbing position
		if(configuration.elbowAngle>0.0f){
			moveElbowServo(beginningElbowAngle - (beginningElbowAngle - configuration.elbowAngle) * i / iterations);
		}
		if(configuration.shoulderAngle>-0.1f){
			moveShoulderServo(beginningShoulderAngle - (beginningShoulderAngle - configuration.shoulderAngle) * i / iterations);
		}
		if(configuration.wristAngle>0.0f){
			moveWristServo(beginningWristAngle - (beginningWristAngle - configuration.wristAngle) * i / iterations);
		}
		if(configuration.gripAngle>0.0f){
			moveGripServo(beginningGripAngle - (beginningGripAngle - configuration.gripAngle) * i / iterations);
		}
		delay(10);
    }
}

void setStepperSpeed(int stepsPerSecond)
{
	stepper->setSpeedInHz(stepsPerSecond);
}

void testMotors()
{
	// Example stepper motor usage
	moveStepper(200);
	while (!hasStepperReachedPosition())
		;
	delay(1000);
	moveStepper(-200);
	while (!hasStepperReachedPosition())
		;
	delay(1000);

	// Example servo movements
	moveServo(elbowServo, 80); // Move elbow servo to 90 degrees
	delay(1000);
	moveServo(elbowServo, 100); // Move elbow servo to 90 degrees
	delay(1000);
	moveServo(shoulderServo, 80); // Move shoulder servo to 45 degrees
	delay(1000);
	moveServo(shoulderServo, 100); // Move shoulder servo to 45 degrees
	delay(1000);
	moveServo(wristServo, 80); // Move shoulder servo to 45 degrees
	delay(1000);
	moveServo(wristServo, 100); // Move shoulder servo to 45 degrees
	delay(1000);
	moveServo(gripServo, 80); // Close grip slightly (10 degrees)
	delay(1000);
	moveServo(gripServo, 100); // Close grip slightly (10 degrees)
	delay(1000);
}