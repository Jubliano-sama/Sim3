#include "config.h"
#include "motor.h"
#include "config.h"

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

Servo shoulderServo;
Servo elbowServo;
Servo wristServo;
Servo gripServo;

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
	stepper = engine.stepperConnectToPin(stepPin);
	if (stepper)
	{
		stepper->setDirectionPin(dirPin);
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
	stepper->move(steps);
}

void stopShoulder()
{
	Serial.println("Stopping stepper motor immediately.");
	stepper->stopMove();
}

float getShoulderAngle(){
	return positionToAngle(stepper->getCurrentPosition());
}

void rotateShoulderRelativeAngle(float angle){
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
	Serial.print("Converted position ");
	Serial.print(position);
	Serial.print(" to angle: ");
	Serial.println(angle);
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
	moveServo(elbowServo, angle);
}

void moveShoulderServo(int angle)
{
	Serial.print("Moving shoulder servo to angle: ");
	Serial.println(angle);
	moveServo(shoulderServo, angle);
}

void moveGripServo(int angle)
{
	Serial.print("Moving grip servo to angle: ");
	Serial.println(angle);
	moveServo(gripServo, angle);
}

void moveWristServo(int angle)
{
	Serial.print("Moving wrist servo to angle: ");
	Serial.println(angle);
	moveServo(wristServo, angle);
}

void moveToArmConfiguration(ArmConfiguration configuration) {
	moveShoulderServo(configuration.shoulderAngle);
	moveElbowServo(configuration.elbowAngle);
	moveWristServo(configuration.wristAngle);
}

void setStepperSpeed(int stepsPerSecond){
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
	moveServo(elbowServo, 0);  // Move elbow servo to 90 degrees
	delay(1000);
	moveServo(elbowServo, 90); // Move elbow servo to 90 degrees
	delay(1000);
	moveServo(shoulderServo, 0);  // Move shoulder servo to 45 degrees
	delay(1000);
	moveServo(shoulderServo, 90); // Move shoulder servo to 45 degrees
	delay(1000);
	moveServo(wristServo, 0);  // Move shoulder servo to 45 degrees
	delay(1000);
	moveServo(wristServo, 90); // Move shoulder servo to 45 degrees
	delay(1000);
	moveServo(gripServo, 0);  // Close grip slightly (10 degrees)
	delay(1000);
	moveServo(gripServo, 90); // Close grip slightly (10 degrees)
	delay(1000);
}