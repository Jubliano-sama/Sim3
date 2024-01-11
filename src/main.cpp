#include <Arduino.h>
#include "basicFunctions.h"

// Function prototypes
double pidControl(double setpoint, double input, double Kp, double Ki, double Kd);
int calculateWeightedArraySum(const bool array[], int arrSize);
double *calculateMotorInput(double pidOutput);
bool isAllZero(bool *arr, int arrSize);
bool isAllOne(bool *arr, int arrSize);
void handlePauseSign();
bool handlePossibleStopPauseSign();
bool checkOvershoot();
void handleOvershoot();
void(* resetFunc) (void) = 0;

void setup()
{
	setupBasicFunctions();
}

void loop()
{
	if (!digitalRead(switchPin))
		{
		driveMotors(0, 0);
#if DEBUG >= 1
		Serial.print("\nSwitchpin off");
		delay(500);
#endif
		return;
	}

	// check for special cases
	if (handlePossibleStopPauseSign())
		return;

	if (checkOvershoot())
	{
		handleOvershoot();
		return;
	}

	// no special case was found: using normal PID control
	double pid = pidControl(0, calculateWeightedArraySum(getAnalogSensorValues(), IRSensorsCount), Kp, Ki, Kd);
	double *motorInput;
	motorInput = calculateMotorInput(pid);
#if DEBUG >= 2
	Serial.print("\nPID output: ");
	Serial.print(pid);
#endif
	driveMotors(motorInput[0], motorInput[1]);

}

int calculateWeightedArraySum(const bool array[], int arrSize)
{
	int middleIndex = arrSize / 2; // Calculate middle index

	int value = 0; // Initialize result value

	for (int i = 0; i < arrSize; i++)
	{
		int arrayValue = array[i];           // Get array value at index i
		int positionDelta = i - middleIndex; // Calculate position delta

		value += arrayValue * positionDelta; // Adjust result value
	}

#if DEBUG >= 2
	Serial.print("\nWeighted sum: ");
	Serial.print(value);
#endif
	return value; // Return calculated value
}

double pidControl(const double setpoint, const double input, const double Kp, const double Ki, const double Kd)
{
	// Store the old time
	static unsigned long prevTime = 0;
	static double lastError = 0;
	// Calculate the current time
	unsigned long currentTime = millis();

	// Calculate the elapsed time
	unsigned long dt = currentTime - prevTime;
	prevTime = currentTime;

	if (dt == 0)
	{
		return 0;
	}
	// Calculate error
	double error = setpoint - input;

	// Calculate proportional term
	double proportional = error * Kp;

	// Calculate integral term
	static double integral = 0.0;
	integral += error * dt;
	integral = constrain(integral, -0.5/Ki, 0.5/Ki);
	double integralTerm = Ki * integral;

	// Calculate derivative term
	double derivative = (error - lastError) / dt;
	double derivativeTerm = Kd * derivative;

	// Calculate PID output
	double pidOutput = proportional + integralTerm + derivativeTerm;

	// Update last error for derivative calculation
	lastError = error;

	// Return PID output
	return pidOutput;
}

double* calculateMotorInput(double pidOutput) {
    static double motorInputs[2]; // [left, right]

    // Clipping the pidOutput between -0.5 and 0.5
    pidOutput = constrain(pidOutput, -0.5, 0.5);

    // Calculating the motor inputs based on the new linear relationships
    // and clipping them at a maximum of 1
    motorInputs[0] = min(-4 * pidOutput + 1, 1); // Left motor (non-dominant)
    motorInputs[1] = min(4 * pidOutput + 1, 1);  // Right motor (non-dominant)

    return motorInputs;
}

bool handlePossibleStopPauseSign()
{
	if (!isAllOne(getAnalogSensorValues(), IRSensorsCount))
	{
		return false;
	}

#if DEBUG >= 1
	Serial.print("\nStop or pause sign detected: investigating...");
#endif

	unsigned long beginTime = millis();

	// Checking for for pause sign
	while ((millis() - beginTime) < stopPauseDelay)
	{

		// safety check, if switchpin is low immediately stop motors and return to main loop.
		if (!digitalRead(switchPin))
		{
			driveMotors(0, 0);
			return true;
		}

		driveMotors(0.5, 0.5);
		if (isAllZero(getAnalogSensorValues(), IRSensorsCount))
		{
#if DEBUG >= 1
			Serial.print("\nPause sign detected!");
#endif
			handlePauseSign();
			return true;
		}
	}

	// detected stop sign
#if DEBUG >= 1
	Serial.print("\nStop sign detected!");
#endif
	driveMotors(0, 0);
	while (digitalRead(switchPin) == HIGH)
	{
#if DEBUG >= 1
		Serial.print("\nRobot reached stop sign, toggle switchpin to continue again");
		delay(1000);
#endif
	}
	return true;
}

void handlePauseSign()
{
	delay(5000);

	while (isAllZero(getAnalogSensorValues(), IRSensorsCount) || isAllOne(getAnalogSensorValues(), IRSensorsCount))
	{
		if (digitalRead(switchPin))
		{
			// move forward
			driveMotors(1, 1);
		}
		else
		{
			// stop movement
#if DEBUG >= 1
			Serial.print("Switchpin detected low");
#endif
			driveMotors(0, 0);
			return;
		}
	}
}

bool checkOvershoot()
{
	if (isAllZero(getAnalogSensorValues(), IRSensorsCount))
	{
		return true;
	}

	/*
	if (readFrontIRSensor())
	{
	  return false;
	}
	*/

	return false;
}

void handleOvershoot()
{
#if DEBUG >= 1
	Serial.print("\nDetected overshoot, handling it...");
#endif
	
	double directionParsed = (double)map(lastDirection, 0, 1, -1, 1);
	while (isAllZero(getAnalogSensorValues(), IRSensorsCount))
	{
		// safety check
		if (!digitalRead(switchPin))
		{
			driveMotors(0, 0);
			return;
		}
		// turn in last direction we went in until overshoot is resolved
#if DEBUG >= 2
		Serial.print("Overshoot still detected... turning more");
#endif
		driveMotors(directionParsed * 0.4, -directionParsed * 0.4);
	}
}

bool isAllZero(bool *arr, int arrSize)
{
	for (int i = 0; i < arrSize; i++)
	{
		if (arr[i] != 0)
		{
			return false;
		}
	}
	return true;
}

bool isAllOne(bool *arr, int arrSize)
{
	for (int i = 0; i < arrSize; i++)
	{
		if (arr[i] != 1)
		{
			return false;
		}
	}
	return true;
}