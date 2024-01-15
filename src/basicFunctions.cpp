#include <Arduino.h>
#include "basicFunctions.h"

double calibratieFactors[] = {1, 1, 1, 1}; // calibratiefactoren tussen 0 en 1

bool sensorArr[IRSensorsCount];

// 0= turning left, 1= turning right
int lastDirection = 1;

// Setup
void setupBasicFunctions()
{
	// IR sensoren
	for (int i = 0; i < IRSensorsCount; i++)
	{
		pinMode(IRSensors[i], INPUT);
	}

	pinMode(frontIRSensorPin, INPUT_PULLUP);
	// StartSwitch
	pinMode(switchPin, INPUT_PULLUP);

	Serial.begin(115200);
}

bool readFrontIRSensor()
{
	bool frontIRValue = digitalRead(frontIRSensorPin);

#if DEBUG >= 2
	Serial.print("\nFront sensor: ");
	Serial.print(frontIRValue);
#endif
	return frontIRValue;
}

void updateIRSensors()
{
	for (int i = 0; i < IRSensorsCount; i++)
	{
		// check if sensor is disabled
		for (int j = 0; j < disabledSensorsCount; j++){
			if (IRSensors[i] == disabledSensors[j]){
				sensorArr[i] = LOW;
				continue;
			}
		}
		
		sensorArr[i] = digitalRead(IRSensors[i]);
	}
}

bool *getSensorValues()
{
	updateIRSensors();
#if DEBUG >= 2
	Serial.print("\nSensorarr: ");
	for (bool element : sensorArr) // for each element in the array
		Serial.print(element);	   // print the current element
#endif

	if (sensorArr[0] == 0 ){
		lastDirection = 1;
	} else if (sensorArr[IRSensorsCount - 1] == 1){
		lastDirection = 0;
	}
	return &sensorArr[0];
}

bool* getAnalogSensorValues() {
    static bool analogSensorArr[analogSensorsCount];
    
    #if DEBUG >= 2
    Serial.print("\n[");
    #endif

    for (int i = 0; i < analogSensorsCount; i++) {
        int sensorValue = analogRead(analogSensors[i]);
        analogSensorArr[i] = sensorValue > analogSensorThresholds[i];

        // Debugging: Output analog values
        #if DEBUG >= 2
        Serial.print(sensorValue);
        if (i < analogSensorsCount - 1) {
            Serial.print(", ");
        }
        #endif
    }

    // Debugging: Output boolean values
    #if DEBUG >= 2
    Serial.println("]");
    Serial.print(" Digital Values: [");
    for (int i = 0; i < analogSensorsCount; i++) {
        Serial.print(analogSensorArr[i]);
    }
    Serial.println("]");
    #endif

	if (analogSensorArr[0] == 1 ){
		lastDirection = -1;
	} else if (analogSensorArr[analogSensorsCount - 1] == 1){
		lastDirection = 1;
	}
    return analogSensorArr;
}

double mapDouble(double x, double inMin, double inMax, double outMin, double outMax)
{
	if (inMax == inMin) return x;

	// Check if the input value is within the input range
	if (x < inMin || x > inMax)
	{
		return x; // Return the input value if it's outside the range
	}

	// Calculate the mapped value
	double deltaIn = inMax - inMin;
	double deltaOut = outMax - outMin;

	double mappedVal = x - inMin;
	mappedVal *= deltaOut / deltaIn;
	mappedVal += outMin;

	return mappedVal;
}

int convertToPWM(double input)
{
	return (int)constrain(mapDouble(input, 0, 1, 0, 255), 0, 255);
}

void driveMotors(double left, double right)
{
#if DEBUG >= 2
	Serial.print("\nReceived inputs: ");
	Serial.print(left);
	Serial.print(" ");
	Serial.print(right);
#endif
	double motorSignals[4] = {left, left, right, right};

	// Set motor direction pins
	for (int motorIndex = 0; motorIndex < 4; motorIndex++)
	{
		bool forwardState = false;
		if (motorSignals[motorIndex] >= 0){
			forwardState = true;
		}
		digitalWrite(motorsForwardPins[motorIndex], forwardState);
		digitalWrite(motorsBackwardPins[motorIndex], !forwardState);

#if DEBUG >= 2
		Serial.print("\nMotor ");
		Serial.print(motorIndex);
		Serial.print(" forward pin: ");
		Serial.print(forwardState);
		Serial.print(", backward pin: ");
		Serial.print(!forwardState);
#endif
	}

	for (int i = 0; i < 4; i++)
	{
		int pwmValue = convertToPWM(calibratieFactors[i] * abs(motorSignals[i]));
		analogWrite(motorsENAPins[i], pwmValue);
#if DEBUG >= 2
		Serial.print("\nSet motor: ");
		Serial.print(i);
		Serial.print(", to: ");
		Serial.print(pwmValue);
#endif
	}
}