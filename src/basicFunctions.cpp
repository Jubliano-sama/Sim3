#include <Arduino.h>
#include "basicFunctions.h"

double calibratieFactors[] = {0.4, 0.4, 0.4, 0.4}; // calibratiefactoren tussen 0 en 1

bool sensorArr[IRSensorsCount];

// 0=left, 1=right
bool lastDirection = 0;

// Setup
void setupBasicFunctions()
{
    // IR sensoren
    for (int i = 0; i < IRSensorsCount; i++)
    {
        pinMode(IRSensors[i], INPUT);
    }

    // StartSwitch
    pinMode(switchPin, INPUT);

    Serial.begin(115200);
}

bool readFrontIRSensor()
{
    bool frontIRValue = digitalRead(frontIRSensorPin);
    frontIRValue = !frontIRValue;

#ifdef DEBUG
    Serial.print("\nFront sensor: ");
    Serial.print(frontIRValue);
#endif
    return frontIRValue;
}

void updateIRSensors()
{
    for (int i = 0; i < IRSensorsCount; i++)
    {
        sensorArr[i] = digitalRead(IRSensors[i]);
        sensorArr[i] = !sensorArr[i];
    }
}

bool *getSensorValues()
{
    updateIRSensors();
#ifdef DEBUG
    Serial.print("\nSensorarr: ");
    for (bool element : sensorArr) // for each element in the array
        Serial.print(element);     // print the current element
#endif
    return &sensorArr[0];
}

int convertToPWM(double input)
{
    return map(input, 0, 1, 0, 255);
}

void driveMotors(double left, double right)
{
    double motorSignals[4] = {left, left, right, right};

    if (left > right)
    {
        lastDirection = 1;
    }
    else
    {
        lastDirection = 0;
    }

    // Set motor direction pins
    for (int motorIndex = 0; motorIndex < 4; motorIndex++)
    {
        bool forwardState = (bool)ceil(motorSignals[motorIndex]);
        digitalWrite(motorsFowardPins[motorIndex], forwardState);
        digitalWrite(motorsBackwardPins[motorIndex], !forwardState);

#ifdef DEBUG
        Serial.print("\nMotor ");
        Serial.print(motorIndex);
        Serial.print(" forward pin: ");
        Serial.print(motorsFowardPins[motorIndex]);
        Serial.print(", backward pin: ");
        Serial.print(motorsBackwardPins[motorIndex]);
#endif
    }

    for (int i = 0; i < 4; i++)
    {
        int pwmValue = convertToPWM(calibratieFactors[i] * abs(motorSignals[i]));
        analogWrite(motorsENAPins[i], pwmValue);
#ifdef DEBUG
        Serial.print("\nSetting motor: ");
        Serial.print(i);
        Serial.print(", to: ");
        Serial.print((float)pwmValue / 255 * 100);
        Serial.print("%");
#endif
    }
}