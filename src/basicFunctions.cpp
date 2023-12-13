#include <Arduino.h>
#include "basicFunctions.h"

double calibratieFactors[] = {1, 1, 1, 1}; // calibratiefactoren tussen 0 en 1

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
    double motorSignals[2] = {left, right};

    if (left > right)
    {
        lastDirection = 1;
    }
    else
    {
        lastDirection = 0;
    }

    // LET OP: BUG IN DEZE CODE ZEER WAARSCHIJNLIJK, CHECK DE PINS!!
    digitalWrite(motorsFowardPins[0], (bool)ceil(left));
    digitalWrite(motorsFowardPins[1], (bool)ceil(right));
    digitalWrite(motorsBackwardPins[2], !motorsFowardPins[0]);
    digitalWrite(motorsBackwardPins[3], !motorsFowardPins[1]);

    for (int i = 0; i < 4; i++)
    {
        int pwmValue = convertToPWM(calibratieFactors[i] * abs(motorSignals[i]));
        analogWrite(motorsENAPins[i], pwmValue);
#ifdef DEBUG
        Serial.print("\nWriting motor, value:");
        Serial.print(i);
        Serial.print(pwmValue);
#endif
    }
}