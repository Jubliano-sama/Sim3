#include <Arduino.h>
#include "basicFunctions.h"

double calibratieFactors[]={1,1,1,1}; //calibratiefactoren tussen 0 en 1

bool sensorArr[IRSensorsCount];

//Setup
void setupBasicFunctions(){
    //IR sensoren
    for(int i=0;i<IRSensorsCount;i++){
        pinMode(IRSensors[i], INPUT);
    }

    //StartSwitch
    pinMode(startSwitch, INPUT);
}

bool readFrontIRSensor(){
    bool frontIRValue=digitalRead(frontIRSensorPin);
    frontIRValue= !frontIRValue;

    return frontIRValue;
}

void updateIRSensors(){
    for(int i=0;i<IRSensorsCount;i++){
        sensorArr[i]=digitalRead(IRSensors[i]);
        sensorArr[i]= !sensorArr[i];
    }
}

bool * getSensorValues(){
    updateIRSensors();
    return sensorArr;
}

int  convertToPWM(double input){
    return map(input, 0,1,0, 255);
}

void driveMotors(double left, double right){
    double motorSignals[2] = {left, right};

    // LET OP: BUG IN DEZE CODE ZEER WAARSCHIJNLIJK, CHECK DE PINS!!
    digitalWrite(motorsFowardPins[0],(bool)ceil(left));
    digitalWrite(motorsFowardPins[1],(bool)ceil(right));
    digitalWrite(motorsBackwardPins[2],!motorsFowardPins[0]);
    digitalWrite(motorsBackwardPins[3],!motorsFowardPins[1]);

    for (int i=0;i<4; i++){
        analogWrite(motorsENAPins[i],convertToPWM(calibratieFactors[i] * abs(motorSignals[i])));
    }
}