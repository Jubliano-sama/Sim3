#include <Arduino.h>
//sim 1

//constants
//IR sensoren
const int IRSensors[] ={22,23,24,25,26}; //van links naar recht, van laag naar hoog
const int aantalIRSensors=5;
const int frontIRSensor=52;

//StartSwitch
const int startSwitch=53;

//DC motors {LV,LA,RV,RA}
const int motorsFoward[]={51,49,47,45};
const int motorsBackward[]={50,48,46,44};
const int motorsENA[]={2,3,4,5};
const int aantalMotors=4;

//PWM signaal voor motoren
double calibratieFactors[]={1,1,1,1}; //calibratiefactoren tussen 0 en 1

//Setup
void setupBasicFunctions(){
    //IR sensoren
    for(int i=0;i<aantalIRSensors;i++){
        pinMode(IRSensors[i], INPUT);
    }

    //StartSwitch
    pinMode(startSwitch, INPUT);
}

bool readFrontIRSensor(){
    bool frontIRValue=digitalRead(frontIRSensor);
    frontIRValue= !frontIRValue;

    return frontIRValue;
}

bool *readIRSensors(){
    static bool IRValues[aantalIRSensors];
    for(int i=0;i<aantalIRSensors;i++){
        IRValues[i]=digitalRead(IRSensors[i]);
        IRValues[i]= !IRValues[i];
    }

    return IRValues;
}

int  convertToPWM(double input){
    return map(input, 0,1,0, 255);
}

void driveMotors(double leftFront, double rightFront, double leftBack, double rightBack){
    double motorSignals[4] = {leftFront, rightFront, leftBack, rightBack};
    for (int i=0;i<aantalMotors; i++){
        digitalWrite(motorsFoward[i],HIGH);
        digitalWrite(motorsBackward[i],LOW);
        analogWrite(motorsENA[i],convertToPWM(calibratieFactors[i] * motorSignals[i]));
    }
}