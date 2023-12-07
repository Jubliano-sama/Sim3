#include <Arduino.h>
//sim 1

//constants
//IR sensoren
const int IRSensors[] ={22,23,24,25,26}; //van links naar recht, van laag naar hoog
const int aantalIRSensors=5;

//StartSwitch
const int startSwitch=53;

//DC motors {LV,LA,RV,RA}
const int motorsFoward[]={51,49,47,45};
const int motorsBackward[]={50,48,46,44};
const int motorsENA[]={2,3,4,5};


//Setup
void setup(){
    //IR sensoren
    for(int i=0;i<aantalIRSensors;i++){
        pinMode(IRSensors[i], INPUT);
    }

    //StartSwitch
    pinMode(startSwitch, INPUT);
}

void readIRSensors(){
    for(int i=0;i<aantalIRSensors;i++){
        int IRValues[i]=digitalRead(IRSensors[i]);
    }
}


void loop(){
    if(startSwitch==1){
        readIRSensors();
    }
  
}