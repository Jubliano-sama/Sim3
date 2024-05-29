#include "motor.h"
#include <Arduino.h>
#include "config.h"

void initSensors() {
  pinMode(IR_SENSOR_PIN, INPUT);
}

float startDetectionRoutine() {
  for(int i; i <360; i++){
      rotateShoulderRelativeAngle(1);
      
      while(!hasStepperReachedPosition){
        if(digitalRead(IR_SENSOR_PIN)){ // Object seen
          return getShoulderAngle();
        }
      }
  }
  return -1;
}