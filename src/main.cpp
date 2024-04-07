#include <Arduino.h>
#include "config.h"
#include "motor.h"

void setup() {
  setupMotors();
}

void loop() {
  testMotors();
}

void moveShoulderToFieldValue(int value) {
  

  Serial.print("Attempting to move to value: ");
  Serial.println(value);
  // Find value in list of values
  int position = -1;

  for(int i = 0; i < amountOfFieldValues; i++){
    if(fieldValues[i] == value) position = i;
  }
  
  if(position = -1){
    Serial.println("value not found in list of values.");
    return;
  }

  Serial.print("Value array index found: ");
  Serial.println(position);

  // Convert array index to angle
  float angle = (float)position/(float)amountOfFieldValues * 360.0f;

  // Now this function will handle the intricacies of moving to the position efficiently.
  rotateShoulderAbsoluteAngle(angle);
}