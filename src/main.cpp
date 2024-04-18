#include <Arduino.h>
#include "config.h"
#include "motor.h"

float convertFieldValueToAngle(int value);

void moveToGrabbingPosition();
void moveToRotatingPosition();
void closeGrippers();
void openGrippers();

void setup() {
  Serial.begin(9600);
  setupMotors();
}

void loop() {
  moveToGrabbingPosition();
  openGrippers();
  delay(1000);
  closeGrippers();
  delay(1000);
  moveToRotatingPosition();
  delay(100);

  // rotates to value 18
  rotateShoulderAbsoluteAngle(convertFieldValueToAngle(18));
  delay(1500);
  moveToGrabbingPosition();
  delay(1000);
  openGrippers();
  delay(1000);
  closeGrippers();
  delay(1000);

  moveToRotatingPosition();
  delay(100);

  // Rotates to field value 14
  rotateShoulderAbsoluteAngle(convertFieldValueToAngle(14));
  delay(1500);
}

void moveToGrabbingPosition(){
  Serial.println("Moving to Grabbing Position");
  
  // Moves arm down to grabbing position.
  moveShoulderServo(20);
  moveElbowServo(166);
  moveWristServo(170);
}

void moveToRotatingPosition() {
  Serial.println("Moving to Rotating position");
  
  // Moves arm up so that it doesnt drag the object on the ground.
  moveElbowServo(90);
}

void closeGrippers() {
  moveGripServo(gripClosingAngle);
}

void dropObject() {
  moveGripServo(gripClosingAngle);
}

float convertFieldValueToAngle(int value) {
  Serial.print("Attempting to move to value: ");
  Serial.println(value);
  // Find value in list of values
  int position = -1;

  for(int i = 0; i < amountOfFieldValues; i++){
    if(fieldValues[i] == value) position = i;
  }
  
  if(position == -1){
    Serial.println("value not found in list of values.");
    return 0.0f;
  }

  Serial.print("Value array index found: ");
  Serial.println(position);

  // Convert array index to angle
  float angle = (float)position/(float)amountOfFieldValues * 360.0f;

  return -angle + 9;
}