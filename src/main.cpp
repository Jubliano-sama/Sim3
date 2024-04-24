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
  moveToGrabbingPosition();
  openGrippers();
  moveElbowServo(180);
  delay(1000);
  for(int i = 0; i<=100; i++){
    delay(10);
    moveElbowServo(180-((44*i)/100));
  }
  moveToGrabbingPosition();
  delay(1000);
  delay(500);
  closeGrippers();
  delay(500);
}

void loop() {
  moveToRotatingPosition();
  rotateShoulderAbsoluteAngle(convertFieldValueToAngle(16));
  delay(500);
  moveToGrabbingPosition();
  delay(1000);
  openGrippers();
  delay(500);
  closeGrippers();
  delay(300);
  moveToRotatingPosition();
  rotateShoulderAbsoluteAngle(convertFieldValueToAngle(18));
  delay(500);
  moveToGrabbingPosition();
  delay(1000);
  openGrippers();
  delay(500);
  closeGrippers();
  delay(500);
}


void moveToGrabbingPosition(){
  Serial.println("Moving to Grabbing Position");
  
  // Moves arm down to grabbing position.
  moveShoulderServo(10);
  moveElbowServo(136);
  moveWristServo(170);
}

// Moves arm up so that it doesnt drag the object on the ground.
void moveToRotatingPosition() {
  Serial.println("Moving to Rotating position");
  moveShoulderServo(20);
}

void closeGrippers() {
  moveGripServo(gripClosingAngle);
}

void openGrippers() {
  moveGripServo(gripOpenAngle);
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
  return -angle;
}