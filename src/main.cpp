#include <Arduino.h>
#include "config.h"
#include "motor.h"

float convertFieldValueToAngle(int value);

void setup() {
  Serial.begin(9600);
  setupMotors();
}

void loop() {
  //Moves to first grabbing position
  moveShoulderServo(20);
  moveElbowServo(166);
  moveWristServo(170);

  // Grabs the object
  delay(1000);
  moveGripServo(gripOpenAngle);
  delay(2000);
  moveGripServo(gripClosingAngle);
  delay(1000);

  // Moves arm up so that it doesnt drag the object on the ground.
  moveElbowServo(90);
  delay(1000);

  // rotates to value 18
  rotateShoulderAbsoluteAngle(convertFieldValueToAngle(18));
  delay(2000);

  // Moves arm down to grabbing position.
  moveShoulderServo(20);
  moveElbowServo(166);
  moveWristServo(170);
  delay(1000);

  //Lets go of the object and grabs again
  moveGripServo(gripOpenAngle);
  delay(1000);
  moveGripServo(gripClosingAngle);
  delay(2000);

  // Moves arm up so that it doesnt drag the object on the ground.
  moveElbowServo(90);
  delay(500);

  // Rotates to field value 14
  rotateShoulderAbsoluteAngle(convertFieldValueToAngle(14));
  delay(1500);

  // Moves arm down to grabbing position.
  moveShoulderServo(20);
  moveElbowServo(166);
  moveWristServo(170);

  // Lets go of the object and grabs again
  delay(1000);
  moveGripServo(gripOpenAngle);
  delay(1000);
  moveGripServo(gripClosingAngle);
  delay(2000);

  // Moves arm up so that it doesnt drag the object on the ground.
  moveElbowServo(90);
  delay(500);

  // Rotates shoulder to field value 6
  rotateShoulderAbsoluteAngle(convertFieldValueToAngle(6));
  delay(1500);
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