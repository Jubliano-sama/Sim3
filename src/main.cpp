#include <Arduino.h>
#include "config.h"
#include "motor.h"

void moveShoulderToFieldValue(int value);

void setup() {
  Serial.begin(9600);
  setupMotors();
}

void loop() {
  moveShoulderServo(20);
  moveElbowServo(166);
  moveWristServo(170);

  delay(1000);
  moveGripServo(gripOpenAngle);
  delay(2000);
  moveGripServo(gripClosingAngle);
  delay(1000);
  moveElbowServo(90);
  delay(1000);
  moveShoulderToFieldValue(18);
  delay(2000);
  moveShoulderServo(20);
  moveElbowServo(166);
  moveWristServo(170);
  delay(1000);
  moveGripServo(gripOpenAngle);
  delay(1000);
  moveGripServo(gripClosingAngle);
  delay(2000);
  moveElbowServo(90);
  delay(500);
  moveShoulderToFieldValue(14);
  delay(1500);

  moveShoulderServo(20);
  moveElbowServo(166);
  moveWristServo(170);
  delay(1000);
  moveGripServo(gripOpenAngle);
  delay(1000);
  moveGripServo(gripClosingAngle);
  delay(2000);
  moveElbowServo(90);
  delay(500);
  moveShoulderToFieldValue(6);
  delay(1500);

    moveShoulderServo(20);
  moveElbowServo(166);
  moveWristServo(170);
  delay(1000);
  moveGripServo(gripOpenAngle);
  delay(1000);
  moveGripServo(gripClosingAngle);
  delay(2000);
  moveElbowServo(90);
  delay(500);
  moveShoulderToFieldValue(20);
  delay(1500);

    moveShoulderServo(20);
  moveElbowServo(166);
  moveWristServo(170);
  delay(1000);
  moveGripServo(gripOpenAngle);
  delay(1000);
  moveGripServo(gripClosingAngle);
  delay(2000);
  moveElbowServo(90);
  delay(500);
  moveShoulderToFieldValue(10);
  delay(1500);
}

void moveShoulderToFieldValue(int value) {
  

  Serial.print("Attempting to move to value: ");
  Serial.println(value);
  // Find value in list of values
  int position = -1;

  for(int i = 0; i < amountOfFieldValues; i++){
    if(fieldValues[i] == value) position = i;
  }
  
  if(position == -1){
    Serial.println("value not found in list of values.");
    return;
  }

  Serial.print("Value array index found: ");
  Serial.println(position);

  // Convert array index to angle
  float angle = (float)position/(float)amountOfFieldValues * 360.0f;

  // Now this function will handle the intricacies of moving to the position efficiently.
  rotateShoulderAbsoluteAngle(-angle + 9);
}