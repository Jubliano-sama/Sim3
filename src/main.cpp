#include <Arduino.h>
#include "basicFunctions.h"

const double Kp = 0.05;
const double Ki = 0;
const double Kd = 0;

const int switchPin = 53;

void setup () {
  pinMode(switchPin, INPUT);
}


double pidControl(double setpoint, double input, double Kp, double Ki, double Kd) {
  // Store the old time
  static unsigned long prevTime = 0;
  static double lastError = 0;
  // Calculate the current time
  unsigned long currentTime = millis();

  // Calculate the elapsed time
  unsigned long dt = currentTime - prevTime;
  prevTime = currentTime;

  if (dt == 0){
    return 0;
  }
  // Calculate error
  double error = setpoint - input;

  // Calculate proportional term
  double proportional = error * Kp;

  // Calculate integral term
  static double integral = 0.0;
  integral += error * dt;
  double integralTerm = Ki * integral;

  // Calculate derivative term
  double derivative = (error - lastError) / dt;
  double derivativeTerm = Kd * derivative;

  // Calculate PID output
  double pidOutput = proportional + integralTerm + derivativeTerm;

  // Update last error for derivative calculation
  lastError = error;

  // Return PID output
  return pidOutput; 
}

int calculateWeightedArraySum(const bool array[]) {
  int length = sizeof(array) / sizeof(array[0]); // Determine array length
  
  if (length % 2 == 0) { // Check if array length is even
    return -1; // Return error if even length
  }

  int middleIndex = length / 2; // Calculate middle index

  int value = 0; // Initialize result value

  for (int i = 0; i < length; i++) {
    int arrayValue = array[i]; // Get array value at index i
    int positionDelta = i - middleIndex; // Calculate position delta

    value += arrayValue * positionDelta; // Adjust result value
  }

  return value; // Return calculated value
}
double *calculateMotorInput(double pidOutput){
    static double motorInputs[] = {0,0};
    pidOutput += 0.5;
    pidOutput = constrain(pidOutput,0,1); //pidOutput = 1 ==> full left pidOutput = o ==> full right  
    if (pidOutput > 0.5){
        motorInputs[0] = 1;     // motorInput[0] = L
        motorInputs[1] = 1/pidOutput -1;    // motorInput[1] = R
    }
    else {
        motorInputs[0] = 1/pidOutput -1;
        motorInputs[1] = 1;
    }
    return motorInputs;
}

bool isAllZero(bool arr[]) {
    for (int i = 0; i < sizeof(arr) / sizeof(arr[0]); i++) {
        if (arr[i] != 0) {
            return false;
        }
    }
    return true;
}

bool isAllOne(bool arr[]) {
    for (int i = 0; i < sizeof(arr) / sizeof(arr[0]); i++) {
        if (arr[i] != 1) {
            return false;
        }
    }
    return true;
}

bool checkStopSign(bool sensorArr[], bool frontSensor){
  if (!frontSensor){
    return false;
  }
  
  if (!isAllOne(sensorArr)){
    return false;
  }

  return true;
}

bool checkPauseSign(bool sensorArr[], bool frontSensor){
  if (!frontSensor){
    return false;
  }
  
  if (!isAllZero(sensorArr)){
    return false;
  }

  return true;
}

void handlePauseSign(){
  delay(5000);

  while (isAllZero(readIRSensors())){
    if(digitalRead(switchPin)){
      // move forward
      driveMotors(1,1,1,1);
    } else{
      // stop movement
      driveMotors(0,0,0,0);
      delay(500);
    }
  }
}

void loop() { 
  if(digitalRead(switchPin)){
    bool* sensorArr;
    sensorArr = readIRSensors();
    bool frontSensor = readFrontIRSensor();
    if (checkPauseSign(sensorArr, frontSensor)){
      handlePauseSign();
    }

    if (checkStopSign(sensorArr, frontSensor)) {
      driveMotors(0,0,0,0);
    } else {
      double pid = pidControl(0, calculateWeightedArraySum(sensorArr), Kp, Ki, Kd);
      double* motorInput;
      motorInput = calculateMotorInput(pid);
      driveMotors(motorInput[0], motorInput[1], motorInput[0], motorInput[1]);
    }

  } else {
    delay(500);
  }
}