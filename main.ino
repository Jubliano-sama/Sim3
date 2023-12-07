#include <Arduino.h>

void setup(){}

void loop(){}

float pidControl(float setpoint, float input, float Kp, float Ki, float Kd) {
  // Store the old time
  static unsigned long prevTime = 0;

  // Calculate the current time
  unsigned long currentTime = millis();

  // Calculate the elapsed time
  unsigned long dt = currentTime - prevTime;
  prevTime = currentTime;

  if (dt == 0){
    return
    }
  // Calculate error
  float error = setpoint - input;

  // Calculate proportional term
  float proportional = error * Kp;

  // Calculate integral term
  static float integral = 0.0;
  integral += error * dt;
  float integralTerm = Ki * integral;

  // Calculate derivative term
  float derivative = (error - lastError) / dt;
  float derivativeTerm = Kd * derivative;

  // Calculate PID output
  float pidOutput = proportional + integralTerm + derivativeTerm;

  // Update last error for derivative calculation
  lastError = error;

  // Return PID output
  return pidOutput;
}

int calculateWeightedArraySum(const int array[]) {
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