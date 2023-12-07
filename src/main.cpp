#include <Arduino.h>

const double Kp = 0.05;
const double Ki = 0;
const double Kd = 0;

void setup () {
}

void loop() {
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