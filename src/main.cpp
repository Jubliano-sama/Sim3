#include <Arduino.h>
#include "basicFunctions.h"

// Function prototypes
double pidControl(double setpoint, double input, double Kp, double Ki, double Kd);
int calculateWeightedArraySum(const bool array[], int arrSize);
double *calculateMotorInput(double pidOutput);
bool isAllZero(bool *arr, int arrSize);
bool isAllOne(bool *arr, int arrSize);
void handlePauseSign();
bool handlePossibleStopPauseSign();
bool checkOvershoot();
void handleOvershoot();

void setup()
{
  setupBasicFunctions();
}

void loop()
{
  while(true){
    if (!digitalRead(switchPin)){
      driveMotors(0,0);
      continue;
    }

    // check for special cases
    if (handlePossibleStopPauseSign()) continue;

    if (checkOvershoot())
    {
      handleOvershoot();
      continue;
    }

    // no special case was found: using normal PID control
    double pid = pidControl(0, calculateWeightedArraySum(getSensorValues(), IRSensorsCount), Kp, Ki, Kd);
    double *motorInput;
    motorInput = calculateMotorInput(pid);
#if DEBUG >= 2
    Serial.print("\nPID output:");
    Serial.print(pid);
#endif
    driveMotors(motorInput[0], motorInput[1]);
    
  }
}

int calculateWeightedArraySum(const bool array[], int arrSize)
{

  if (arrSize % 2 == 0)
  {            // Check if array length is even
    return -1; // Return error if even length
  }

  int middleIndex = arrSize / 2; // Calculate middle index

  int value = 0; // Initialize result value

  for (int i = 0; i < arrSize; i++)
  {
    int arrayValue = array[i];           // Get array value at index i
    int positionDelta = i - middleIndex; // Calculate position delta

    value += arrayValue * positionDelta; // Adjust result value
  }

  return value; // Return calculated value
}

double pidControl(double setpoint, double input, double Kp, double Ki, double Kd)
{
  // Store the old time
  static unsigned long prevTime = 0;
  static double lastError = 0;
  // Calculate the current time
  unsigned long currentTime = millis();

  // Calculate the elapsed time
  unsigned long dt = currentTime - prevTime;
  prevTime = currentTime;

  if (dt == 0)
  {
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

double *calculateMotorInput(double pidOutput)
{
  static double motorInputs[] = {0, 0};
  pidOutput = constrain(pidOutput, -0.5, 0.5); // pidOutput = 0.5 ==> turn left pidOutput = -0.5 ==> turn right
  if (pidOutput > 0)
  {
    motorInputs[1] = 1;                                   // motorInput[0] = L // motorInput[1] = R
    motorInputs[0] = -8*pidOutput*pidOutput +1;           
  }
  else
  {
    motorInputs[1] = -8*pidOutput*pidOutput + 1;
    motorInputs[0] = 1;
  }
  return motorInputs;
}

bool handlePossibleStopPauseSign()
{
  if (!isAllOne(getSensorValues(), IRSensorsCount))
  {
    return false;
  }

#if DEBUG >= 1
    Serial.print("\nStop or pause sign detected: investigating...");
#endif  

  int beginTime = millis();

  // Checking for for pause sign
  while((millis() - beginTime) < stopPauseDelay){
    
    // safety check, if switchpin is low immediately stop motors and return to main loop.
    if(!digitalRead(switchPin)){
      driveMotors(0,0);
      return true;
    }

    driveMotors(0.2,0.2);
    if (isAllZero(getSensorValues(), IRSensorsCount)){
#if DEBUG >= 1
      Serial.print("\nPause sign detected!");
#endif  
      handlePauseSign();
      return true;
    }
  }

  // detected stop sign
#if DEBUG >= 1
  Serial.print("\nStop sign detected!");
#endif
  driveMotors(0,0);
  while(digitalRead(switchPin) == HIGH){
#if DEBUG >= 1
    Serial.print("\nRobot reached stop sign, toggle switchpin to continue again");
    delay(1000);
#endif
  }

  return true;
}

void handlePauseSign(){
  delay(5000);

  while (isAllZero(getSensorValues(), IRSensorsCount) || isAllOne(getSensorValues(), IRSensorsCount)){
    if(digitalRead(switchPin)){
      // move forward
      driveMotors(1,1);
    } else{
      // stop movement
#if DEBUG >= 1
      Serial.print("Switchpin detected low");
#endif
    driveMotors(0,0);
    return;
    }
  }
}

bool checkOvershoot()
{
  if (!isAllZero(getSensorValues(), IRSensorsCount))
  {
    return false;
  }

  /*
  if (readFrontIRSensor())
  {
    return false;
  }
  */

  return true;
}

void handleOvershoot()
{
#if DEBUG >= 1
  Serial.print("/nDetected overshoot, handling it...");
#endif
  unsigned long startTime = millis();
  const unsigned long timeout = 5000; // 5 seconds timeout

  while (isAllOne(getSensorValues(), IRSensorsCount))
  {
    // Check for timeout to prevent infinite loop
    if (millis() - startTime > timeout)
    {
#if DEBUG >= 1
      Serial.println("Overshoot handling timeout");
#endif
      break; // Exit the loop if timeout is reached
    }

    // turn in last direction we went in until overshoot is resolved
    double directionParsed = map((double)lastDirection, 0, 1, -1, 1);
    driveMotors(directionParsed, -directionParsed);
  }
}

bool isAllZero(bool *arr, int arrSize)
{
  for (int i = 0; i < arrSize; i++)
  {
    if (arr[i] != 0)
    {
      return false;
    }
  }
  return true;
}

bool isAllOne(bool *arr, int arrSize)
{
  for (int i = 0; i < arrSize; i++)
  {
    if (arr[i] != 1)
    {
      return false;
    }
  }
  return true;
}