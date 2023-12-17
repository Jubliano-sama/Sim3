#include <Arduino.h>
#include "basicFunctions.h"

void setup()
{
  pinMode(switchPin, INPUT);
  setupBasicFunctions();
  Serial.begin(9600);
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

void handlePauseSign(){
  delay(5000);

  while (isAllZero(getSensorValues(), IRSensorsCount) || isAllOne(getSensorValues(), IRSensorsCount)){
    if(digitalRead(switchPin)){
      // move forward
      driveMotors(1,1);
    } else{
      // stop movement
      driveMotors(0,0);
      delay(5000);
    }
  }
}

void handlePossibleStopPauseSign()
{
  if (!isAllOne(getSensorValues(), IRSensorsCount))
  {
    return;
  }
  int beginTime = millis();

  while((millis() - beginTime) < stopPauseDelay){
    driveMotors(0.2,0.2);
    if (isAllZero(getSensorValues(), IRSensorsCount)){
      handlePauseSign();
#ifdef DEBUG
Serial.print("\nPause sign detected");
#endif
      return;
    }
  }

  // detected stop sign
#ifdef DEBUG
  Serial.print("\nStop sign detected");
#endif
  while(digitalRead(switchPin) == HIGH);
}
bool checkOvershoot()
{
  if (!isAllZero(getSensorValues(), IRSensorsCount))
  {
    return false;
  }
  if (readFrontIRSensor())
  {
    return false;
  }

  return true;
}

void handleOvershoot()
{
  unsigned long startTime = millis();
  const unsigned long timeout = 5000; // 5 seconds timeout

  while (isAllOne(getSensorValues(), IRSensorsCount))
  {
    // Check for timeout to prevent infinite loop
    if (millis() - startTime > timeout)
    {
      Serial.println("Overshoot handling timeout");
      break; // Exit the loop if timeout is reached
    }

    // turn in last direction we went in until overshoot is resolved
    double directionParsed = map((double)lastDirection, 0, 1, -1, 1);
    driveMotors(directionParsed, -directionParsed);
  }
}

void loop()
{
  if (digitalRead(switchPin))
  {
    bool frontSensor = readFrontIRSensor();
    bool *sensorValues = getSensorValues();
    handlePossibleStopPauseSign();
    if (checkOvershoot())
    {
      handleOvershoot();
    }
    else
    {
      double pid = pidControl(0, calculateWeightedArraySum(getSensorValues(), IRSensorsCount), Kp, Ki, Kd);
      double *motorInput;
      motorInput = calculateMotorInput(pid);
      driveMotors(motorInput[0], motorInput[1]);
#ifdef DEBUG
      Serial.print("\nPID output:");
      Serial.print(pid);
      Serial.print("\nMotor input: ");
      Serial.print(motorInput[0]);
      Serial.print(motorInput[1]);
#endif
    }
  }
  else
  {
    delay(500);
  }
}