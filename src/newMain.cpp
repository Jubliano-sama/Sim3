#include <Arduino.h>
#include "config.h"
#include "motor.h"
#include "carHardware.h"

enum State
{
    STATE_INITIALIZATION,
    STATE_DRIVING,
    STATE_PAUSE_BEGIN,
    STATE_SCAN_FIELD,
    STATE_MOVE_OBJECTS,
    STATE_END_PAUSE,
    STATE_OVERSHOOT,
    STATE_SWITCHPIN_OFF,
    STATE_STOPPED
};

// Current state
State currentState = STATE_INITIALIZATION;

int pauseCounter = 0;
int *objectsAngles;

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

bool safeWait(unsigned int millisToWait) {
    unsigned int beginTime = millis();
    while (millis() - beginTime < millisToWait) {
        if (!digitalRead(switchPin)) {
            currentState = STATE_SWITCHPIN_OFF;
            return false;  // Indicate that the switch pin was toggled
        }
    }
    return true;  // Indicate that the wait time elapsed without toggling
}
int arrayBoolSum(bool *arr, int arrCount)
{
    int count = 0;
    for (int i = 0; i < arrCount; i++)
    {
        count += (int)arr[i];
    }
    return count;
}

int calculateWeightedArraySum(const bool array[], int arrSize)
{
    int middleIndex = arrSize / 2; // Calculate middle index

    int value = 0; // Initialize result value

    for (int i = 0; i < arrSize; i++)
    {
        int arrayValue = array[i];           // Get array value at index i
        int positionDelta = i - middleIndex; // Calculate position delta

        value += arrayValue * positionDelta; // Adjust result value
    }

#if DEBUG >= 2
    Serial.print("\nWeighted sum: ");
    Serial.print(value);
#endif
    return value; // Return calculated value
}

void closeGrippers()
{
    moveGripServo(gripClosingAngle);
}

void openGrippers()
{
    moveGripServo(gripOpenAngle);
}

void pushObjectToPreferredPosition()
{
    moveToArmConfiguration(pushingObjectPosition);
    openGrippers();
    delay(1000);

    // Slowly move forward to account for all object placings
    for (int i = 0; i <= 100; i++)
    {
        // Safety check
        if (!digitalRead(switchPin))
        {
            currentState = STATE_SWITCHPIN_OFF;
            return;
        }

        delay(10);
        // Interpolate between pushing position and grabbing position
        moveElbowServo(pushingObjectPosition.elbowAngle - (pushingObjectPosition.elbowAngle - grabbingPosition.elbowAngle)*i/100);
    }
    moveToArmConfiguration(grabbingPosition);
}

double calculatePID(const double desiredValue, const double actualValue, const double Kp, const double Ki, const double Kd)
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
    double error = desiredValue - actualValue;

    // Calculate proportional term
    double proportional = error * Kp;

    // Calculate integral term
    static double integral = 0.0;
    integral += error * dt;
    double integralTerm = Ki * integral;
    integralTerm = constrain(integralTerm, -0.2, 0.2);
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
    static double motorInputs[2]; // [left, right]

    // Clipping the pidOutput between -0.5 and 0.5
    pidOutput = constrain(pidOutput, -0.5, 0.5);

    // Calculating the motor inputs based on a linear relationship
    // and clipping them at a maximum of 1
    motorInputs[0] = min(-4 * pidOutput + 1, 1); // Left motor (non-dominant)
    motorInputs[1] = min(4 * pidOutput + 1, 1);  // Right motor (non-dominant)

    return motorInputs;
}

bool checkOvershoot()
{
    if (isAllZero(car::getAnalogSensorValues(), analogSensorsCount))
    {
        return true;
    }

    return false;
}

bool checkStopPauseSign()
{
    if (!isAllOne(car::getAnalogSensorValues(), analogSensorsCount))
    {
        return false;
    }
    return true;
}

void handleOvershoot()
{
    double speed = 0.1;
#if DEBUG >= 1
    Serial.print("\nDetected overshoot, handling it...");
#endif

    double directionParsed = (double)lastDirection;
    while (isAllZero(car::getAnalogSensorValues(), analogSensorsCount))
    {
        // safety check
        if (!digitalRead(switchPin))
        {
            car::driveMotors(0, 0);
            return;
        }
// turn in last direction we went in until overshoot is resolved
#if DEBUG >= 2
        Serial.print("Overshoot still detected... turning more");
#endif
        if (speed < 0.5)
        {
            speed += 0.1;
        }
        delay(10);
#if DEBUG >= 2
        Serial.println(speed);
#endif
        car::driveMotors(directionParsed * speed, -directionParsed * speed);
    }
}

void driveCar()
{

    bool *sensorValues = car::getAnalogSensorValues();
    double pid = calculatePID(0, calculateWeightedArraySum(sensorValues, analogSensorsCount), Kp, Ki, Kd);
    double *motorInput;
    motorInput = calculateMotorInput(pid);

    Serial.print("\nPID output: ");
    Serial.println(pid);

    car::driveMotors(motorInput[0], motorInput[1]);
}

int *scanField()
{
    static int objectPositions[2] = {-1, -1};
    setStepperSpeed(scanningSpeed);
    // Will keep turn for 1 round
    rotateShoulderRelativeAngle(-360);

    int numberOfObjectsFound = 0;
    while (!hasStepperReachedPosition())
    {
        if(!digitalRead(switchPin)){
            currentState = STATE_SWITCHPIN_OFF;
            return objectPositions;
        }
        if (digitalRead(objectDetectionPin))
        {
            if (numberOfObjectsFound < 2)
            {
                objectPositions[numberOfObjectsFound] = int(getShoulderAngle() - 10);
                numberOfObjectsFound++;
            }
            else
            {
                Serial.print("ERROR: MORE THAN TWO OBJECTS DETECTED");
            }
        }
    }
    setStepperSpeed(stepperMaxSpeed);
    return objectPositions;
}


void moveObjects(){
    moveToArmConfiguration(carryingPosition);
    rotateShoulderAbsoluteAngle((float)objectsAngles[0]);
    
    // Safely waits(switchpin proof) until arm has probably reached positions, tuning needed
    if(!safeWait(500)) return;

    pushObjectToPreferredPosition();
    closeGrippers();
    if(!safeWait(500)) return;
}

void pauseBegin()
{
    int beginTime = millis();
    
    while((millis() - beginTime) < beginPauseDriveForwardTime){
        if (!digitalRead(switchPin)){
            currentState = STATE_SWITCHPIN_OFF;
            return;
        }
        car::driveMotors(0.4, 0.4);
    }
}

void endPause()
{
    // Move forward until line is found to avoid double pause
    while (isAllZero(car::getAnalogSensorValues(), analogSensorsCount) || isAllOne(car::getAnalogSensorValues(), analogSensorsCount))
    {
        // safety check
        if (!digitalRead(switchPin))
        {
            currentState = STATE_SWITCHPIN_OFF;
            return;
        }
        car::driveMotors(1, 1);
    }
}


void updateStateMachine()
{
    switch (currentState)
    {
    case STATE_INITIALIZATION:
        Serial.println("ILLEGAL STATE: STATE INITIALIZATION");
        break;
    case STATE_DRIVING:
        if (!digitalRead(switchPin))
        {
            currentState = STATE_SWITCHPIN_OFF;
            return;
        }
        if (checkOvershoot())
        {
            currentState = STATE_OVERSHOOT;
            break;
        }
        if (checkStopPauseSign())
        {
            if (pauseCounter >= 1)
            {
                Serial.println("Stop sign detected");
                currentState = STATE_STOPPED;
                break;
            }
            currentState = STATE_PAUSE_BEGIN;
            Serial.println("Pause sign detected");
            break;
        }
        driveCar();
        break;
    case STATE_PAUSE_BEGIN:
        pauseBegin();
        currentState = STATE_SCAN_FIELD;
        break;
    case STATE_SCAN_FIELD:
        objectsAngles = scanField();
        break;
    case STATE_MOVE_OBJECTS:
        moveObjects();
        currentState = STATE_END_PAUSE;
        break;
    case STATE_END_PAUSE:
        endPause();
        currentState = STATE_DRIVING;
        break;
    case STATE_OVERSHOOT:
        handleOvershoot();
        currentState = STATE_DRIVING;
        break;
    case STATE_SWITCHPIN_OFF:
        Serial.println("Switchpin off");
        car::driveMotors(0, 0);
        delay(500);
        // if switchpin is turned on, return to driving state
        if (digitalRead(switchPin))
            currentState = STATE_DRIVING;
        break;
    default:
        // Default case should not be reached
        break;
    }
}

void setup()
{
    // Initialization code
    car::setupCarHardware();
    setupMotors();
    currentState = STATE_DRIVING;
}