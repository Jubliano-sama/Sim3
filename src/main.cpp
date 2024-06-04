#include <Arduino.h>
#include "config.h"
#include "motor.h"
#include "carHardware.h"

enum State
{
    STATE_INITIALIZATION,
    STATE_DRIVING,
    STATE_PAUSE_BEGIN,
    STATE_MOVE_OBJECT,
    STATE_END_PAUSE,
    STATE_OVERSHOOT,
    STATE_SWITCHPIN_OFF,
    STATE_STOPPED,
    STATE_FIELD_OBJECT_ROUTINE
};

// Current state
State currentState = STATE_INITIALIZATION;

int pauseCounter = 0;

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

bool safeWait(unsigned int millisToWait)
{
    unsigned int beginTime = millis();
    while (millis() - beginTime < millisToWait)
    {
        if (!digitalRead(switchPin))
        {
            currentState = STATE_SWITCHPIN_OFF;
            return false; // Indicate that the switch pin was toggled
        }
    }
    return true; // Indicate that the wait time elapsed without toggling
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
    moveGripServo(GRIP_CLOSING_ANGLE);
}

void openGrippers()
{
    moveGripServo(GRIP_OPEN_ANGLE);
}

void pushObjectToPreferredPosition()
{
    moveToArmConfiguration(pushingObjectPosition);
    openGrippers();
    if (!safeWait(3000))
        return;

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
        moveElbowServo(pushingObjectPosition.elbowAngle - (pushingObjectPosition.elbowAngle - grabbingPosition.elbowAngle) * i / 100);
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

bool safeWaitUntilStepperStopped()
{
    while (!hasStepperReachedPosition())
    {
        if (!digitalRead(switchPin))
        {
            currentState = STATE_SWITCHPIN_OFF;
            return false; // Indicate that the switch pin was toggled
        }
    }
    return true;
}

// Scans a range of angles for objects
float scanRange(float beginAngle, float endAngle)
{
    const int scanningSpeed = shoulderRotationSteps / secondsPerFullScan;
    float objectAngle = -1;
    setStepperSpeed(scanningSpeed);

    Serial.print("Scanning angle range: ");
    Serial.print(beginAngle);
    Serial.print(" to ");
    Serial.println(endAngle);

    // Move to position that cant hit an object
    moveToArmConfiguration(carryingPosition);
    if (!safeWaitUntilStepperStopped())
    {
        currentState = STATE_SWITCHPIN_OFF;
        return -1;
    }
    closeGrippers();

    rotateShoulderAbsoluteAngle(beginAngle);
    // Wait to reach position safely.
    if (!safeWaitUntilStepperStopped())
    {
        currentState = STATE_SWITCHPIN_OFF;
        return -1;
    }

    moveToArmConfiguration(scanningPosition);
    if (!safeWait(1000))
    {
        currentState = STATE_SWITCHPIN_OFF;
        return -1;
    }
    // Will keep turning from where it last scanned, up to the last object found
    rotateShoulderRelativeAngle(endAngle - beginAngle);

    while (!hasStepperReachedPosition())
    {
        if (!digitalRead(switchPin))
        {
            currentState = STATE_SWITCHPIN_OFF;
            return -1;
        }
        if (digitalRead(OBJECT_DETECTION_PIN))
        {
            float currentAngle = getShoulderAngle();
            stopShoulder();
            objectAngle = currentAngle - SCANNING_OFFSET_ANGLE;
        }
    }
    setStepperSpeed(stepperMaxSpeed);
    return objectAngle;
}

float scanForObject(float lastObjectPlacedAngle = 0)
{
    static float angleScanned = 0;

    Serial.println("Starting scan");

    // Will keep turning from where it last scanned, up to the last object found
    float objectPlace = scanRange(angleScanned, -360.0f + lastObjectPlacedAngle + SCANNING_OFFSET_ANGLE + SCANNING_TOLERANCE);

    // values below 0 mean that the object wasnt found
    if (objectPlace > 0.0f)
    {
        angleScanned = objectPlace;
        return objectPlace - SCANNING_OFFSET_ANGLE;
    }
    else if (currentState == STATE_SWITCHPIN_OFF)
    {
        return -1;
    }

    // Object couldnt be found turning counterclockwise.
    // Proceed by turning clockwise from home
    Serial.println("Object wasnt found counterclockwise, trying clockwise");
    objectPlace = scanRange(0, lastObjectPlacedAngle - SCANNING_OFFSET_ANGLE - SCANNING_TOLERANCE);

    if (objectPlace > 0.0f)
    {
        angleScanned = objectPlace;
        return objectPlace - SCANNING_OFFSET_ANGLE;
    }
    else if (currentState == STATE_SWITCHPIN_OFF)
    {
        return -1;
    }

    Serial.println("ERROR: NO OBJECTS FOUND");
    return -1.0f;
}

void moveObject(float objectAngle, float destinationAngle)
{
    Serial.print("Moving Object from");
    Serial.print(objectAngle);
    Serial.print(" to ");
    Serial.println(destinationAngle);

    // First get to safe position
    openGrippers();
    moveToArmConfiguration(carryingPosition);
    if (!safeWait(1000))
        return;

    rotateShoulderAbsoluteAngle(objectAngle);
    if (!safeWaitUntilStepperStopped())
        return;
    pushObjectToPreferredPosition();
    if (!safeWait(1000))
        return;
    closeGrippers();

    // Move to destination
    rotateShoulderAbsoluteAngle(destinationAngle);
    if (!safeWaitUntilStepperStopped())
        return;
    moveToArmConfiguration(placingPosition);
    if (!safeWait(1000))
        return;
    openGrippers();
    if (!safeWait(500))
        return;
}

void pauseBegin()
{
    Serial.println("Beginning pause");
    int beginTime = millis();

    while ((millis() - beginTime) < beginPauseDriveForwardTime)
    {
        if (!digitalRead(switchPin))
        {
            currentState = STATE_SWITCHPIN_OFF;
            return;
        }
        car::driveMotors(0.4, 0.4);
    }
}

void endPause()
{
    Serial.println("Ending pause");
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

void moveToHome()
{
    // return to home
    openGrippers();
    if (!safeWait(500))
    return;
    moveToArmConfiguration(carryingPosition);
    if (!safeWait(1000))
    {
        return;
    }
    rotateShoulderAbsoluteAngle(0);
    if (!safeWaitUntilStepperStopped())
    {
        return;
    }
    moveToArmConfiguration(homePosition);
    if (!safeWait(500))
        return;
}

void fieldObjectRoutine()
{
    float objectAngle1 = scanForObject(0); // First scan the field
    if (objectAngle1 < 0.0f)
    {
        Serial.println("No objects found, returning");
        return;
    }
    // Displace first object by 180 degrees
    moveObject(objectAngle1, objectAngle1 + 180);
    moveToHome();
    // Wait 3 seconds
    if (!safeWait(3000))
    {
        return;
    }

    float objectAngle2 = scanForObject(objectAngle1); // Scan the field for the second object
    if (objectAngle2 < 0.0f)
    {
        Serial.println("No objects found, returning");
        return;
    }
    // Displace second object by 180 degrees
    moveObject(objectAngle2, objectAngle2 + 180);
    moveToHome();
    moveObject(objectAngle1, 0);
    moveToHome();
    if (!safeWait(3000))
    {
        return;
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
            break;
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
        currentState = STATE_FIELD_OBJECT_ROUTINE;
        break;
    case STATE_FIELD_OBJECT_ROUTINE:
        fieldObjectRoutine();
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
void loop()
{
    updateStateMachine();
}