#include <Arduino.h>
#include <SPI.h>
#include "config.h"
#include "motor.h"
#include "carHardware.h"
#include <Wire.h>
#include <Adafruit_VL6180X.h>

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

Adafruit_VL6180X objectSensor = Adafruit_VL6180X();

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
        if (speed < 0.8)
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

bool moveToPositionSafely(const ArmConfiguration& config, unsigned int waitTime = 0)
{
    moveToArmConfiguration(config);
    return waitTime == 0 ? safeWaitUntilStepperStopped() : safeWait(waitTime);
}

bool rotateShoulderSafely(float angle)
{
    rotateShoulderAbsoluteAngle(angle);
    return safeWaitUntilStepperStopped();
}

float measureAmbientValue()
{
    unsigned long startTime = millis();
    float sum = 0;
    int count = 0;

    if(!moveToPositionSafely(carryingPosition, 1000) || !rotateShoulderSafely(0) || !moveToPositionSafely(scanningPosition, 1000)){
        currentState = STATE_SWITCHPIN_OFF;
        return -1;
    }

    while (millis() - startTime < 1000)
    {
        sum += objectSensor.readRange();
        count++;
        delay(1); // 10ms delay for ~100 samples over 1 second
    }
    Serial.print("Ambient value measured at: ");
    float average = sum / count;
    Serial.println(average);
    return average;
}

int *updateAndGetRollingBuffer(int *lastBuffer, int lastBufferSize, float newValue)
{
    // Shift elements
    for (int i = 1; i < lastBufferSize; i++)
    {
        lastBuffer[i] = lastBuffer[i - 1];
    }
    lastBuffer[0] = newValue;
    return lastBuffer;
}

// Scans a range of angles for objects
float scanRange(float beginAngle, float endAngle, int ambientValue)
{
    const float threshold = ambientValue - SCANNING_THRESHOLD_MM;
    const int bufferSize = 25;
    // prepare rolling scanning buffer
    int tempScanBuffer[bufferSize];
    for (int i = 0; i < bufferSize; i++)
    {
        tempScanBuffer[i] = ambientValue;
    }
    int *scanBufferPointer = tempScanBuffer;

    const int scanningSpeed = shoulderRotationSteps / secondsPerFullScan;
    float objectAngle = -1;
    setStepperSpeed(scanningSpeed);

    Serial.print("Scanning angle range: ");
    Serial.print(beginAngle);
    Serial.print(" to ");
    Serial.println(endAngle);

    // first move to carryingposition then rotateshoulder then move to scanning position
    if (!moveToPositionSafely(carryingPosition, 1000) || !rotateShoulderSafely(beginAngle) || !moveToPositionSafely(scanningPosition, 1000))
    {
        currentState = STATE_SWITCHPIN_OFF;
        return -1;
    }


    rotateShoulderRelativeAngle(endAngle - beginAngle);

    while (!hasStepperReachedPosition())
    {
        if (!digitalRead(switchPin))
        {
            currentState = STATE_SWITCHPIN_OFF;
            return -1;
        }

        int currentMeasurement = objectSensor.readRange();
        scanBufferPointer = updateAndGetRollingBuffer(scanBufferPointer, bufferSize, currentMeasurement);
        int sum = 0;
        for (int i = 0; i < bufferSize; i++)
        {
            sum += scanBufferPointer[i];
        }
        float rollingAverage = (float)(sum / bufferSize);

        if (rollingAverage < threshold)
        {
            float currentAngle = getShoulderAngle();
            objectAngle = currentAngle - SCANNING_OFFSET_ANGLE;
        }
        delay(2); // Ensure this delay for updating rolling average
    }

    stopShoulder();
    setStepperSpeed(stepperMaxSpeed);
    Serial.print("Found object at: ");
    Serial.println(objectAngle);
    return objectAngle;
}

float *scanForObject()
{
    static float objectAngles[2] = {-1, -1};

    Serial.println("Starting scan");

    int ambientValue = measureAmbientValue();
    // if switch pin is off
    if (ambientValue < 0)
        return objectAngles;

    // Find first object
    float objectPlace = scanRange(0, 360.0f, ambientValue);

    if (objectPlace > 0.0f)
    {
        objectAngles[0] = objectPlace - SCANNING_OFFSET_ANGLE;
    }
    else
    {
        Serial.println("No object found, returning");
        return objectAngles;
    }
    // Find second object

    objectPlace = scanRange(objectPlace, 360.0f, ambientValue);

    if (objectPlace > 0.0f)
    {
        objectAngles[0] = objectPlace - SCANNING_OFFSET_ANGLE;
    }
    else
    {
        Serial.println("No object found, returning");
        return objectAngles;
    }

    return objectAngles;
}

void moveObject(float objectAngle, float destinationAngle)
{
    Serial.print("Moving Object from");
    Serial.print(objectAngle);
    Serial.print(" to ");
    Serial.println(destinationAngle);

    // First get to safe position
    openGrippers();
    if(!moveToPositionSafely(carryingPosition, 1000) || !rotateShoulderSafely(objectAngle)){
        return;
    }
    pushObjectToPreferredPosition();
    if (!safeWait(1000))
        return;
    closeGrippers();

    // Move to destination
    if(!rotateShoulderSafely(destinationAngle) || !moveToPositionSafely(placingPosition, 1000)){
        return;
    }
    openGrippers();
    if (!safeWait(500))
        return;
}

void beginPause()
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

    // at last stop the car
    car::driveMotors(0, 0);
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
    if(!moveToPositionSafely(carryingPosition, 1000) || !rotateShoulderSafely(0) || !moveToPositionSafely(homePosition, 1000)){
        return;
    }
}

void fieldObjectRoutine()
{
    float *objectAngles = scanForObject();
    if (objectAngles[0] < 0.0f || objectAngles[1] < 0.0f)
    {
        Serial.println("One or less objects found, returning");
        return;
    }
    // Displace first object by 180 degrees
    moveObject(objectAngles[0], objectAngles[0] + 180);
    moveToHome();
    // Wait 3 seconds
    if (!safeWait(3000))
    {
        return;
    }

    // Displace second object by 180 degrees
    moveObject(objectAngles[1], objectAngles[1] + 180);
    moveToHome();
    moveObject(objectAngles[0], 0);
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
        beginPause();
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
    Serial.begin(115200); // Start Serial at 115200bps
    Wire.begin();         // Start I2C library
    delay(100);           // delay .1s

    if (!objectSensor.begin())
    {
        Serial.println("Failed to initialize. Freezing..."); // Initialize device and check for errors
        while (1)
            ;
    }
    currentState = STATE_DRIVING;

    delay(1000);
}
void loop()
{
    moveToArmConfiguration(homePosition);
}