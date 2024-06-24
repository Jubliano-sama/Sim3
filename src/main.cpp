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
    unsigned long beginTime = millis();
    while (millis() - beginTime < millisToWait)
    {
        if (!digitalRead(SWITCHPIN))
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
    interpolateToArmConfiguration(ArmConfiguration(-1,-1, pushingObjectPosition.wristAngle,-1), 500);
    interpolateToArmConfiguration(pushingObjectPosition, 1000);
    if (!safeWait(1000))
        return;

    // Slowly move forward to account for all object placings
    interpolateToArmConfiguration(pushingObjectPosition2, 2000);
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
        if (!digitalRead(SWITCHPIN))
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

    car::driveMotors(motorInput[0]*driveSpeedMultiplier, motorInput[1]*driveSpeedMultiplier);
}

bool safeWaitUntilStepperStopped()
{
    while (!hasStepperReachedPosition())
    {
        if (!digitalRead(SWITCHPIN))
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
    float sum = 0;
    int count = 0;

    if(!moveToPositionSafely(carryingPosition, 1000) || !rotateShoulderSafely(0) || !moveToPositionSafely(scanningPosition, 1000)){
        currentState = STATE_SWITCHPIN_OFF;
        return -1;
    }

    int i = 0;
    while (i < 500)
    {
        sum += (float)objectSensor.readRange();
        count++;
        delay(1);
        i++;
    }
    Serial.print("Ambient value measured at: ");
    float average = sum / (float)count;
    Serial.println(average);
    return average;
}

float *updateAndGetRollingScanBuffer(float *lastBuffer, int lastBufferSize, float newValue)
{
    float previousLastScan = lastBuffer[lastBufferSize-1];
    // Shift elements
    for (int i = 1; i < lastBufferSize; i++)
    {
        lastBuffer[i] = lastBuffer[i - 1];
    }
    lastBuffer[0] = (newValue+previousLastScan)/2.0f;
    return lastBuffer;
}

// Scans a range of angles for objects
float scanRange(float beginAngle, float endAngle, int ambientValue)
{
    Serial.print("Scanning angle range: ");
    Serial.print(beginAngle);
    Serial.print(" to ");
    Serial.println(endAngle);

    const float threshold = ambientValue - SCANNING_THRESHOLD_MM;
    const int bufferSize = 90;
    // prepare rolling scanning buffer
    float tempScanBuffer[bufferSize];
    for (int i = 0; i < bufferSize; i++)
    {
        tempScanBuffer[i] = ambientValue;
    }
    float *scanBufferPointer = tempScanBuffer;

    const int scanningSpeed = shoulderRotationSteps / secondsPerFullScan;
    float objectAngle = -1;
    setStepperSpeed(scanningSpeed);
    // first move to carryingposition then rotateshoulder then move to scanning position
    if (!moveToPositionSafely(carryingPosition, 1000) || !rotateShoulderSafely(beginAngle))
    {
        currentState = STATE_SWITCHPIN_OFF;
        return -1;
    }


    interpolateToArmConfiguration(scanningPosition, 1000);
    delay(1000);
    rotateShoulderRelativeAngle(endAngle - beginAngle);
    
    float previousShoulderAngle = getShoulderAngle();
    
    const unsigned long delayBetweenStepsMicros = ((secondsPerFullScan * 1e6)/shoulderRotationSteps)/2;
    const unsigned long measurementsPerStep = (float)bufferSize / (SCANNING_ROLLING_AVERAGE_ANGLE / (360.0f / shoulderRotationSteps));
    const unsigned long delayBetweenMeasurements = delayBetweenStepsMicros / measurementsPerStep;
    unsigned int sum = bufferSize*ambientValue;
    unsigned int lastUpdatedIndex = bufferSize;

    while (!hasStepperReachedPosition())
    {
        if(getShoulderAngle() == previousShoulderAngle){
            unsigned long beginTime = micros();
            int currentMeasurement = objectSensor.readRange();
            if (lastUpdatedIndex <= 0) {
                lastUpdatedIndex = bufferSize;
            }
            sum -= scanBufferPointer[lastUpdatedIndex - 1];
            sum += currentMeasurement;
            scanBufferPointer[lastUpdatedIndex-1] = currentMeasurement;
            lastUpdatedIndex--;
            delayMicroseconds(delayBetweenMeasurements - (micros() - beginTime));
            continue;
        }
        previousShoulderAngle = getShoulderAngle();

        if (!digitalRead(SWITCHPIN))
        {
            currentState = STATE_SWITCHPIN_OFF;
            return -1;
        }

        float rollingAverage = (sum / (float)bufferSize);
        Serial.print("Rolling average: ");
        Serial.println(rollingAverage);
        if (rollingAverage < threshold)
        {
            float currentAngle = getShoulderAngle();
            objectAngle = currentAngle - SCANNING_OFFSET_ANGLE;
            break;
        }
    }

    stopShoulder();
    setStepperSpeed(stepperMaxSpeed);
    Serial.print("Found object at: ");
    Serial.println(objectAngle);
    return objectAngle;
}

void scanWholeField(float startingAngle, ArmConfiguration scanningPos, float* averagesArray, float* whereAverageWasMeasuredArray, int arraySize)
{
    const int scanningSpeed = shoulderRotationSteps / secondsPerFullScan;

    for (int i =0; i < arraySize; i++){
        averagesArray[i] = 99999999;
        whereAverageWasMeasuredArray[i] = 180;
    }
    setStepperSpeed(scanningSpeed);

    // first move to carryingposition then rotateshoulder then move to scanning position
    if (!moveToPositionSafely(carryingPosition, 1000) || !rotateShoulderSafely(startingAngle))
    {
        currentState = STATE_SWITCHPIN_OFF;
        return;
    }


    interpolateToArmConfiguration(scanningPos, 1000);
    delay(1000);
    float previousShoulderAngle = startingAngle;
    rotateShoulderRelativeAngle(360-2*startingAngle);
    int index = 0;
    while(!hasStepperReachedPosition()){
        unsigned long sum = 0;
        unsigned int amountOfValues = 0;
        while((getShoulderAngle() - previousShoulderAngle < 2.5f) && !hasStepperReachedPosition()){
            sum += objectSensor.readRange();
            amountOfValues++;
        }
        previousShoulderAngle = getShoulderAngle();
        whereAverageWasMeasuredArray[index] = previousShoulderAngle;
        float average = (float)sum/(float)amountOfValues;
        averagesArray[index] = average;
        Serial.print("Average: ");
        Serial.println(average);
        Serial.println(whereAverageWasMeasuredArray[index]);
        index++;
    }
    
    setStepperSpeed(stepperMaxSpeed);
    return; // Return the array of averages
}

// Returns a pointer to an array containing the objects angles in chronological order
float *scanForObject()
{
    static float objectAngles[2] = {-1, -1};

    Serial.println("Starting scan");
    float scanValues[155];
    float whereValueWasMeasured[155];
    scanWholeField(32.0f, scanningPosition, scanValues, whereValueWasMeasured, 155);

    rotateShoulderRelativeAngle(-shoulderRotationSteps);
    safeWaitUntilStepperStopped();

    float scanValues2[155];
    scanWholeField(32.0f, scanningPosition2, scanValues2, whereValueWasMeasured, 155);

    Serial.println("Scan Completed");
    float lowestValue = 999999;
    int lowestValueIndex = -1;
    for(int i = 1; i < 154; i++){
        float generalArea = pow(scanValues[i-1] + scanValues[i] + scanValues[i+1], 0.91) + scanValues2[i-1] + scanValues2[i] + scanValues2[i+1];
        if(generalArea < lowestValue){
            lowestValue = generalArea;
            lowestValueIndex = i;
        }
    }
    Serial.print("Index: ");
    Serial.println(lowestValueIndex);
    Serial.print("Angle: ");
    Serial.println(whereValueWasMeasured[lowestValueIndex]);
    objectAngles[0] = whereValueWasMeasured[lowestValueIndex] + SCANNING_OFFSET_ANGLE;
    for (int i = constrain(lowestValueIndex -4, 0, 180); i < constrain(lowestValueIndex + 4, 0, 180); i++ ){
        scanValues[i] = 999999;
    }
    lowestValue = 99999999;
    lowestValueIndex = -1;

    for(int i = 1; i < 154; i++){
        float generalArea = pow(scanValues[i-1] + scanValues[i] + scanValues[i+1], 0.91) + scanValues2[i-1] + scanValues2[i] + scanValues2[i+1];
        if(generalArea < lowestValue){
            lowestValue = generalArea;
            lowestValueIndex = i;
        }
    }
    Serial.print("Index: ");
    Serial.println(lowestValueIndex);
    Serial.print("Angle: ");
    Serial.println(whereValueWasMeasured[lowestValueIndex]);
    objectAngles[1] = whereValueWasMeasured[lowestValueIndex] + SCANNING_OFFSET_ANGLE;
    
    Serial.println(objectAngles[0]);
    Serial.println(objectAngles[1]);
    return objectAngles;
}

void moveObject(float objectAngle, float destinationAngle, bool openGripperAtEnd)
{
    Serial.print("Moving Object from");
    Serial.print(objectAngle);
    Serial.print(" to ");
    Serial.println(destinationAngle);

    // First get to safe position
    openGrippers();
    interpolateToArmConfiguration(carryingPosition, 1000);
    if(!rotateShoulderSafely(objectAngle)){
        return;
    }
    pushObjectToPreferredPosition();
    interpolateToArmConfiguration(ArmConfiguration(grabbingPosition.shoulderAngle +27 , grabbingPosition.elbowAngle+50, 160), 1500);
    delay(500);
    openGrippers();
    delay(1000);
    interpolateToArmConfiguration(grabbingPosition, 1000);
    delay(1000);
    closeGrippers();
    delay(1000);
    interpolateToArmConfiguration(carryingPosition, 500);
    // Move to destination
    if(!rotateShoulderSafely(destinationAngle)){
        return;
    }
    if(openGripperAtEnd) interpolateToArmConfiguration(placingPosition, 1000);
    delay(1000);
    if(openGripperAtEnd) openGrippers();
    if (!safeWait(1000))
        return;
}

void beginPause()
{
    Serial.println("Beginning pause");
    int beginTime = millis();

    while ((millis() - beginTime) < BEGIN_PAUSE_DRIVEFORWARD_MS)
    {
        if (!digitalRead(SWITCHPIN))
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
    // Move forward for set amount of time so that discrepancies are avoided
    
    //while (isAllZero(car::getAnalogSensorValues(), analogSensorsCount) || isAllOne(car::getAnalogSensorValues(), analogSensorsCount))

    unsigned long beginTime = millis();
    while((millis() - beginTime) < END_PAUSE_DRIVEFORWARD_MS)
    {
        // safety check
        if (!digitalRead(SWITCHPIN))
        {
            currentState = STATE_SWITCHPIN_OFF;
            return;
        }
        car::driveMotors(1*driveSpeedMultiplier, 1*driveSpeedMultiplier);
    }
}

void moveToHome()
{
    if (!safeWait(500))
        return;
    interpolateToArmConfiguration(carryingPosition, 1000);
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
    interpolateToArmConfiguration(carryingPosition, 500);
    // Displace first object by 180 degrees
    moveObject(objectAngles[0], objectAngles[0] + 180,true);
    moveToHome();
    // Wait 3 seconds
    if (!safeWait(3000))
    {
        return;
    }

    // Displace second object by 180 degrees
    moveObject(objectAngles[1], objectAngles[1] + 180, true);
    moveToHome();
    moveObject(objectAngles[0]+180, 0, false);
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
        if (!digitalRead(SWITCHPIN))
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
            pauseCounter++;
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
        stopShoulder();
        delay(500);
        // if switchpin is turned on, return to driving state
        if (digitalRead(SWITCHPIN)){
            currentState = STATE_DRIVING;
            pauseCounter = 0;
        }
        break;
    case STATE_STOPPED:
        {
        unsigned long beginMillis = millis();
        while ((millis() - beginMillis) < STOP_DRIVE_FORWARD_MS)
        {
            if (!digitalRead(SWITCHPIN))
            {
                currentState = STATE_SWITCHPIN_OFF;
                break;
            }
            car::driveMotors(0.4, 0.4);
        }
        car::driveMotors(0, 0);
        rotateShoulderSafely(270);
        interpolateToArmConfiguration(placingPosition,1000);
        delay(1000);
        openGrippers();
        delay(500);
        interpolateToArmConfiguration(carryingPosition, 1000);
        moveToHome();
        while(digitalRead(SWITCHPIN));
        currentState = STATE_SWITCHPIN_OFF;
        break;
        }
    default:
        // Default case should not be reached
        break;
    }

}

void setup()
{
    pinMode(SWITCHPIN, INPUT_PULLUP);
    // Initialization code
    car::setupCarHardware();
    setupMotors();
    moveToArmConfiguration(homePosition);
    Serial.begin(115200); // Start Serial at 115200bps
    Wire.begin();         // Start I2C library
    delay(100);           // delay .1s

    if (!objectSensor.begin())
    {
        Serial.println("Failed to initialize. Freezing..."); // Initialize device and check for errors
        while (true)
            ;
    }
    currentState = STATE_DRIVING;

    delay(1000);
}

void loop()
{
    moveObject(90, 270, true);
    moveObject(270, 90, true);
}