/**
 * Moves the robot forward at specified speeds
 * @param _rightSpeed Speed for the right motor (0-255)
 * @param _leftSpeed Speed for the left motor (0-255)
 */
void moveForward(int _rightSpeed, int _leftSpeed)
{
    if (!gameEnded) // Only move if game is still active
    {
        analogWrite(MOTOR_B1, _rightSpeed); // Right motor forward
        analogWrite(MOTOR_A1, _leftSpeed);  // Left motor forward
    }
}

/**
 * Moves the robot backward at specified speeds
 * @param _rightSpeed Speed for the right motor (0-255)
 * @param _leftSpeed Speed for the left motor (0-255)
 */
void moveBackward(int _rightSpeed, int _leftSpeed)
{
    if (!gameEnded) // Only move if game is still active
    {
        analogWrite(MOTOR_B2, _rightSpeed); // Right motor backward
        analogWrite(MOTOR_A2, _leftSpeed);  // Left motor backward
    }
}

/**
 * Stops all motors immediately
 */
void stopMotors()
{
    analogWrite(MOTOR_B1, 0); // Right forward off
    analogWrite(MOTOR_B2, 0); // Right backward off
    analogWrite(MOTOR_A1, 0); // Left forward off
    analogWrite(MOTOR_A2, 0); // Left backward off
}

/**
 * Makes the robot turn in place (rotate 180 degrees)
 * @param _rightSpeed Speed for the right motor
 * @param _leftSpeed Speed for the left motor
 */
void turn180(int _rightSpeed, int _leftSpeed)
{
    analogWrite(MOTOR_B2, 0);           // Right backward off
    analogWrite(MOTOR_A1, 0);           // Left forward off
    analogWrite(MOTOR_B1, _rightSpeed); // Right forward on
    analogWrite(MOTOR_A2, _leftSpeed);  // Left backward on
}

/**
 * Turns the robot left by a specified angle using wheel encoders
 * @param angle Angle in degrees to turn
 */
void turnLeftMillis(int angle)
{
    static unsigned long lastCheck = 0;
    const unsigned long checkInterval = 5;
    static int targetPulses;

    if (robotState != RobotState::TURNING_LEFT)
    {
        // Initialize turn operation
        resetTicks();
        targetPulses = 0;
        // Calculate required wheel movement based on angle
        float turnDistance = (angle / 360.0) * turn_Circumference;
        targetPulses = (turnDistance / WHEEL_CIRCUMFERENCE) * PULSE_PER_REVOLUTION;

        stopMotors();

        moveForward(0, 200); // Only left wheel moves
        setTurnLeftColor();  // Update LED indicator
        robotState = RobotState::TURNING_LEFT;
        motionComplete = false;
    }

    if (robotState == RobotState::TURNING_LEFT)
    {
        lastCheck = millis();
        // Check if we've reached the desired angle
        if (_leftTicks >= targetPulses)
        {
            stopMotors();
            robotState = RobotState::FOLLOWING_LINE;
            motionComplete = true;
            linePosition = LinePosition::CENTER;
        }
    }
}

/**
 * Turns the robot right by a specified angle using wheel encoders
 * @param angle Angle in degrees to turn
 */
void turnRightMillis(int angle)
{
    static unsigned long lastCheck = 0;
    const unsigned long checkInterval = 5;
    static int targetPulses;

    if (robotState != RobotState::TURNING_RIGHT)
    {
        // Initialize turn operation
        resetTicks();
        targetPulses = 0;
        // Calculate required wheel movement based on angle
        float turnDistance = (angle / 360.0) * turn_Circumference;
        targetPulses = (turnDistance / WHEEL_CIRCUMFERENCE) * PULSE_PER_REVOLUTION;
        stopMotors();

        moveForward(200, 0); // Only right wheel moves
        setTurnRightColor(); // Update LED indicator
        robotState = RobotState::TURNING_RIGHT;
        motionComplete = false;
    }

    if (robotState == RobotState::TURNING_RIGHT && millis() - lastCheck >= checkInterval)
    {
        lastCheck = millis();
        readSensors();
        // Check if we've reached the desired angle
        if (_rightTicks >= targetPulses)
        {
            stopMotors();
            robotState = RobotState::FOLLOWING_LINE;
            motionComplete = true;
            linePosition = LinePosition::CENTER;
        }
    }
}

/**
 * Performs a 180-degree turn until a line is detected
 */
void turnAroundMillis()
{
    static unsigned long lastCheck = 0;
    const unsigned long checkInterval = 5;
    static int targetPulses;

    if (robotState != RobotState::TURNING_AROUND)
    {
        // Initialize turn around operation
        resetTicks();
        targetPulses = 0;

        // Calculate the distance for a 180-degree turn
        float turnDistance = (3.14 * (DISTANCE_BETWEEN_WHEELS / 2));
        targetPulses = (turnDistance / WHEEL_CIRCUMFERENCE) * PULSE_PER_REVOLUTION;

        turn180(200, 200);    // Both wheels moving in opposite directions
        setTurnAroundColor(); // Update LED indicator

        robotState = RobotState::TURNING_AROUND;
        motionComplete = false;
    }

    if (robotState == RobotState::TURNING_AROUND)
    {
        // Check if line detected (stop turning when we've found a line)
        sensorValues[0] = analogRead(sensorPins[0]);
        if (sensorValues[0] > sensorThreshold[0] || sensorValues[4] > sensorThreshold[4])
        {
            stopMotors();
            robotState = RobotState::FOLLOWING_LINE;
            motionComplete = true;
            linePosition = LinePosition::CENTER;
        }
    }
}

/**
 * Moves forward using PID control for straight movement or line following
 * @param _leftSpeed Base speed for left motor
 * @param _rightSpeed Base speed for right motor
 * @param withOutLine If true, use encoder feedback for straight movement
 * @param lineTracking If true, follow a line using sensors
 */
void moveForwardPID(int _leftSpeed, int _rightSpeed, bool withOutLine, bool lineTracking)
{
    if (withOutLine)
    {
        // PID for straight movement using wheel encoders
        Kp = 1.0;  // Proportional gain
        Ki = 0.01; // Integral gain
        Kd = 1.0;  // Derivative gain

        error = _leftTicks - _rightTicks; // Error is the difference in wheel rotation

        integral += error;
        derivative = error - lastError;
        lastError = error;

        integral = constrain(integral, -10, 10); // Prevent integral windup
    }
    else if (lineTracking)
    {
        // PID for line following using sensors
        readSensors();
        int position = calculateLinePosition();

        Kp = 0.4;     // Proportional gain
        Ki = 0.00001; // Integral gain
        Kd = 0.1;     // Derivative gain

        int center = (NUM_SENSORS - 1) * 1000 / 2;
        error = position - center; // Error is deviation from center line

        integral += error;
        derivative = error - lastError;
        lastError = error;
    }

    // Calculate correction and apply to motor speeds
    correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
    _leftSpeed -= correction;
    _rightSpeed += correction;

    // Ensure speeds stay within valid range
    _leftSpeed = constrain(_leftSpeed, 0, 255);
    _rightSpeed = constrain(_rightSpeed, 0, 255);

    moveForward(_leftSpeed, _rightSpeed);
}

/**
 * Interrupt Service Routine for left wheel encoder
 * Increments the left tick counter when a pulse is detected
 */
void leftEncoderISR()
{
    static unsigned long timer;
    if (millis() > timer)
    {
        _leftTicks++;
        timer = millis() + ISR_INTERVAL; // Debounce
    }
}

/**
 * Interrupt Service Routine for right wheel encoder
 * Increments the right tick counter when a pulse is detected
 */
void rightEncoderISR()
{
    static unsigned long timer;
    if (millis() > timer)
    {
        _rightTicks++;
        timer = millis() + ISR_INTERVAL; // Debounce
    }
}