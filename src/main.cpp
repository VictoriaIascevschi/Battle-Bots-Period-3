#include <Arduino.h>
#include <config.cpp>
#include <ledControl.cpp>
#include <gripperControl.cpp>
#include <linePosition.cpp>
#include <motorControl.cpp>
#include <measureDistance.cpp>
#include <endGameLogic.cpp>
#include <lightSensorCalibration.cpp>
#include <SoftwareSerial.h>

void setup()
{
    // Initialize Bluetooth serial communication
    SoftwareSerial BTSerial(1, 0);
    Serial.begin(9600);         // Start serial communication
    BTSerial.begin(9600);       // Start Bluetooth communication
    NeoPixel.begin();           // Initialize LED strip
    NeoPixel.setBrightness(40); // Set LED brightness

    // Configure motor pins as outputs
    pinMode(MOTOR_B1, OUTPUT);
    pinMode(MOTOR_B2, OUTPUT);
    pinMode(MOTOR_A1, OUTPUT);
    pinMode(MOTOR_A2, OUTPUT);
    // Ensure motors are stopped at startup
    digitalWrite(MOTOR_B1, LOW);
    digitalWrite(MOTOR_B2, LOW);
    digitalWrite(MOTOR_A1, LOW);
    digitalWrite(MOTOR_A2, LOW);

    // Configure ultrasonic sensor pins
    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);

    // Set up wheel encoder interrupts for motor feedback
    attachInterrupt(digitalPinToInterrupt(MOTOR_R1), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR_R2), rightEncoderISR, CHANGE);

    // Configure gripper servo
    pinMode(SERVO, OUTPUT);
    digitalWrite(SERVO, LOW);
}

void loop()
{
    unsigned long currentTime = millis();

    // Display standby color if robot is not calibrated
    if (!robotCalibrated)
    {
        setStandByColor();
    }

    // If game has ended, open gripper and exit loop
    if (gameEnded)
    {
        if (currentTime - previousTime >= GRIPPER_INTERVAL)
        {
            previousTime = currentTime;
            openGripper();
        }
        return;
    }

    // Robot detection logic - detect opponent robot if cone is in position
    if (coneInSquare && !robotDetected)
    {
        static unsigned long detectionStartTime = 0;

        // Check if something is detected within 30cm
        if (measureDistance() < 30)
        {
            if (detectionStartTime == 0)
            {
                detectionStartTime = millis(); // Start timing the detection
            }
            else if (millis() - detectionStartTime >= 350)
            {
                robotDetected = true; // Confirm detection after 350ms
                detectionStartTime = 0;
            }
        }
        else
        {
            detectionStartTime = 0; // Reset detection timer if nothing detected
        }
    }

    // Run sensor calibration after robot detection
    if (robotDetected && !sensorsCalibrated)
    {
        calibrateSensors();

        if (sensorsCalibrated)
        {
            robotCalibrated = true;
        }
        return;
    }

    // Collision avoidance - turn around if obstacle is ahead
    if (gameStarted && !gameEnded)
    {
        float distance = measureDistance();

        if (distance < 12) // Object detected within 12cm
        {
            if (robotState == RobotState::TURNING_LEFT || robotState == RobotState::TURNING_RIGHT)
            {
                robotState = RobotState::TURNING_AROUND;
            }

            turn180(220, 220); // Perform 180° turn to avoid obstacle
            return;
        }
    }

    // Gripper control based on cone state
    if (conePickedUp)
    {
        if (currentTime - previousTime >= GRIPPER_INTERVAL)
        {
            previousTime = currentTime;
            closeGripper(); // Keep gripper closed when holding cone
        }
    }
    else
    {
        if (currentTime - previousTime >= GRIPPER_INTERVAL)
        {
            previousTime = currentTime;
            openGripper(); // Keep gripper open when no cone
        }
    }

    // Mark cone as picked up after calibration
    if (sensorsCalibrated && !conePickedUp)
    {
        conePickedUp = true;
        return;
    }

    // Start game sequence - make initial turn after picking up cone
    if (sensorsCalibrated && !gameStarted && conePickedUp)
    {
        turnLeftMillis(90); // Initial 90° left turn
        if (robotState != RobotState::FOLLOWING_LINE)
            return;
        gameStarted = true;
    }

    // Main line following and navigation logic
    if (gameStarted && !gameEnded)
    {
        getLinePosition(); // Read line sensors

        switch (linePosition)
        {
        case LinePosition::T_JUNCTION:
            turnLeftMillis(90); // Turn left at T-junctions
            break;

        case LinePosition::LEFT:
            turnLeftMillis(70); // Correct course when line is on the left
            break;

        case LinePosition::NO_LINE:
            turnAroundMillis(); // Turn around if line is lost
            break;

        case LinePosition::RIGHT:
            robotState = RobotState::FOLLOWING_LINE;
            moveForwardPID(baseSpeed, baseSpeed, false, true); // Follow line with PID control
            break;

        case LinePosition::CENTER:
            if (robotState == RobotState::FOLLOWING_LINE)
            {
                moveForwardPID(baseSpeed, baseSpeed, false, true); // Move forward on center line
                setForwardColor();                                 // Set LED color for forward movement
            }
            break;

        default:
            break;
        }
    }
}