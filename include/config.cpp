#include <Adafruit_NeoPixel.h>

// Robot state defines the current operational mode
enum RobotState
{
    FOLLOWING_LINE, // Robot is following the line
    TURNING_LEFT,   // Robot is executing a left turn
    TURNING_RIGHT,  // Robot is executing a right turn
    TURNING_AROUND, // Robot is turning 180 degrees
};

// Line position defines where the line is relative to the robot
enum LinePosition
{
    T_JUNCTION, // Robot is at a T-junction
    LEFT,       // Line is to the left of the robot
    RIGHT,      // Line is to the right of the robot
    NO_LINE,    // No line detected
    CENTER      // Line is centered under the robot
};

// Current state variables
RobotState robotState = RobotState::FOLLOWING_LINE;
LinePosition linePosition = LinePosition::CENTER;

// Pin definitions
const int TRIG = 13;       // Ultrasonic sensor trigger pin
const int ECHO = 12;       // Ultrasonic sensor echo pin
const int MOTOR_B1 = 6;    // Motor B forward
const int MOTOR_B2 = 5;    // Motor B backward
const int MOTOR_A1 = 11;   // Motor A forward
const int MOTOR_A2 = 10;   // Motor A backward
const int SERVO = 7;       // Servo motor pin
const int NEOPIXEL_IN = 4; // NeoPixel data pin
const int MOTOR_R1 = 2;    // Rotation sensor A
const int MOTOR_R2 = 3;    // Rotation sensor B

int baseSpeed = 255; // Default motor speed

// Line sensor configuration
const int NUM_SENSORS = 8;                                      // Number of line sensors
int sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7}; // Pins for sensors
int sensorValues[NUM_SENSORS];                                  // Current readings
int sensorMin[NUM_SENSORS];                                     // Min calibration values
int sensorMax[NUM_SENSORS];                                     // Max calibration values
int sensorThreshold[NUM_SENSORS];                               // Threshold for black/white detection

// Line detection flags
bool leftTurn;        // Indicates a left turn is detected
bool rightTurn;       // Indicates a right turn is detected
bool tJunctionOrBase; // Indicates T-junction or base is detected
bool deadEnd;         // Indicates a dead end is detected

// Robot physical characteristics and positioning constants
const float WHEEL_CIRCUMFERENCE = 20.4;           // Circumference of wheels in cm
const int PULSE_PER_REVOLUTION = 20;              // Encoder pulses per wheel revolution
const float DISTANCE_BETWEEN_WHEELS = 22.75;      // Distance between wheels in cm
static const int DISTANCE_FROM_BASE_TO_CONE = 55; // Distance from starting position to cone in cm
const int TARGET = DISTANCE_FROM_BASE_TO_CONE;    // Target distance to travel

// Encoder tick counters
volatile signed int _leftTicks = 0;  // Left wheel encoder ticks
volatile signed int _rightTicks = 8; // Right wheel encoder ticks

// NeoPixel LED configuration
const int NUM_PIXELS = 4;         // Number of NeoPixels
const int PIXEL_BOTTOM_LEFT = 0;  // Bottom left pixel index
const int PIXEL_BOTTOM_RIGHT = 1; // Bottom right pixel index
const int PIXEL_TOP_RIGHT = 2;    // Top right pixel index
const int PIXEL_TOP_LEFT = 3;     // Top left pixel index
Adafruit_NeoPixel NeoPixel = Adafruit_NeoPixel(NUM_PIXELS, NEOPIXEL_IN, NEO_RGB + NEO_KHZ800);

// Gripper servo configuration
const int GRIPPER_OPEN = 1800;   // PWM value for open gripper
const int GRIPPER_CLOSE = 990;   // PWM value for closed gripper
const int PULSE = 2000;          // Servo pulse duration
int previousTime = 0;            // Last time the gripper was updated
const int GRIPPER_INTERVAL = 20; // Time between gripper updates in ms

// Interrupt service routine interval
const int ISR_INTERVAL = 20; // Time between ISR calls in ms

// Robot state flags
bool coneInSquare = true;         // Whether the cone is in the square
bool sensorsCalibrated = false;   // Whether sensors have been calibrated
bool conePickedUp = false;        // Whether the cone has been picked up
bool gameStarted = false;         // Whether the game has started
bool coneDroppedOff = false;      // Whether the cone has been dropped off
bool gameEnded = false;           // Whether the game has ended
bool motionComplete = true;       // Whether the current motion is complete
bool robotDetected = false;       // Whether another robot has been detected
bool blackSquareDetected = false; // Whether a black square has been detected
bool robotCalibrated = false;     // Whether the robot has been calibrated

// Black square detection timing
const int MIN_SQUARE_TIME = 2000; // Minimum time to confirm black square in ms

// PID control variables for line following
int error = 0, lastError = 0; // Current and previous error
float integral = 0;           // Integral term for PID
float derivative = 0;         // Derivative term for PID
float Kp;                     // Proportional gain
float Ki;                     // Integral gain
float Kd;                     // Derivative gain
int correction;               // Correction value for steering

// Turn calculation variables
int pulses;                                 // Number of pulses for turn
int angle;                                  // Angle to turn
int radius = DISTANCE_BETWEEN_WHEELS;       // Turn radius (distance between wheels)
int turn_Circumference = 2 * 3.14 * radius; // Circumference of turn
float turnDistances = 0;                    // Distance to travel for turn

/**
 * Resets the encoder tick counters to zero
 */
void resetTicks()
{
    _leftTicks = 0;
    _rightTicks = 0;
}
