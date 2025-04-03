/**
 * Function to calibrate light sensors by finding their minimum and maximum values
 * This is done by moving the robot forward while taking sensor readings
 */
void calibrateSensors()
{
    static bool firstRun = true; // Flag to initialize values on first execution
    readSensors();               // Read current sensor values
    setCalibrationColor();       // Set indicator LED color during calibration

    // Initialize min and max arrays on first run only
    if (firstRun)
    {
        Serial.println("Starting sensor calibration...");
        for (int i = 0; i < NUM_SENSORS; i++)
        {
            sensorMin[i] = 1023; // Set min to max possible value
            sensorMax[i] = 0;    // Set max to min possible value
        }
        firstRun = false; // Clear first run flag
    }

    // Check if we've moved far enough to complete calibration
    if (_leftTicks > TARGET || _rightTicks > TARGET)
    {
        stopMotors();             // Stop the robot
        sensorsCalibrated = true; // Mark calibration as complete

        // Print calibration results and calculate thresholds
        Serial.println("Calibration complete. Final values:");
        for (int i = 0; i < NUM_SENSORS; i++)
        {
            // Calculate threshold as average of min and max
            sensorThreshold[i] = (sensorMin[i] + sensorMax[i]) / 2;
            Serial.print("Sensor ");
            Serial.print(i);
            Serial.print(": Min=");
            Serial.print(sensorMin[i]);
            Serial.print(", Max=");
            Serial.print(sensorMax[i]);
            Serial.print(", Threshold=");
            Serial.println(sensorThreshold[i]);
        }
        return;
    }

    // Update min and max values for each sensor
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        int sensorValue = analogRead(sensorPins[i]);

        // Update minimum if current reading is lower
        if (sensorValue < sensorMin[i])
        {
            sensorMin[i] = sensorValue;
        }
        // Update maximum if current reading is higher
        if (sensorValue > sensorMax[i])
        {
            sensorMax[i] = sensorValue;
        }
    }

    // Continue moving forward during calibration using PID control
    moveForwardPID(200, 200, true, false);
}