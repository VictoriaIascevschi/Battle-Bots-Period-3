/**
 * Control the gripper servo with a specific pulse width
 * @param pulse The pulse width in microseconds, or 0 to use previous value
 */
void gripper(int pulse)
{
    static unsigned long timer; // Tracks when to send the next pulse
    static int lastPulse;       // Stores the last valid pulse width

    if (millis() > timer) // Only execute if 20ms has passed since last pulse
    {
        if (pulse > 0)
        {
            lastPulse = pulse; // Store new pulse width if valid
        }
        else
        {
            pulse = lastPulse; // Use previous pulse width if input is 0 or negative
        }

        // Generate the PWM signal manually
        digitalWrite(SERVO, HIGH);
        delayMicroseconds(pulse); // High signal duration sets servo position
        digitalWrite(SERVO, LOW);
        timer = millis() + 20; // Schedule next pulse in 20ms
    }
}

/**
 * Close the gripper by setting servo to the close position
 */
void closeGripper()
{
    gripper(GRIPPER_CLOSE); // Use predefined pulse width for closed position
}

/**
 * Open the gripper by setting servo to the open position
 */
void openGripper()
{
    gripper(GRIPPER_OPEN); // Use predefined pulse width for open position
}
