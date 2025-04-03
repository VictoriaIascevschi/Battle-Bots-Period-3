/**
 * Handles the end game sequence when the robot completes its task
 */
void endGame()
{
    // Only execute if the game hasn't already ended
    if (!gameEnded)
    {
        // Stop all motors first
        stopMotors();

        // Record current time for timing the backward movement
        unsigned long startTime = millis();

        // Move backward for 1 second (1000ms)
        while (millis() < startTime + 1000)
        {
            openGripper();          // Release any object being held
            setBackwardColor();     // Set LED indicators to backward movement color
            moveBackward(255, 255); // Move backward at full speed
        }

        // Stop all motors after completing backward movement
        stopMotors();
        setStopColor(); // Set LED indicators to stopped state color

        // Update game state flags
        coneDroppedOff = true; // Mark the cone as successfully dropped off
        gameEnded = true;      // Mark the game as completed
        motionComplete = true; // Mark all motion as completed
    }
}