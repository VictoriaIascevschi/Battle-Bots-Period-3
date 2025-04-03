/**
 * Reads all analog values from the line sensors
 */
void readSensors()
{
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        sensorValues[i] = analogRead(sensorPins[i]);
    }
}

/**
 * Calculates the position of the line based on weighted sensor values
 * @return Position value (higher values indicate line is more to the right)
 */
int calculateLinePosition()
{
    long weightedSum = 0;
    long sum = 0;

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        int value = sensorValues[i];

        // Multiply each sensor reading by its position (i) and scale
        weightedSum += (long)value * i * 1000;
        sum += value;
    }

    // Return weighted average position
    return weightedSum / sum;
}

void endGame(); // Forward declaration of endGame function

/**
 * Determines the current line position and detects special conditions
 * like turns, junctions, and end conditions
 */
void getLinePosition()
{
    // Update sensor readings
    readSensors();

    // Detect left turn - right sensors active, left sensors inactive
    leftTurn = sensorValues[5] > sensorThreshold[5] && sensorValues[6] > sensorThreshold[6] &&
               sensorValues[7] > sensorThreshold[7] && sensorValues[0] < sensorThreshold[0] &&
               sensorValues[1] < sensorThreshold[1];

    // Detect right turn - left sensors active, right sensors inactive
    rightTurn = sensorValues[5] < sensorThreshold[5] && sensorValues[6] < sensorThreshold[6] &&
                sensorValues[7] < sensorThreshold[7] && sensorValues[0] > sensorThreshold[0] &&
                sensorValues[1] > sensorThreshold[1] && sensorValues[2] > sensorThreshold[2];

    // Detect T-junction or base - all sensors active
    tJunctionOrBase = sensorValues[0] > sensorThreshold[0] && sensorValues[1] > sensorThreshold[1] &&
                      sensorValues[2] > sensorThreshold[2] && sensorValues[3] > sensorThreshold[3] &&
                      sensorValues[4] > sensorThreshold[4] && sensorValues[5] > sensorThreshold[5] &&
                      sensorValues[6] > sensorThreshold[6] && sensorValues[7] > sensorThreshold[7];

    // Detect dead end - no sensors active
    deadEnd = sensorValues[0] < sensorThreshold[0] && sensorValues[1] < sensorThreshold[1] &&
              sensorValues[2] < sensorThreshold[2] && sensorValues[3] < sensorThreshold[3] &&
              sensorValues[4] < sensorThreshold[4] && sensorValues[5] < sensorThreshold[5] &&
              sensorValues[6] < sensorThreshold[6] && sensorValues[7] < sensorThreshold[7];

    // Variables for black square detection
    static unsigned long blackSquareStartTime = 0;
    static bool potentialBlackSquare = false;
    static int blackSquareOffset = 50;

    // Check if all sensors are reading significantly higher than threshold (black surface)
    bool allSensorsBlack = sensorValues[0] > (sensorThreshold[0] + blackSquareOffset) &&
                           sensorValues[1] > (sensorThreshold[1] + blackSquareOffset) &&
                           sensorValues[2] > (sensorThreshold[2] + blackSquareOffset) &&
                           sensorValues[3] > (sensorThreshold[3] + blackSquareOffset) &&
                           sensorValues[4] > (sensorThreshold[4] + blackSquareOffset) &&
                           sensorValues[5] > (sensorThreshold[5] + blackSquareOffset) &&
                           sensorValues[6] > (sensorThreshold[6] + blackSquareOffset) &&
                           sensorValues[7] > (sensorThreshold[7] + blackSquareOffset);

    // Start timer when black square is first detected
    if (allSensorsBlack && !potentialBlackSquare)
    {
        blackSquareStartTime = millis();
        potentialBlackSquare = true;
    }

    // If black square is detected continuously for 100ms, consider it a valid detection
    if (allSensorsBlack && potentialBlackSquare && (millis() - blackSquareStartTime >= 100))
    {
        blackSquareDetected = true;
        if (!gameEnded)
        {
            endGame();
        }
    }

    // Reset black square detection if sensors no longer detect black
    if (!allSensorsBlack)
    {
        potentialBlackSquare = false;
    }

    // Update line position state if motion is complete
    if (motionComplete)
    {
        if (leftTurn)
        {
            linePosition = LinePosition::LEFT;
        }
        else if (rightTurn)
        {
            linePosition = LinePosition::RIGHT;
        }
        else if (deadEnd)
        {
            linePosition = LinePosition::NO_LINE;
        }
        else if (tJunctionOrBase && !blackSquareDetected)
        {
            linePosition = LinePosition::T_JUNCTION;
        }
        else
        {
            linePosition = LinePosition::CENTER;
        }
    }
}