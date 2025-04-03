/**
 * @brief Measures distance using an ultrasonic sensor.
 *
 * This function triggers an ultrasonic sensor connected to TRIG and ECHO pins,
 * then calculates the distance based on the time taken for the echo to return.
 *
 * @return Distance in centimeters.
 *         Returns 999 if the measurement is invalid (no echo or out of range).
 *         Valid range is approximately 2cm to 200cm.
 */
float measureDistance()
{
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    long duration = pulseIn(ECHO, HIGH, 30000);
    float distance = duration * 0.034 / 2;

    if (distance == 0 || distance > 200)
    {
        return 999;
    }

    return distance;
}