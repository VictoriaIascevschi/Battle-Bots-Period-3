/**
 * LED Control Module
 * Controls the NeoPixel LEDs for different robot states and actions
 */

/**
 * Turns off all LEDs by setting their color to black (0,0,0)
 */
void resetLights()
{
    NeoPixel.setPixelColor(PIXEL_TOP_LEFT, 0, 0, 0);
    NeoPixel.setPixelColor(PIXEL_TOP_RIGHT, 0, 0, 0);
    NeoPixel.setPixelColor(PIXEL_BOTTOM_LEFT, 0, 0, 0);
    NeoPixel.setPixelColor(PIXEL_BOTTOM_RIGHT, 0, 0, 0);
    NeoPixel.show(); // Update the physical LEDs
}

/**
 * Creates a blinking effect for all LEDs
 *
 * @param frequency Interval in milliseconds between on/off states
 * @param r Red color component (0-255)
 * @param g Green color component (0-255)
 * @param b Blue color component (0-255)
 */
void blinkLEDs(int frequency, int r, int g, int b)
{
    static unsigned long previousMillis = 0; // Stores last time LEDs were updated
    static bool isOn = false;                // Tracks current LED state
    const unsigned long interval = frequency;

    unsigned long currentMillis = millis();

    // Check if it's time to toggle the LED state
    if (currentMillis - previousMillis >= interval)
    {
        previousMillis = currentMillis;
        isOn = !isOn; // Toggle state

        if (isOn)
        {
            // Turn LEDs on with specified color
            NeoPixel.setPixelColor(PIXEL_TOP_LEFT, r, g, b);
            NeoPixel.setPixelColor(PIXEL_TOP_RIGHT, r, g, g);
            NeoPixel.setPixelColor(PIXEL_BOTTOM_LEFT, r, g, g);
            NeoPixel.setPixelColor(PIXEL_BOTTOM_RIGHT, r, g, g);
            NeoPixel.show();
        }
        else
        {
            // Turn LEDs off
            resetLights();
        }
    }
}

/**
 * Sets LEDs to standby mode - blinking red
 */
void setStandByColor()
{
    blinkLEDs(350, 255, 0, 0); // Blink red color every 350ms
}

/**
 * Sets LEDs to calibration mode - solid blue
 */
void setCalibrationColor()
{
    NeoPixel.setPixelColor(PIXEL_TOP_LEFT, 0, 60, 255);     // Blue
    NeoPixel.setPixelColor(PIXEL_TOP_RIGHT, 0, 60, 255);    // Blue
    NeoPixel.setPixelColor(PIXEL_BOTTOM_LEFT, 0, 60, 255);  // Blue
    NeoPixel.setPixelColor(PIXEL_BOTTOM_RIGHT, 0, 60, 255); // Blue
    NeoPixel.show();
}

/**
 * Sets LEDs to indicate forward movement - top LEDs green
 */
void setForwardColor()
{
    NeoPixel.setPixelColor(PIXEL_TOP_LEFT, 0, 255, 0);   // Green
    NeoPixel.setPixelColor(PIXEL_TOP_RIGHT, 0, 255, 0);  // Green
    NeoPixel.setPixelColor(PIXEL_BOTTOM_LEFT, 0, 0, 0);  // Off
    NeoPixel.setPixelColor(PIXEL_BOTTOM_RIGHT, 0, 0, 0); // Off
    NeoPixel.show();
}

/**
 * Sets LEDs to indicate backward movement - bottom LEDs red
 */
void setBackwardColor()
{
    NeoPixel.setPixelColor(PIXEL_TOP_LEFT, 0, 0, 0);       // Off
    NeoPixel.setPixelColor(PIXEL_TOP_RIGHT, 0, 0, 0);      // Off
    NeoPixel.setPixelColor(PIXEL_BOTTOM_LEFT, 255, 0, 0);  // Red
    NeoPixel.setPixelColor(PIXEL_BOTTOM_RIGHT, 255, 0, 0); // Red
    NeoPixel.show();
}

/**
 * Sets LEDs to indicate right turn - right LEDs orange
 */
void setTurnRightColor()
{
    NeoPixel.setPixelColor(PIXEL_TOP_LEFT, 0, 0, 0);         // Off
    NeoPixel.setPixelColor(PIXEL_TOP_RIGHT, 255, 100, 0);    // Orange
    NeoPixel.setPixelColor(PIXEL_BOTTOM_LEFT, 0, 0, 0);      // Off
    NeoPixel.setPixelColor(PIXEL_BOTTOM_RIGHT, 255, 100, 0); // Orange
    NeoPixel.show();
}

/**
 * Sets LEDs to indicate left turn - left LEDs orange
 */
void setTurnLeftColor()
{
    NeoPixel.setPixelColor(PIXEL_TOP_LEFT, 255, 100, 0);    // Orange
    NeoPixel.setPixelColor(PIXEL_TOP_RIGHT, 0, 0, 0);       // Off
    NeoPixel.setPixelColor(PIXEL_BOTTOM_LEFT, 255, 100, 0); // Orange
    NeoPixel.setPixelColor(PIXEL_BOTTOM_RIGHT, 0, 0, 0);    // Off
    NeoPixel.show();
}

/**
 * Sets LEDs to indicate turning around - all LEDs orange
 */
void setTurnAroundColor()
{
    NeoPixel.setPixelColor(PIXEL_TOP_LEFT, 255, 100, 0);     // Orange
    NeoPixel.setPixelColor(PIXEL_TOP_RIGHT, 255, 100, 0);    // Orange
    NeoPixel.setPixelColor(PIXEL_BOTTOM_LEFT, 255, 100, 0);  // Orange
    NeoPixel.setPixelColor(PIXEL_BOTTOM_RIGHT, 255, 100, 0); // Orange
    NeoPixel.show();
}

/**
 * Sets LEDs to indicate stop - all LEDs red
 */
void setStopColor()
{
    NeoPixel.setPixelColor(PIXEL_TOP_LEFT, 255, 0, 0);     // Red
    NeoPixel.setPixelColor(PIXEL_TOP_RIGHT, 255, 0, 0);    // Red
    NeoPixel.setPixelColor(PIXEL_BOTTOM_LEFT, 255, 0, 0);  // Red
    NeoPixel.setPixelColor(PIXEL_BOTTOM_RIGHT, 255, 0, 0); // Red
    NeoPixel.show();
}
