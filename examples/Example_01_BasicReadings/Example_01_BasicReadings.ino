/*
  Using the BMV080 Particulate Matter PM2.5 Sensor

  This example shows how to use the sensor in "continuous mode" to get
  particulate matter readings once every second.

  It uses polling of the device to check if new data is available.

  By: Pete Lewis
  SparkFun Electronics
  Date: September, 2024
  SparkFun code, firmware, and software is released under the MIT License.
    Please see LICENSE.md for further details.

  Hardware Connections:
  IoT RedBoard --> BMV080
  QWIIC --> QWIIC

  BMV080 "mode" jumper set to I2C (default)

  Serial.print it out at 115200 baud to serial monitor.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/26554
*/

#include "SparkFun_BMV080_Arduino_Library.h" // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BMV080
#include <Wire.h>

SparkFunBMV080I2C bmv080;           // Create an instance of the BMV080 class
#define BMV080_ADDR 0x57 // SparkFun BMV080 Breakout defaults to 0x57

void setup()
{
    Serial.begin(115200);

    while (!Serial)
        delay(10); // Wait for Serial to become available.
    // Necessary for boards with native USB (like the SAMD51 Thing+).
    // For a final version of a project that does not need serial debug (or a USB cable plugged in),
    // Comment out this while loop, or it will prevent the remaining code from running.

    Serial.println();
    Serial.println("BMV080 Example 1 - Basic Readings");

    Wire.begin();

    if (bmv080.begin(BMV080_ADDR, Wire) == false)
    {
        Serial.println(
            "BMV080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
        while (1)
            ;
    }
    Serial.println("BMV080 found!");

    // Wire.setClock(400000); //Increase I2C data rate to 400kHz

    /* Initialize the Sensor (read driver, open, reset, id etc.)*/
    bmv080.init();

    /* Set the sensor mode to continuous mode */
    if (bmv080.setMode(SFE_BMV080_MODE_CONTINUOUS) == true)
    {
        Serial.println("BMV080 set to continuous mode");
    }
    else
    {
        Serial.println("Error setting BMV080 mode");
    }
}

void loop()
{
    if (bmv080.dataAvailable())
    {
        float pm25 = bmv080.getPM25();

        Serial.print(pm25);

        if (bmv080.getIsObstructed() == true)
        {
            Serial.print("\tObstructed");
        }

        Serial.println();
    }
    delay(100);
}
