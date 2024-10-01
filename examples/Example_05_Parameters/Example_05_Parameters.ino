/*
  Using the BMV080 Particulate Matter PM2.5 Sensor

  This Example shows how to read and set the parameters of the BMV080 sensor.

  These include:
    volumetric_mass_density
    integration_time
    distribution_id
    do_obstruction_detection
    do_vibration_filtering

  After these parameters are read and set, This example shows how to use the 
  sensor in "continuous mode" to get particulate matter readings once every 
  second.

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
  https://www.sparkfun.com/products/?????
*/

#include <Wire.h>
#include "SparkFun_BMV080_Arduino_Library.h" // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BMV080

Bmv080 bmv080; // Create an instance of the BMV080 class
#define BMV080_ADDR 0x57  // SparkFun BMV080 Breakout defaults to 0x57

i2c_device_t i2c_device = {}; // I2C device struct instance for Bosch API

void setup()
{
    Serial.begin(115200);

    while(!Serial) delay(10); // Wait for Serial to become available.
    // Necessary for boards with native USB (like the SAMD51 Thing+).
    // For a final version of a project that does not need serial debug (or a USB cable plugged in),
    // Comment out this while loop, or it will prevent the remaining code from running.

    Serial.println();
    Serial.println("BMV080 Example 5 - Get and Set Parameters");

    Wire.begin();

    if (bmv080.begin(BMV080_ADDR, Wire) == false) {
        Serial.println("BMV080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
        while (1)
        ;
    }
    Serial.println("BMV080 found!");

    // Wire.setClock(400000); //Increase I2C data rate to 400kHz

    /* Communication interface initialization */
    i2c_init(&i2c_device);

    /* Initialize the Sensor (read driver, open, reset, id etc.)*/
    bmv080.init(&i2c_device);

    getSetParameters(); // Get and set parameters

    /* Set the sensor mode to continuous mode */
    if(bmv080.setMode(SFE_BMV080_MODE_CONTINUOUS) == true)
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
    if(bmv080.dataAvailable())
    {
        float pm25 = bmv080.getPM25();

        Serial.print(pm25);

        if(bmv080.getIsObstructed() == true)
        {
            Serial.print("\tObstructed");
        }

        Serial.println();
    }
    delay(100);
}

void getSetParameters(void)
{
    // Variables to store the parameters
    // Note, for custom settings, you will need to set these variables before
    // calling the set functions (down below).

    float volumetric_mass_density = 0.0f;
    float integration_time = 0.0f;
    uint32_t distribution_id = 0;
    bool do_obstruction_detection = false;
    bool do_vibration_filtering = false;

    /***************************************************************************
     * 
     * Reading configuration parameters
     * 
     * *************************************************************************/


    /* Get default parameter "volumetric_mass_density" */
    volumetric_mass_density = bmv080.getVolumetricMassDensity();
    Serial.print("BMV080 parameter 'volumetric_mass_density' READ: ");
    Serial.println(volumetric_mass_density);

    /* Get default parameter "integration_time" */
    integration_time = bmv080.getIntegrationTime();
    Serial.print("BMV080 parameter 'integration_time' READ: ");
    Serial.println(integration_time);

    /* Get default parameter "distribution_id" */
    distribution_id = bmv080.getDistributionId();
    Serial.print("BMV080 parameter 'distribution_id' READ: ");
    Serial.println(distribution_id);

    /* Get default parameter "do_obstruction_detection" */
    do_obstruction_detection = bmv080.getDoObstructionDetection();
    Serial.print("BMV080 parameter 'do_obstruction_detection' READ: ");
    if(do_obstruction_detection == true)
    {
        Serial.println("true");
    }
    else
    {
        Serial.println("false");
    }

    /* Get default parameter "do_vibration_filtering" */
    do_vibration_filtering = bmv080.getDoVibrationFiltering();
    Serial.print("BMV080 parameter 'do_vibration_filtering' READ: ");
    if(do_vibration_filtering == true)
    {
        Serial.println("true");
    }
    else
    {
        Serial.println("false");
    }


    /***************************************************************************
     * 
     * Setting (custom) configuration parameters
     * 
     * *************************************************************************/

    // Uncomment the following lines to set the parameters to custom values

    // volumetric_mass_density = 1.6f;
    // integration_time = 10.0f;
    // distribution_id = 3;
    // do_obstruction_detection = true;
    // do_vibration_filtering = false;

    /* Set custom parameter "volumetric_mass_density" */
    if(bmv080.setVolumetricMassDensity(volumetric_mass_density) == true)
    {
        Serial.print("BMV080 parameter 'volumetric_mass_density' SET TO: ");
        Serial.println(volumetric_mass_density);
    }
    else
    {
        Serial.println("Error setting BMV080 parameter 'volumetric_mass_density'");
    }

    /* Set custom parameter "integration_time" */
    if(bmv080.setIntegrationTime(integration_time) == true)
    {
        Serial.print("BMV080 parameter 'integration_time' SET TO: ");
        Serial.println(integration_time);
    }
    else
    {
        Serial.println("Error setting BMV080 parameter 'integration_time'");
    }

    /* Set custom parameter "distribution_id" */
    if(bmv080.setDistributionId(distribution_id) == true)
    {
        Serial.print("BMV080 parameter 'distribution_id' SET TO: ");
        Serial.println(distribution_id);
    }
    else
    {
        Serial.println("Error setting BMV080 parameter 'distribution_id'");
    }

    /* Set custom parameter "do_obstruction_detection" */
    if(bmv080.setDoObstructionDetection(do_obstruction_detection) == true)
    {
        Serial.print("BMV080 parameter 'do_obstruction_detection' SET TO: ");
        if(do_obstruction_detection == true)
        {
            Serial.println("true");
        }
        else
        {
            Serial.println("false");
        }
    }
    else
    {
        Serial.println("Error setting BMV080 parameter 'do_obstruction_detection'");
    }

    /* Set custom parameter "do_vibration_filtering" */
    if(bmv080.setDoVibrationFiltering(do_vibration_filtering) == true)
    {
        Serial.print("BMV080 parameter 'do_vibration_filtering' SET TO: ");
        if(do_vibration_filtering == true)
        {
            Serial.println("true");
        }
        else
        {
            Serial.println("false");
        }
    }
    else
    {
        Serial.println("Error setting BMV080 parameter 'do_vibration_filtering'");
    }

    Serial.println();
}