/*
  Using the BMV080 Particulate Matter PM2.5 Sensor

  This example shows how to use the sensor in "continuous mode" to get
  particulate matter readings once every second.

  When the sensor is ready to report new data, it  will trigger an interrupt 
  with the IRQ line going low.

  By: Pete Lewis
  SparkFun Electronics
  Date: September, 2024
  SparkFun code, firmware, and software is released under the MIT License.
	Please see LICENSE.md for further details.

  Hardware Connections:
  IoT RedBoard --> BMV080
  QWIIC --> QWIIC
  14  --> IRQ

  BMV080 "mode" jumper set to I2C (default)

  Serial.print it out at 115200 baud to serial monitor.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/?????
*/

#include <Wire.h>
#include "SparkFun_BMV080_Arduino_Library.h" // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BMV080

Bmv080 bmv080;
#define BMV080_IRQ  14 // The BMV080 interrupt pin
#define BMV080_ADDR 0x57  // SparkFun BMV080 Breakout defaults to 0x57

SET_LOOP_TASK_STACK_SIZE(60 * 1024);  // 60KB

extern "C" bmv080_handle_t bmv080_handle = NULL;

/* A unique handle is used to address a BMV080 sensor unit */
//static bmv080_handle_t bmv080_handle = NULL;

/* handle for print function to be used in interrupt service routine */
//static print_function_t print_handle = NULL; 

//volatile uint32_t data_ready_callback_count = 0;

//void print_to_serial(const char *format, ...);

i2c_device_t i2c_device = {};

#define IRQ_Pin 14

bool int_flag = false;

void setup()
{
    // // Start serial
    // Serial.begin(115200);

    // while(!Serial) delay(10); // Wait for Serial to become available.
    // // Necessary for boards with native USB (like the SAMD51 Thing+).
    // // For a final version of a project that does not need serial debug (or a USB cable plugged in),
    // // Comment out this while loop, or it will prevent the remaining code from running.

    // Serial.println();
    // Serial.println("BMV080 Example 1 - Basic Readings");

    // Wire.begin();

    // if (bmv080.begin(BMV080_ADDR, Wire, BMV080_IRQ) == false) {
    //     Serial.println("BMV080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    //     while (1)
    //     ;
    // }
    // Serial.println("BMV080 found!");

    // Wire.setClock(400000); //Increase I2C data rate to 400kHz
    
    Serial.begin(115200);
    Serial.println("Starting BMV080 example...");

    /* Communication interface initialization */

    i2c_init(&i2c_device);
    
    setup_sensor();

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

void setup_sensor(void)
{

    uint16_t major = 0;
    uint16_t minor = 0;
    uint16_t patch = 0;
    char git_hash[12];
    int32_t commits_ahead = 0;

    bmv080_status_code_t bmv080_current_status = bmv080_get_driver_version(&major, &minor, &patch, git_hash, &commits_ahead);

    if (bmv080_current_status != E_BMV080_OK)
    {
        printf("Error getting BMV080 driver version: %d\n", bmv080_current_status);
    }
    
    printf("BMV080 driver version: %d.%d.%d\n", major, minor, patch);

    /* Open the BMV080 sensor unit */
    bmv080_sercom_handle_t sercom_handle = (bmv080_sercom_handle_t)&i2c_device;
    bmv080_callback_read_t read = (const bmv080_callback_read_t)combridge_i2c_read_16bit;
    bmv080_callback_write_t write = (const bmv080_callback_write_t)combridge_i2c_write_16bit;
    bmv080_callback_delay_t delay_ms = (const bmv080_callback_delay_t)combridge_delay;

    bmv080_current_status = bmv080_open(&bmv080_handle, sercom_handle, read, write, delay_ms);

    if (bmv080_current_status != E_BMV080_OK)
    {
        printf("Error opening BMV080 handle: %d\n", bmv080_current_status);
    }
    else
    {
        printf("BMV080 handle opened successfully\n");
    }

    bmv080.setHandle(bmv080_handle);

    /* Reset the BMV080 sensor unit */
    bmv080_current_status = bmv080_reset(bmv080_handle);

    if (bmv080_current_status != E_BMV080_OK)
    {
        printf("Error resetting BMV080 sensor unit: %d\n", bmv080_current_status);
    }
    else
    {
        printf("BMV080 sensor unit reset successfully\n");
    }

    /* Getting the ID of a sensor unit */
    char id[13];
    memset(id, 0x00, 13);
    bmv080_current_status = bmv080_get_sensor_id(bmv080_handle, id);

    if (bmv080_current_status != E_BMV080_OK)
    {
        printf("Error getting BMV080 sensor ID: %d\n", bmv080_current_status);
    }
    else
    {
        printf("BMV080 sensor ID: %s\n", id);
    }

    /* Getting (default) configuration parameters */

    /* Get default parameter "volumetric_mass_density" */
    float volumetric_mass_density = 0.0f;
    bmv080_current_status = bmv080_get_parameter(bmv080_handle, "volumetric_mass_density", (void*)&volumetric_mass_density);    

    if (bmv080_current_status != E_BMV080_OK)
    {
        printf("Error getting BMV080 parameter 'volumetric_mass_density': %d\n", bmv080_current_status);
    }
    else
    {
        printf("BMV080 parameter 'volumetric_mass_density': %.2f\n", volumetric_mass_density);
    }

    /* Get default parameter "integration_time" */
    float integration_time = 0.0f;
    bmv080_current_status = bmv080_get_parameter(bmv080_handle, "integration_time", (void*)&integration_time);

    if (bmv080_current_status != E_BMV080_OK)
    {
        printf("Error getting BMV080 parameter 'integration_time': %d\n", bmv080_current_status);
    }
    else
    {
        printf("BMV080 parameter 'integration_time': %.2f\n", integration_time);
    }    

    /* Get default parameter "distribution_id" */
    uint32_t distribution_id = 0;
    bmv080_current_status = bmv080_get_parameter(bmv080_handle, "distribution_id", (void*)&distribution_id);

    if (bmv080_current_status != E_BMV080_OK)
    {
        printf("Error getting BMV080 parameter 'distribution_id': %d\n", bmv080_current_status);
    }
    else
    {
        printf("BMV080 parameter 'distribution_id': %d\n", distribution_id);
    }

    /* Get default parameter "do_obstruction_detection" */
    bool do_obstruction_detection = false;
    bmv080_current_status = bmv080_get_parameter(bmv080_handle, "do_obstruction_detection", (void*)&do_obstruction_detection);

    if (bmv080_current_status != E_BMV080_OK)
    {
        printf("Error getting BMV080 parameter 'do_obstruction_detection': %d\n", bmv080_current_status);
    }
    else
    {
        printf("BMV080 parameter 'do_obstruction_detection': %s\n", do_obstruction_detection ? "true" : "false");
    }

    /* Get default parameter "do_vibration_filtering" */

    bool do_vibration_filtering = false;
    bmv080_current_status = bmv080_get_parameter(bmv080_handle, "do_vibration_filtering", (void*)&do_vibration_filtering);

    if (bmv080_current_status != E_BMV080_OK)
    {
        printf("Error getting BMV080 parameter 'do_vibration_filtering': %d\n", bmv080_current_status);
    }
    else
    {
        printf("BMV080 parameter 'do_vibration_filtering': %s\n", do_vibration_filtering ? "true" : "false");
    }

    /*********************************************************************************************************************
    * Setting (custom) configuration parameters
    *********************************************************************************************************************/

    bmv080_current_status = bmv080_set_parameter(bmv080_handle, "volumetric_mass_density", (void*)&volumetric_mass_density);

    if (bmv080_current_status != E_BMV080_OK)
    {
        printf("Error setting BMV080 parameter 'volumetric_mass_density': %d\n", bmv080_current_status);
    }
    else
    {
        printf("BMV080 parameter 'volumetric_mass_density' set successfully\n");
    }

    bmv080_current_status = bmv080_set_parameter(bmv080_handle, "integration_time", (void*)&integration_time);

    if (bmv080_current_status != E_BMV080_OK)
    {
        printf("Error setting BMV080 parameter 'integration_time': %d\n", bmv080_current_status);
    }
    else
    {
        printf("BMV080 parameter 'integration_time' set successfully\n");
    }

    bmv080_current_status = bmv080_set_parameter(bmv080_handle, "distribution_id", (void*)&distribution_id);

    if (bmv080_current_status != E_BMV080_OK)
    {
        printf("Error setting BMV080 parameter 'distribution_id': %d\n", bmv080_current_status);
    }
    else
    {
        printf("BMV080 parameter 'distribution_id' set successfully\n");
    }

    bmv080_current_status = bmv080_set_parameter(bmv080_handle, "do_obstruction_detection", (void*)&do_obstruction_detection);

    if (bmv080_current_status != E_BMV080_OK)
    {
        printf("Error setting BMV080 parameter 'do_obstruction_detection': %d\n", bmv080_current_status);
    }
    else
    {
        printf("BMV080 parameter 'do_obstruction_detection' set successfully\n");
    }

    bmv080_current_status = bmv080_set_parameter(bmv080_handle, "do_vibration_filtering", (void*)&do_vibration_filtering);

    if (bmv080_current_status != E_BMV080_OK)
    {
        printf("Error setting BMV080 parameter 'do_vibration_filtering': %d\n", bmv080_current_status);
    }
    else
    {
        printf("BMV080 parameter 'do_vibration_filtering' set successfully\n");
    }
}