/******************************************************************************
    sfeQwiicBuzzer.h
    SparkFun Qwiic Buzzer Library header file

    by Pete Lewis @SparkFun Electronics
    January 2024

    Based on original source code written by
    Fischer Moseley @ SparkFun Electronics
    Original Creation Date: July 24, 2019

    Development environment specifics:
    IDE: Arduino 2.2.1
    Hardware Platform: Arduino Uno/SparkFun Redboard
    Qwiic Buzzer Version: v10

    SPDX-License-Identifier: MIT

    Copyright (c) 2023 SparkFun Electronics

    Distributed as-is; no warranty is given.
******************************************************************************/

#include "sfeBmv080.h"
#include "bmv080.h"
#include "bmv080_defs.h"

#define SPI_CLK_FREQ          ((uint32_t)(1e6))

// Some communication functions used with the system. These are from the original code from
// Bosch  - so keeping them the same. It is unclear if the library they provide depends on these
// specific values - it probably does - so leaving as is.

#define E_COMBRIDGE_OK ((int8_t)0)
/*! -1: Status codes returned when memory allocation fails */
#define E_COMBRIDGE_ERROR_MEMORY_ALLOCATION ((int8_t) - 1)
/*! -2: Status codes returned when the read operation fails */
#define E_COMBRIDGE_ERROR_READ ((int8_t) - 2)
/*! -3: Status codes returned when the write operation fails */
#define E_COMBRIDGE_ERROR_WRITE ((int8_t) - 3)
/*! -4: Status codes returned when writing the header fails */
#define E_COMBRIDGE_ERROR_WRITE_HEADER ((int8_t) - 4)
/*! -5:  Status codes returned when a reference is null */
#define E_COMBRIDGE_ERROR_NULLPTR ((int8_t) - 5)

// C function used in this library only - so static

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

    /* Our bus read and write functions */

    // --------------------------------------------------------------------------------------------
    // Callback for reading data-- called from the Bosch supplied library
    //
    static int8_t device_read_16bit_CB(bmv080_sercom_handle_t handle, uint16_t header, uint16_t *payload,
                                       uint16_t payload_length)
    {
        if (handle == nullptr)
            return E_COMBRIDGE_ERROR_NULLPTR;   

        // Our output var.
        size_t nRead = 0;

        // Get our sparkfun toolkit bus object/interface
        sfeTkIBus *theBus = (sfeTkIBus *)handle;

        if(theBus->type() == kBusTypeI2C)
            header = header << 1; // I2C specific shift           

        sfeTkError_t rc = theBus->readRegister16Region16(header, payload, payload_length, nRead);

        if (rc != kSTkErrOk || nRead != payload_length)
            return E_COMBRIDGE_ERROR_READ;  

        return E_COMBRIDGE_OK;
    }

    // --------------------------------------------------------------------------------------------
    // Callback for reading data-- called from the Bosch supplied library
    //
    static int8_t device_write_16bit_CB(bmv080_sercom_handle_t handle, uint16_t header, const uint16_t *payload,
                                        uint16_t payload_length)
    {
        if (handle == nullptr)
            return E_COMBRIDGE_ERROR_NULLPTR;

        sfeTkIBus *theBus = (sfeTkIBus *)handle;

        if(theBus->type() == kBusTypeI2C) // I2C specific shift
            header = header << 1;

        sfeTkError_t rc = theBus->writeRegister16Region16(header, payload, payload_length);

        // okay, not okay?
        return rc == kSTkErrOk ? E_COMBRIDGE_OK : E_COMBRIDGE_ERROR_WRITE;
    }

    // --------------------------------------------------------------------------------------------
    // Delay callback function for the Bosch library
    //
    static int8_t device_delay_CB(uint32_t period)
    {
        delay(period);

        return E_COMBRIDGE_OK;
    }

    // This function is just used in this file, so declaring it static

    /* Custom function for consuming sensor readings */
    static void use_sensor_output(bmv080_output_t bmv080_output, void *callback_parameters)
    {
        // data_ready_callback_count += 1;
        // print_function_t print = (print_function_t)callback_parameters;

        ((sfeBmv080 *)callback_parameters)->setSensorValue(bmv080_output);

        // Serial.println(bmv080_output.pm2_5);

        // Serial.println("u");

        // sfeBmv080::_sensorValue.pm2_5 = bmv080_output.pm2_5; // update the class variable with the new PM2.5 value
        // setSensorValue(bmv080_output.pm2_5);
        // sensorValue.pm2_5 = bmv080_output.pm2_5;
        //  print("Runtime: %.2f s, PM2.5: %.0f ug/m^3, obstructed: %s, outside detection limits: %s\r\n",
        //      bmv080_output.runtime_in_sec, bmv080_output.pm2_5, (bmv080_output.is_obstructed ? "yes" : "no"),
        //      (bmv080_output.is_outside_detection_limits ? "yes" : "no"));
    }

    static void bmv080_service_routine(const bmv080_handle_t handle, void *callback_parameters)
    {
        /* The interrupt is served by the BMV080 sensor driver */
        bmv080_status_code_t bmv080_current_status =
            bmv080_serve_interrupt(handle, (bmv080_callback_data_ready_t)use_sensor_output, callback_parameters);
        if (bmv080_current_status != E_BMV080_OK)
        {
            printf("Fetching measurement data failed with BMV080 status %d\r\n", (int32_t)bmv080_current_status);
        }
    }

#ifdef __cplusplus
}
#endif

sfeTkError_t sfeBmv080::begin(sfeTkIBus *theBus)
{
    // Nullptr check
    if (theBus == nullptr)
        return kSTkErrFail;

    // Set bus pointer
    _theBus = theBus;

    return kSTkErrOk;
}

float sfeBmv080::getPM25()
{
    return _sensorValue.pm2_5;
}

bool sfeBmv080::getIsObstructed()
{
    return _sensorValue.is_obstructed;
}

void sfeBmv080::setSensorValue(bmv080_output_t bmv080_output)
{
    _dataAvailable = true;
    _sensorValue.pm2_5 = bmv080_output.pm2_5;
    _sensorValue.runtime_in_sec = bmv080_output.runtime_in_sec;
    _sensorValue.is_obstructed = bmv080_output.is_obstructed;
    _sensorValue.is_outside_detection_limits = bmv080_output.is_outside_detection_limits;
}

bool sfeBmv080::setMode(uint8_t mode)
{
    bmv080_status_code_t bmv080_current_status; // return status from the Bosch API function

    if (mode == SFE_BMV080_MODE_CONTINUOUS)
    {
        bmv080_current_status = bmv080_start_continuous_measurement(bmv080_handle_class);
    }
    else if (mode == SFE_BMV080_MODE_DUTY_CYCLE)
    {
        bmv080_duty_cycling_mode_t duty_cycling_mode = E_BMV080_DUTY_CYCLING_MODE_0;
        bmv080_current_status = bmv080_start_duty_cycling_measurement(
            bmv080_handle_class, (bmv080_callback_tick_t)millis, duty_cycling_mode);
    }

    // check if the mode was set correctly
    if (bmv080_current_status == E_BMV080_OK)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool sfeBmv080::dataAvailable()
{
    bmv080_service_routine(bmv080_handle_class, this);
    if (_dataAvailable == true)
    {
        _dataAvailable = false;
        return true;
    }
    else
        return false;
}

// Our init method
bool sfeBmv080::init()
{
    // Do we have a bus?
    if (_theBus == nullptr)
        return false;

    if (!getDriverVersion() || !open() || !reset() || !getID())
        return false;

    return true;
}



bool sfeBmv080::open()
{
    if (_theBus == nullptr)
        return false;

    // Open the device - pass in the data read, data write and delay functions callbacks. Note - the "secrom_handle_t"
    // is just a pointer to our Tookkit communication bus objects

    // When sending a sercom handle of SPI to bmv_open, the pointer must be a struct that includes both the SPI port (instance) and the Settings
    // This is because the Bosch API needs to know the SPI port and the settings for the SPI port



    bmv080_status_code_t status =
        bmv080_open(&bmv080_handle_class, (bmv080_sercom_handle_t)_theBus, (bmv080_callback_read_t)device_read_16bit_CB,
                    (bmv080_callback_write_t)device_write_16bit_CB, (bmv080_callback_delay_t)device_delay_CB);

    if (status != E_BMV080_OK)
    {
        Serial.println("BMV080 open failed");
        return false;
    }
    else
    {
        Serial.println("BMV080 open successfully");
        return true;
    }
}

bool sfeBmv080::reset()
{
    bmv080_status_code_t bmv080_current_status = bmv080_reset(bmv080_handle_class);

    if (bmv080_current_status != E_BMV080_OK)
    {
        Serial.println("BMV080 reset failed");
        return false;
    }
    else
    {
        Serial.println("BMV080 reset successfully");
        return true;
    }
}

bool sfeBmv080::getDriverVersion()
{
    uint16_t major = 0;
    uint16_t minor = 0;
    uint16_t patch = 0;
    char git_hash[12];
    int32_t commits_ahead = 0;

    bmv080_status_code_t bmv080_current_status =
        bmv080_get_driver_version(&major, &minor, &patch, git_hash, &commits_ahead);

    if (bmv080_current_status != E_BMV080_OK)
    {
        printf("Error getting BMV080 driver version: %d\n", bmv080_current_status);
        return false;
    }

    printf("BMV080 driver version: %d.%d.%d\n", major, minor, patch);
    return true;
}

bool sfeBmv080::getID()
{
    char id[13];
    memset(id, 0x00, 13);
    bmv080_status_code_t bmv080_current_status = bmv080_get_sensor_id(bmv080_handle_class, id);

    if (bmv080_current_status != E_BMV080_OK)
    {
        printf("Error getting BMV080 sensor ID: %d\n", bmv080_current_status);
        return false;
    }
    else
    {
        printf("BMV080 sensor ID: %s\n", id);
        return true;
    }
}

uint16_t sfeBmv080::getDutyCyclingPeriod()
{
    uint16_t duty_cycling_period = 0;
    bmv080_status_code_t bmv080_current_status =
        bmv080_get_parameter(bmv080_handle_class, "duty_cycling_period", (void *)&duty_cycling_period);
    if (bmv080_current_status != E_BMV080_OK)
    {
        printf("Error getting BMV080 Duty Cycling Period: %d\n", bmv080_current_status);
        return 0;
    }
    else
    {
        printf("BMV080 Duty Cycling Period Read: %d\n", duty_cycling_period);
        return duty_cycling_period;
    }
}

bool sfeBmv080::setDutyCyclingPeriod(uint16_t duty_cycling_period)
{
    bmv080_status_code_t bmv080_current_status =
        bmv080_set_parameter(bmv080_handle_class, "duty_cycling_period", (void *)&duty_cycling_period);
    if (bmv080_current_status != E_BMV080_OK)
    {
        printf("Error setting BMV080 Duty Cycling Period: %d\n", bmv080_current_status);
        return false;
    }
    else
    {
        printf("BMV080 Duty Cycling Period Set: %d\n", duty_cycling_period);
        return true;
    }
}

float sfeBmv080::getVolumetricMassDensity()
{
    float volumetric_mass_density = 0.0;
    bmv080_status_code_t bmv080_current_status =
        bmv080_get_parameter(bmv080_handle_class, "volumetric_mass_density", (void *)&volumetric_mass_density);
    if (bmv080_current_status != E_BMV080_OK)
    {
        printf("Error getting BMV080 Volumetric Mass Density: %d\n", bmv080_current_status);
        return 0.0;
    }
    else
    {
        return volumetric_mass_density;
    }
}

bool sfeBmv080::setVolumetricMassDensity(float volumetric_mass_density)
{
    bmv080_status_code_t bmv080_current_status =
        bmv080_set_parameter(bmv080_handle_class, "volumetric_mass_density", (void *)&volumetric_mass_density);
    if (bmv080_current_status != E_BMV080_OK)
    {
        printf("Error setting BMV080 Volumetric Mass Density: %d\n", bmv080_current_status);
        return false;
    }
    else
    {
        return true;
    }
}

float sfeBmv080::getIntegrationTime()
{
    float integration_time = 0.0;
    bmv080_status_code_t bmv080_current_status =
        bmv080_get_parameter(bmv080_handle_class, "integration_time", (void *)&integration_time);
    if (bmv080_current_status != E_BMV080_OK)
    {
        printf("Error getting BMV080 Integration Time: %d\n", bmv080_current_status);
        return 0.0;
    }
    else
    {
        return integration_time;
    }
}

bool sfeBmv080::setIntegrationTime(float integration_time)
{
    bmv080_status_code_t bmv080_current_status =
        bmv080_set_parameter(bmv080_handle_class, "integration_time", (void *)&integration_time);
    if (bmv080_current_status != E_BMV080_OK)
    {
        printf("Error setting BMV080 Integration Time: %d\n", bmv080_current_status);
        return false;
    }
    else
    {
        return true;
    }
}

uint32_t sfeBmv080::getDistributionId()
{
    uint32_t distribution_id = 0;
    bmv080_status_code_t bmv080_current_status =
        bmv080_get_parameter(bmv080_handle_class, "distribution_id", (void *)&distribution_id);
    if (bmv080_current_status != E_BMV080_OK)
    {
        printf("Error getting BMV080 Distribution ID: %d\n", bmv080_current_status);
        return 0;
    }
    else
    {
        return distribution_id;
    }
}

bool sfeBmv080::setDistributionId(uint32_t distribution_id)
{
    bmv080_status_code_t bmv080_current_status =
        bmv080_set_parameter(bmv080_handle_class, "distribution_id", (void *)&distribution_id);
    if (bmv080_current_status != E_BMV080_OK)
    {
        printf("Error setting BMV080 Distribution ID: %d\n", bmv080_current_status);
        return false;
    }
    else
    {
        return true;
    }
}

bool sfeBmv080::getDoObstructionDetection()
{
    bool do_obstruction_detection = false;
    bmv080_status_code_t bmv080_current_status =
        bmv080_get_parameter(bmv080_handle_class, "do_obstruction_detection", (void *)&do_obstruction_detection);
    if (bmv080_current_status != E_BMV080_OK)
    {
        printf("Error getting BMV080 Obstruction Detection: %d\n", bmv080_current_status);
        return false;
    }
    else
    {
        return do_obstruction_detection;
    }
}

bool sfeBmv080::setDoObstructionDetection(bool do_obstruction_detection)
{
    bmv080_status_code_t bmv080_current_status =
        bmv080_set_parameter(bmv080_handle_class, "do_obstruction_detection", (void *)&do_obstruction_detection);
    if (bmv080_current_status != E_BMV080_OK)
    {
        printf("Error setting BMV080 Obstruction Detection: %d\n", bmv080_current_status);
        return false;
    }
    else
    {
        return true;
    }
}

bool sfeBmv080::getDoVibrationFiltering()
{
    bool do_vibration_filtering = false;
    bmv080_status_code_t bmv080_current_status =
        bmv080_get_parameter(bmv080_handle_class, "do_vibration_filtering", (void *)&do_vibration_filtering);
    if (bmv080_current_status != E_BMV080_OK)
    {
        printf("Error getting BMV080 Vibration Filtering: %d\n", bmv080_current_status);
        return false;
    }
    else
    {
        return do_vibration_filtering;
    }
}

bool sfeBmv080::setDoVibrationFiltering(bool do_vibration_filtering)
{
    bmv080_status_code_t bmv080_current_status =
        bmv080_set_parameter(bmv080_handle_class, "do_vibration_filtering", (void *)&do_vibration_filtering);
    if (bmv080_current_status != E_BMV080_OK)
    {
        printf("Error setting BMV080 Vibration Filtering: %d\n", bmv080_current_status);
        return false;
    }
    else
    {
        return true;
    }
}
