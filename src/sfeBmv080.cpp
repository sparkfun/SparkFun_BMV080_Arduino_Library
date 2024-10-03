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

// Some communication functions used with the system

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

/* Exported functions prototypes ---------------------------------------------*/

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

    /* Our bus read and write functions */

    // --------------------------------------------------------------------------------------------
    //
    static int8_t device_read_16bit_CB(bmv080_sercom_handle_t handle, uint16_t header, uint16_t *payload,
                                       uint16_t payload_length)
    {
        if (handle == nullptr)
            return E_COMBRIDGE_ERROR_NULLPTR;

        sfeTkIBus *theBus = (sfeTkIBus *)handle;
        uint8_t *payload_byte = (uint8_t *)payload;

        /* 16-bit header left shifted by 1, since the R/W bit (most significant bit) is passed along with the 7-bit
         * device address  */
        uint16_t header_adjusted = header << 1;

        size_t nRead = 0;

        sfeTkError_t rc = theBus->readRegister16Region(header_adjusted, (uint8_t *)payload, payload_length * 2, nRead);

        if (rc != kSTkErrOk)
            return E_COMBRIDGE_ERROR_READ;

        if (nRead != payload_length * 2)
            return E_COMBRIDGE_ERROR_READ;

        // Need to swap the byte order
        for (uint16_t i = 0; i < payload_length; i++)
            payload[i] = ((payload[i] << 8) | (payload[i] >> 8)) & 0xffff;

        return E_COMBRIDGE_OK;
    }

    // --------------------------------------------------------------------------------------------
    //
    static int8_t device_write_16bit_CB(bmv080_sercom_handle_t handle, uint16_t header, const uint16_t *payload,
                                        uint16_t payload_length)
    {
        if (handle == nullptr)
            return E_COMBRIDGE_ERROR_NULLPTR;

        sfeTkIBus *theBus = (sfeTkIBus *)handle;

        uint16_t header_adjusted = header << 1;

        // Need to reverse the byte order
        uint16_t payload_swapped[payload_length];

        for (uint16_t i = 0; i < payload_length; i++)
            payload_swapped[i] = ((payload[i] << 8) | (payload[i] >> 8)) & 0xffff;

        sfeTkError_t rc =
            theBus->writeRegister16Region(header_adjusted, (uint8_t *)payload_swapped, payload_length * 2);

        return rc == kSTkErrOk ? E_COMBRIDGE_OK : E_COMBRIDGE_ERROR_WRITE;
    }

    // --------------------------------------------------------------------------------------------
    //
    static int8_t device_delay_CB(uint32_t period)
    {
        delay(period);

        return E_COMBRIDGE_OK;
    }

    // static void bmv080_service_routine(const bmv080_handle_t handle, void *callback_parameters);
    // static void use_sensor_output(bmv080_output_t bmv080_output, void *callback_parameters);

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

// bool sfeBmv080::init(i2c_device_t *i2c_device)
bool sfeBmv080::init(void)
{
    if (_theBus == nullptr)
        return false;

    if (getDriverVersion() == false)
    {
        return false;
    }

    if (open() == false)
    {
        return false;
    }

    if (reset() == false)
    {
        return false;
    }

    if (getID() == false)
    {
        return false;
    }

    return true;
}

bool sfeBmv080::open()
{
    if (_theBus == nullptr)
        return false;

    // bmv080_sercom_handle_t sercom_handle = (bmv080_sercom_handle_t)i2c_device;
    // bmv080_callback_read_t read = (const bmv080_callback_read_t)combridge_i2c_read_16bit;
    // bmv080_callback_write_t write = (const bmv080_callback_write_t)combridge_i2c_write_16bit;
    // bmv080_callback_delay_t delay_ms = (const bmv080_callback_delay_t)combridge_delay;

    // bmv080_status_code_t bmv080_current_status =
    //     bmv080_open(&bmv080_handle_class, sercom_handle, read, write, delay_ms);

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
