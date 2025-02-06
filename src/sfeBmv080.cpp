/******************************************************************************
    sfeBmv080.cpp
    SparkFun BMV080 Library CPP file

    by Pete Lewis @SparkFun Electronics
    September 2024

    Development environment specifics:
    IDE: Arduino 2.3.3
    Hardware Platform: SparkFun IoT Redboard ESP32
    BMV080 Breakout HW Version: v01

    SPDX-License-Identifier: MIT

    Copyright (c) 2024 SparkFun Electronics

    Distributed as-is; no warranty is given.
******************************************************************************/

#include "sfeBmv080.h"
#include "bmv080.h"
#include "bmv080_defs.h"

#define SPI_CLK_FREQ ((uint32_t)(1e6))

// Some communication functions used with the system. These are from the original code from
// Bosch  - so keeping them the same. It is unclear if the library they provide depends on these
// specific values - it probably does - so leaving as is.

#define E_COMBRIDGE_OK ((int8_t)0)
/*! -1: Status codes returned when memory allocation fails */
#define E_COMBRIDGE_ERROR_MEMORY_ALLOCATION ((int8_t)-1)
/*! -2: Status codes returned when the read operation fails */
#define E_COMBRIDGE_ERROR_READ ((int8_t)-2)
/*! -3: Status codes returned when the write operation fails */
#define E_COMBRIDGE_ERROR_WRITE ((int8_t)-3)
/*! -4: Status codes returned when writing the header fails */
#define E_COMBRIDGE_ERROR_WRITE_HEADER ((int8_t)-4)
/*! -5:  Status codes returned when a reference is null */
#define E_COMBRIDGE_ERROR_NULLPTR ((int8_t)-5)

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

        if (theBus->type() == kBusTypeI2C) // I2C specific shift
            header = header << 1;

        sfeTkError_t rc = theBus->readRegister(header, payload, payload_length, nRead);

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

        if (theBus->type() == kBusTypeI2C) // I2C specific shift
            header = header << 1;

        sfeTkError_t rc = theBus->writeRegister(header, payload, payload_length);

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

    //---------------------------------------------------------------------
    // This function is just used in this file, so declaring it static

    /* Custom function for consuming sensor readings */
    static void use_sensor_output(bmv080_output_t bmv080_output, void *callback_parameters)
    {
        ((sfeBmv080 *)callback_parameters)->setSensorValue(bmv080_output);
    }

    //---------------------------------------------------------------------
    static void bmv080_service_routine(const bmv080_handle_t handle, void *callback_parameters)
    {
        /* The interrupt is served by the BMV080 sensor driver */
        bmv080_status_code_t bmv080_current_status =
            bmv080_serve_interrupt(handle, (bmv080_callback_data_ready_t)use_sensor_output, callback_parameters);
        if (bmv080_current_status != E_BMV080_OK)
        {
            // TODO: libraries should not output text by default, need to add a debug mode/flag to library?
            printf("Fetching measurement data failed with BMV080 status %d\r\n", (int32_t)bmv080_current_status);
        }
    }

#ifdef __cplusplus
}
#endif

//---------------------------------------------------------------------
sfeTkError_t sfeBmv080::begin(sfeTkIBus *theBus)
{
    // Nullptr check
    if (theBus == nullptr)
        return kSTkErrFail;

    // Set bus pointer
    _theBus = theBus;

    return kSTkErrOk;
}

//---------------------------------------------------------------------
float sfeBmv080::PM25()
{
    return _sensorValue.pm2_5_mass_concentration;
}

//---------------------------------------------------------------------
float sfeBmv080::PM1()
{
    return _sensorValue.pm1_mass_concentration;
}

//---------------------------------------------------------------------
bool sfeBmv080::isObstructed()
{
    return _sensorValue.is_obstructed;
}

//---------------------------------------------------------------------
void sfeBmv080::setSensorValue(bmv080_output_t bmv080_output)
{
    // TODO: should here be a mode where the library user can set register a callback function to handle the data?
    //       This way the end user can get all the sensor data at once - possible issue is stack/re-entrancy
    _dataAvailable = true;

    // cache the latest sensor values - copy output to our class variable
    _sensorValue = bmv080_output;
}

//---------------------------------------------------------------------
bool sfeBmv080::sensorValue(bmv080_output_t *bmv080_output, bool update_data /* default is true*/)
{
    if (!bmv080_output)
        return false;

    // Get the latest sensor data ...
    if (update_data)
    {
        _dataAvailable = false;
        bmv080_service_routine(bmv080_handle_class, this);
    }
    if (_dataAvailable)
        *bmv080_output = _sensorValue;
    return _dataAvailable;
}

//---------------------------------------------------------------------
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

//---------------------------------------------------------------------
bool sfeBmv080::isDataAvailable()
{
    bmv080_service_routine(bmv080_handle_class, this);
    // TODO: What is the logic here?  The expectation is that a user calls this before accessing any data?
    if (_dataAvailable == true)
    {
        _dataAvailable = false;
        return true;
    }
    else
        return false;
}

//---------------------------------------------------------------------
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

//---------------------------------------------------------------------
bool sfeBmv080::open()
{
    if (_theBus == nullptr)
        return false;

    // Open the device - pass in the data read, data write and delay functions callbacks. Note - the "secrom_handle_t"
    // is just a pointer to our Tookkit communication bus objects

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

bool sfeBmv080::driverVersion()
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

bool sfeBmv080::ID()
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

uint16_t sfeBmv080::dutyCyclingPeriod()
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

float sfeBmv080::volumetricMassDensity()
{
    float volumetric_mass_density = 0.0;
    bmv080_status_code_t bmv080_current_status =
        bmv080_get_parameter(bmv080_handle_class, "volumetric_mass_density", (void *)&volumetric_mass_density);
    if (bmv080_current_status != E_BMV080_OK)
    {
// TODO: libraries should not output text by default, need to add a debug mode/flag to library?        
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
        // TODO: libraries should not output text by default, need to add a debug mode/flag to library?
        printf("Error setting BMV080 Volumetric Mass Density: %d\n", bmv080_current_status);
        return false;
    }
    else
    {
        return true;
    }
}

float sfeBmv080::integrationTime()
{
    float integration_time = 0.0;
    bmv080_status_code_t bmv080_current_status =
        bmv080_get_parameter(bmv080_handle_class, "integration_time", (void *)&integration_time);
    if (bmv080_current_status != E_BMV080_OK)
    { // todo -- no printf in library
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

uint32_t sfeBmv080::distributionId()
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

bool sfeBmv080::doObstructionDetection()
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

bool sfeBmv080::doVibrationFiltering()
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

uint8_t sfeBmv080::measurementAlgorithm()
{
    bmv080_measurement_algorithm_t measurement_algorithm;
    bmv080_status_code_t bmv080_current_status =
        bmv080_get_parameter(bmv080_handle_class, "measurement_algorithm", (void *)&measurement_algorithm);
    if (bmv080_current_status != E_BMV080_OK)
    {
        printf("Error getting BMV080 Measurement Algorithm: %d\n", bmv080_current_status);
        return 0;
    }
    else
    {
        return (uint8_t)measurement_algorithm;
    }
}

bool sfeBmv080::setMeasurementAlgorithm(uint8_t measurement_algorithm)
{
    bmv080_measurement_algorithm_t bmv080_measurement_algorithm = (bmv080_measurement_algorithm_t)measurement_algorithm;
    bmv080_status_code_t bmv080_current_status =
        bmv080_set_parameter(bmv080_handle_class, "measurement_algorithm", (void *)&bmv080_measurement_algorithm);
    if (bmv080_current_status != E_BMV080_OK)
    {
        printf("Error setting BMV080 Measurement Algorithm: %d\n", bmv080_current_status);
        return false;
    }
    else
    {
        return true;
    }
}