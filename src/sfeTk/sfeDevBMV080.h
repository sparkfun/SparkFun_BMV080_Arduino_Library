/******************************************************************************
    sfeDevBMV080.h
    SparkFun BMV080 Library header file

    by Pete Lewis @SparkFun Electronics
    September 2025

    This file implements the BMV080 class, prototyped in SparkFun_BMV080_Arduino_Library.h

    BMV080 Breakout HW Version: v01

    SPDX-License-Identifier: MIT

    Copyright (c) 2025 SparkFun Electronics

    Distributed as-is; no warranty is given.
******************************************************************************/

#pragma once

#include "bmv080.h"
#include "bmv080_defs.h"

// Include the platform independent layer of the SparkFun Toolkit
#include <sfeTk/sfeToolkit.h>
#include <stdint.h>

#define SFE_BMV080_DEFAULT_ADDRESS 0x57
#define SFE_BMV080_DEFAULT_IRQ_PIN 14

#define SFE_BMV080_MODE_CONTINUOUS 0
#define SFE_BMV080_MODE_DUTY_CYCLE 1

class sfeDevBMV080
{
  public:
    /// @brief Default constructor
    sfeDevBMV080() : _theBus{nullptr}
    {
    }

    /// @brief Begins the Device
    /// @param theBus SparkFun Toolkit bus to use for communication
    /// @return 0 for succuss, negative for errors, positive for warnings
    sfeTkError_t begin(sfeTkIBus *theBus = nullptr);

    /// @brief Initialize the sensor
    /// @details This function initializes the sensor and should be called
    /// @details before any other functions. It calls Open, Reset, getDriverVersion, and getID.
    /// @return True if successful, false otherwise
    bool init(void);

    /// @brief Get the version information of this sensor driver - the vendor supplied version.
    /// @param major Major version number
    /// @param minor Minor version number
    /// @param patch Patch version number
    /// @return True if successful, false otherwise
    bool driverVersion(uint16_t &major, uint16_t &minor, uint16_t &patch);

    /// @brief Open a sensor unit by initializing a new handle.
    /// @return True if successful, false otherwise
    bool open(void);

    /// @brief Reset the sensor
    /// @return True if successful, false otherwise
    bool reset(void);

    // ID Buffer len
    static const size_t kBMV800IDLength = 13;
    /// @brief Get the ID of the sensor
    /// @param idOut Buffer to return the ID in - must be 13 bytes long
    /// @return True if successful, false otherwise
    bool ID(char idOut[kBMV800IDLength]);

    /// @brief Set the mode of the sensor
    /// @param mode SFE_BMV080_MODE_CONTINUOUS, SFE_BMV080_MODE_DUTY_CYCLE
    /// @return True if successful, false otherwise
    bool setMode(uint8_t mode);

    /// @brief Get the PM2.5 value
    /// @return The PM2.5 value as a float in ug/m3
    float PM25(void);

    /// @brief Get the PM1 value
    /// @return The PM1 value as a float in ug/m3
    float PM1(void);

    /// @brief Get the obstruction status
    /// @return True if obstructed, false otherwise
    bool isObstructed();

    // "Internal" method to set the se
    void setSensorValue(bmv080_output_t bmv080_output);

    /// @brief Get the sensor value, update internal value cache and return the value if requested
    /// @param bmv080_output pointer to value output struct - if nullptr, no data is returned
    /// @return true on success, false if no data is available
    bool readSensor(bmv080_output_t *bmv080_output = nullptr);

    /// @brief Get the duty cycling period
    /// @return The duty cycling period in seconds
    uint16_t dutyCyclingPeriod();

    /// @brief Set the duty cycling period
    /// @param period The duty cycling period in seconds
    /// @return True if successful, false otherwise
    bool setDutyCyclingPeriod(uint16_t duty_cycling_period);

    /// @brief Get a parameter: "volumetric_mass_density"
    /// @return float volumetric_mass_density
    float volumetricMassDensity();

    /// @brief Set a parameter: "volumetric_mass_density"
    /// @param volumetric_mass_density
    /// @return True if successful, false otherwise
    bool setVolumetricMassDensity(float volumetric_mass_density);

    /// @brief Get a parameter: "integration_time"
    /// @return float integration_time
    float integrationTime();

    /// @brief Set a parameter: "integration_time"
    /// @param integration_time
    /// @return True if successful, false otherwise
    bool setIntegrationTime(float integration_time);

    /// @brief Get a parameter: "distribution_id"
    /// @return uint32_t distribution_id
    uint32_t distributionId();

    /// @brief Set a parameter: "distribution_id"
    /// @param distribution_id
    /// @return True if successful, false otherwise
    bool setDistributionId(uint32_t distribution_id);

    /// @brief Get a parameter: "do_obstruction_detection"
    /// @return bool do_obstruction_detection
    bool doObstructionDetection();

    /// @brief Set a parameter: "do_obstruction_detection"
    /// @param do_obstruction_detection
    /// @return True if successful, false otherwise
    bool setDoObstructionDetection(bool do_obstruction_detection);

    /// @brief Get a parameter: "do_vibration_filtering"
    /// @return bool do_vibration_filtering
    bool doVibrationFiltering();

    /// @brief Set a parameter: "do_vibration_filtering"
    /// @param do_vibration_filtering
    /// @return True if successful, false otherwise
    bool setDoVibrationFiltering(bool do_vibration_filtering);

    /// @brief Get a parameter: "measurement_algorithm"
    /// @return uint8_t measurement_algorithm
    uint8_t measurementAlgorithm();

    /// @brief Set a parameter: "measurement_algorithm"
    /// @param measurement_algorithm
    /// @return True if successful, false otherwise
    bool setMeasurementAlgorithm(uint8_t measurement_algorithm);

  private:
    // bosch bmv080 library callback functions (static methods to be used as callbacks)
    static int8_t device_read_16bit_CB(bmv080_sercom_handle_t, uint16_t, uint16_t *, uint16_t);
    static int8_t device_write_16bit_CB(bmv080_sercom_handle_t, uint16_t, const uint16_t *, uint16_t);
    static int8_t device_delay_CB(uint32_t);
    static void set_sensor_value(bmv080_output_t, void *);

    /// @brief Called to pump the service routine of the BMV080 sensor driver
    /// @return True on success, false on failure
    bool sensorServiceRoutine(void);

    bmv080_handle_t _bmv080_handle_class = NULL;
    bool _dataAvailable = false;
    bmv080_output_t _sensorValue;

  protected:
    sfeTkIBus *_theBus;
};