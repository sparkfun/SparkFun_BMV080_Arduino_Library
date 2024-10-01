/******************************************************************************
    sfeBmv080.h
    SparkFun BMV080 Library header file

    by Pete Lewis @SparkFun Electronics
    September 2024

    This file implements the BMV080 class, prototyped in SparkFun_BMV080_Arduino_Library.h

    Development environment specifics:
    IDE: Arduino 2.3.3
    Hardware Platform: SparkFun IoT Redboard ESP32
    BMV080 Breakout HW Version: v01

    SPDX-License-Identifier: MIT

    Copyright (c) 2024 SparkFun Electronics

    Distributed as-is; no warranty is given.
******************************************************************************/

#pragma once

#include "bmv080.h"
#include "bmv080_defs.h"
#include "combridge.h"

#include <SparkFun_Toolkit.h>
#include <stdint.h>

#define SFE_BMV080_DEFAULT_ADDRESS 0x57
#define SFE_BMV080_DEFAULT_IRQ_PIN 14

#define SFE_BMV080_MODE_CONTINUOUS 0
#define SFE_BMV080_MODE_DUTY_CYCLE 1



class sfeBmv080
{
  public:
    /// @brief Default constructor
    sfeBmv080() : _theBus{nullptr}
    {
    }

    /// @brief Begins the Device
    /// @param theBus I2C bus to use for communication
    /// @return 0 for succuss, negative for errors, positive for warnings
    sfeTkError_t begin(sfeTkII2C *theBus = nullptr);

    /// @brief Checks if the Device is connected
    /// @return 0 for succuss, negative for errors, positive for warnings
    sfeTkError_t isConnected();

    /// @brief Initialize the sensor
    /// @details This function initializes the sensor and should be called 
    /// before any other functions. It calls Open, Reset, getDriverVersion, and getID.
    /// @param i2c_device The I2C device to use
    /// @return True if successful, false otherwise
    bool init(i2c_device_t *i2c_device);

    /// @brief Get the version information of this sensor driver.
    /// @return True if successful, false otherwise
    bool getDriverVersion();

    /// @brief Open a sensor unit by initializing a new handle.
    /// @param i2c_device The I2C device to use
    /// @return True if successful, false otherwise
    bool open(i2c_device_t *i2c_device);

    /// @brief Reset the sensor
    /// @return True if successful, false otherwise
    bool reset();

    /// @brief Get the ID of the sensor
    /// @return True if successful, false otherwise
    bool getID();

    /// @brief Set the mode of the sensor
    /// @param mode SFE_BMV080_MODE_CONTINUOUS, SFE_BMV080_MODE_DUTY_CYCLE
    /// @return True if successful, false otherwise
    bool setMode(uint8_t mode);

    /// @brief Get the PM2.5 value
    /// @return The PM2.5 value as a float in ug/m3
    float getPM25();

    /// @brief Get the obstruction status
    /// @return True if obstructed, false otherwise
    bool getIsObstructed();


    void setSensorValue(bmv080_output_t bmv080_output);
    
    /// @brief Check if new data is available
    /// @details This function should be called in the main loop to check if new data is available
    /// @details If new data is available, the data can be read using getPM25 and getIsObstructed
    /// @return True if new data is available, false otherwise
    bool dataAvailable();

    /// @brief Get the duty cycling period
    /// @return The duty cycling period in seconds
    uint16_t getDutyCyclingPeriod();

    /// @brief Set the duty cycling period
    /// @param period The duty cycling period in seconds
    /// @return True if successful, false otherwise
    bool setDutyCyclingPeriod(uint16_t duty_cycling_period);

  private:
    bmv080_handle_t bmv080_handle_class = NULL;
    bool _dataAvailable = false;
    bmv080_output_t _sensorValue;

  protected:
    sfeTkII2C *_theBus;
};