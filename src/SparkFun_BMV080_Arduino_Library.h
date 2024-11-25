/******************************************************************************
    SparkFun_BMV080_Arduino_Library.h
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

#include "sfeBmv080.h"
#include <SparkFun_Toolkit.h>

// The BMV080 Bosch API requires a larger than usual stack size
// In particular, bmv080_serve_interrupt is the culprit.
SET_LOOP_TASK_STACK_SIZE(60 * 1024); // 60KB

class SparkFunBMV080I2C : public sfeBmv080
{
  public:
    /// @brief Begins the Device
    /// @param address I2C device address to use for the sensor
    /// @param wirePort Wire port to use for I2C communication
    /// @return True if successful, false otherwise
    bool begin(const uint8_t address = SFE_BMV080_DEFAULT_ADDRESS, TwoWire &wirePort = Wire)
    {
        // Setup Arudino I2C bus
        _theI2CBus.init(wirePort, address);
        _theI2CBus.setByteOrder(SFTK_MSBFIRST);

        // Begin the sensor
        sfeTkError_t rc = sfeBmv080::begin(&_theI2CBus);

        return rc == kSTkErrOk ? isConnected() : false;
    }

    /// @brief Checks if the Device is connected
    /// @return True if the sensor is connected, false otherwise
    bool isConnected()
    {
        return _theI2CBus.ping() == kSTkErrOk;
    }

  private:
    sfeTkArdI2C _theI2CBus;
};

class SparkFunBMV080SPI : public sfeBmv080
{
  public:
    /// @brief Begins the Device with SPI as the communication bus
    /// @param csPin The chip select pin for the sensor
    /// @param spiPort The SPI port to use for communication
    /// @param spiSettings The SPI settings to use for communication
    /// @return True if successful, false otherwise
    bool begin(uint8_t csPin, SPIClass &spiPort = SPI, SPISettings spiSettings = SPISettings(100000, MSBFIRST, SPI_MODE0))
    {

        // Setup Arduino SPI bus
        _theSPIBus.init(spiPort, spiSettings, csPin, true);

        // Begin the sensor
        sfeTkError_t rc = sfeBmv080::begin(&_theSPIBus);

        return rc == kSTkErrOk ? true : false;
    }

  private:
    sfeTkArdSPI _theSPIBus;
};