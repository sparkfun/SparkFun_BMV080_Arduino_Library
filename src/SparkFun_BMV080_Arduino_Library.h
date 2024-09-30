/******************************************************************************
    SparkFun_BMV080_Arduino_Library.h
    SparkFun BMV080 Library header file

    by Pete Lewis @SparkFun Electronics
    September 2024

    Based on original source code written by
        Fischer Moseley @ SparkFun Electronics
        Original Creation Date: July 24, 2019

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

// TODO: Add includes as needed (e.g. #include <Wire.h>, #include <SPI.h>)
#include "sfeBmv080.h"
//#include <SparkFun_Toolkit.h>

// #include "Arduino.h"
#include <bmv080.h>
#include <bmv080_defs.h>
#include "combridge.h"

// The BMV080 Bosch API requires a larger than usual stack size
// In particular, bmv080_serve_interrupt is the culprit.
SET_LOOP_TASK_STACK_SIZE(60 * 1024);  // 60KB

class Bmv080 : public sfeBmv080
{
  public:
    // /// @brief Begins the Device
    // /// @param address I2C device address to use for the sensor
    // /// @param wirePort Wire port to use for I2C communication
    // /// @return True if successful, false otherwise
    // bool begin(const uint8_t address = SFE_BMV080_DEFAULT_ADDRESS, TwoWire &wirePort = Wire, const uint8_t irqPin = SFE_BMV080_DEFAULT_IRQ_PIN)
    // {
    //     // Setup Arudino I2C bus
    //     _theI2CBus.init(wirePort, address);

    //     // Begin the sensor
    //     return sfeBmv080::begin(&_theI2CBus) == kSTkErrOk;
    // }

    // /// @brief Checks if the Device is connected
    // /// @return True if the sensor is connected, false otherwise
    // bool isConnected()
    // {
    //     return sfeBmv080::isConnected() == kSTkErrOk;
    // }

//   private:
//     sfeTkArdI2C _theI2CBus;
};
