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

#include <SparkFun_Toolkit.h>
#include <stdint.h>

#define SFE_BMV080_DEFAULT_ADDRESS 0x57
#define SFE_BMV080_DEFAULT_IRQ_PIN 14



class sfeBmv080
{
  public:
    /// @brief Default constructor
    sfeBmv080()// : _theBus{nullptr}
    {
    }

    // /// @brief Begins the Device
    // /// @param theBus I2C bus to use for communication
    // /// @return 0 for succuss, negative for errors, positive for warnings
    // sfeTkError_t begin(sfeTkII2C *theBus = nullptr);

    // /// @brief Checks if the Device is connected
    // /// @return 0 for succuss, negative for errors, positive for warnings
    // sfeTkError_t isConnected();

    void setHandle(bmv080_handle_t handle);

    float getPM25();
    bool getIsObstructed();
    //void use_sensor_output(bmv080_output_t bmv080_output, void* callback_parameters);
    //void bmv080_service_routine(void);
    //bmv080_handle_t bmv080_handle = NULL;
    //print_function_t print_handle = NULL; 
    //void print_to_serial(const char *format, ...);
    void setSensorValue(bmv080_output_t bmv080_output);
    //void setSensorValue(float pm25);
    //void bmv080_service_routine(void);
    bmv080_output_t _sensorValue;
    bool dataAvailable();

    bmv080_handle_t bmv080_handle_class = NULL;

  private:
    bool _dataAvailable = false;

  // protected:
  //   sfeTkII2C *_theBus;
};