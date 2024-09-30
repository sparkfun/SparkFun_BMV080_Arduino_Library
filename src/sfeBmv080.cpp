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
#include "bmv080_defs.h"
#include "bmv080.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void bmv080_service_routine(const bmv080_handle_t handle, void* callback_parameters);
void use_sensor_output(bmv080_output_t bmv080_output, void* callback_parameters);


/* Custom function for consuming sensor readings */
void use_sensor_output(bmv080_output_t bmv080_output, void* callback_parameters)
{
    //data_ready_callback_count += 1;
    //print_function_t print = (print_function_t)callback_parameters;


    ((sfeBmv080*)callback_parameters)->setSensorValue(bmv080_output);
    
    //Serial.println(bmv080_output.pm2_5);
    
    //Serial.println("u");

    //sfeBmv080::_sensorValue.pm2_5 = bmv080_output.pm2_5; // update the class variable with the new PM2.5 value
    //setSensorValue(bmv080_output.pm2_5);
    //sensorValue.pm2_5 = bmv080_output.pm2_5;
    // print("Runtime: %.2f s, PM2.5: %.0f ug/m^3, obstructed: %s, outside detection limits: %s\r\n",
    //     bmv080_output.runtime_in_sec, bmv080_output.pm2_5, (bmv080_output.is_obstructed ? "yes" : "no"), (bmv080_output.is_outside_detection_limits ? "yes" : "no"));
}


void bmv080_service_routine(const bmv080_handle_t handle, void* callback_parameters)
{       
        // if ( (bmv080_handle != NULL) && (print_handle != NULL))
        // {
            //Serial.print("s");
            /* The interrupt is served by the BMV080 sensor driver */
            bmv080_status_code_t bmv080_current_status = bmv080_serve_interrupt(handle, (bmv080_callback_data_ready_t)use_sensor_output, callback_parameters);
            //bmv080_status_code_t bmv080_current_status = bmv080_serve_interrupt(bmv080_handle, (bmv080_callback_data_ready_t)use_sensor_output, (void*)print_handle);
            if (bmv080_current_status != E_BMV080_OK)
            {
                //printf("e %d\r\n", (int32_t)bmv080_current_status);
            }
        //}
}


#ifdef __cplusplus
}
#endif


// sfeTkError_t sfeBmv080::begin(sfeTkII2C *theBus)
// {
//     // Nullptr check
//     if (theBus == nullptr)
//         return kSTkErrFail;

//     // Set bus pointer
//     _theBus = theBus;

//     sfeTkError_t err;
//     err = isConnected();
//     // Check whether the ping was successful
//     if (err != kSTkErrOk)
//         return err;

//     // Done!
//     return kSTkErrOk;
// }

// sfeTkError_t sfeBmv080::isConnected()
// {
//     // Just ping the device address
//     return _theBus->ping();
// }

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

    if(mode == SFE_BMV080_MODE_CONTINUOUS)
    {
        bmv080_current_status = bmv080_start_continuous_measurement(bmv080_handle_class);
    }
    else if(mode == SFE_BMV080_MODE_DUTY_CYCLE)
    {
        bmv080_current_status = bmv080_start_duty_cycling_measurement(bmv080_handle_class, (bmv080_callback_tick_t)millis, E_BMV080_DUTY_CYCLING_MODE_0);
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

// void print_to_serial(const char *format, ...) 
// {
//     char print_buffer[1024];
//     va_list args;
//     va_start(args, format);
//     vsnprintf(print_buffer, sizeof(print_buffer), format, args);
//     va_end(args);
//     Serial.print(print_buffer);
// }

//print_function_t print_handle = (const print_function_t)print_to_serial;


void sfeBmv080::setHandle(bmv080_handle_t handle)
{
    bmv080_handle_class = handle;
}

bool sfeBmv080::dataAvailable()
{
    bmv080_service_routine(bmv080_handle_class, this);
    if(_dataAvailable == true)
    {
        _dataAvailable = false;
        return true;
    }
    else
        return false;
}