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

sfeTkError_t sfeBmv080::begin(sfeTkII2C *theBus)
{
    // Nullptr check
    if (theBus == nullptr)
        return kSTkErrFail;

    // Set bus pointer
    _theBus = theBus;

    sfeTkError_t err;
    err = isConnected();
    // Check whether the ping was successful
    if (err != kSTkErrOk)
        return err;

    // Done!
    return kSTkErrOk;
}

sfeTkError_t sfeBmv080::isConnected()
{
    // Just ping the device address
    return _theBus->ping();
}