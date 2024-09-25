/**
 * Copyright (C) Bosch Sensortec GmbH. All Rights Reserved. Confidential.
 *
 * Disclaimer
 *
 * Common:
 * Bosch Sensortec products are developed for the consumer goods industry. They may only be used
 * within the parameters of the respective valid product data sheet. Bosch Sensortec products are
 * provided with the express understanding that there is no warranty of fitness for a particular purpose.
 * They are not fit for use in life-sustaining, safety or security sensitive systems or any system or device
 * that may lead to bodily harm or property damage if the system or device malfunctions. In addition,
 * Bosch Sensortec products are not fit for use in products which interact with motor vehicle systems.
 * The resale and/or use of products are at the purchaser's own risk and his own responsibility. The
 * examination of fitness for the intended use is the sole responsibility of the Purchaser.
 *
 * The purchaser shall indemnify Bosch Sensortec from all third party claims, including any claims for
 * incidental, or consequential damages, arising from any product use not covered by the parameters of
 * the respective valid product data sheet or not approved by Bosch Sensortec and reimburse Bosch
 * Sensortec for all costs in connection with such claims.
 *
 * The purchaser must monitor the market for the purchased products, particularly with regard to
 * product safety and inform Bosch Sensortec without delay of all security relevant incidents.
 *
 * Engineering Samples are marked with an asterisk (*) or (e). Samples may vary from the valid
 * technical specifications of the product series. They are therefore not intended or fit for resale to third
 * parties or for use in end products. Their sole purpose is internal client testing. The testing of an
 * engineering sample may in no way replace the testing of a product series. Bosch Sensortec
 * assumes no liability for the use of engineering samples. By accepting the engineering samples, the
 * Purchaser agrees to indemnify Bosch Sensortec from all claims arising from the use of engineering
 * samples.
 *
 * Special:
 * This software module (hereinafter called "Software") and any information on application-sheets
 * (hereinafter called "Information") is provided free of charge for the sole purpose to support your
 * application work. The Software and Information is subject to the following terms and conditions:
 *
 * The Software is specifically designed for the exclusive use for Bosch Sensortec products by
 * personnel who have special experience and training. Do not use this Software if you do not have the
 * proper experience or training.
 *
 * This Software package is provided `` as is `` and without any expressed or implied warranties,
 * including without limitation, the implied warranties of merchantability and fitness for a particular
 * purpose.
 *
 * Bosch Sensortec and their representatives and agents deny any liability for the functional impairment
 * of this Software in terms of fitness, performance and safety. Bosch Sensortec and their
 * representatives and agents shall not be liable for any direct or indirect damages or injury, except as
 * otherwise stipulated in mandatory applicable law.
 *
 * The Information provided is believed to be accurate and reliable. Bosch Sensortec assumes no
 * responsibility for the consequences of use of such Information nor for any infringement of patents or
 * other rights of third parties which may result from its use. No license is granted by implication or
 * otherwise under any patent or patent rights of Bosch. Specifications mentioned in the Information are
 * subject to change without notice.
 *
 * It is not allowed to deliver the source code of the Software to any third party without permission of
 * Bosch Sensortec.
 *
 * @file combridge.h
 *
 * @brief Header for the serial communication interface functions in combridge.c.
 */

#ifndef COMBRIDGE_H_
#define COMBRIDGE_H_

/* Includes ------------------------------------------------------------------*/
#include "bmv080.h"
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>


#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Exported types ------------------------------------------------------------*/

/*!
* @brief SPI device structure
*/
typedef struct
{
    /*! Instance of arduino SPI protocol instance */
    SPIClass *instance;
    /*! Instance of arduino SPI settings to be applied before every transmission */
    SPISettings settings;
} spi_device_t;

/*!
* @brief I2C device structure
*/
typedef struct
{
    /*! Instance of arduino I2C protocol instance */
    TwoWire *instance;
} i2c_device_t;

/* Exported constants --------------------------------------------------------*/
/*! 0: Status codes returned when there is no warning or error */
#define E_COMBRIDGE_OK                       ((int8_t)0)
/*! -1: Status codes returned when memory allocation fails */
#define E_COMBRIDGE_ERROR_MEMORY_ALLOCATION  ((int8_t)-1)
/*! -2: Status codes returned when the read operation fails */
#define E_COMBRIDGE_ERROR_READ               ((int8_t)-2)
/*! -3: Status codes returned when the write operation fails */
#define E_COMBRIDGE_ERROR_WRITE              ((int8_t)-3)
/*! -4: Status codes returned when writing the header fails */
#define E_COMBRIDGE_ERROR_WRITE_HEADER       ((int8_t)-4)
/*! -5:  Status codes returned when a reference is null */
#define E_COMBRIDGE_ERROR_NULLPTR            ((int8_t)-5)

/* Exported functions prototypes ---------------------------------------------*/

/* Initialization functions */
void spi_init(spi_device_t *spi_device);
void i2c_init(i2c_device_t *i2c_device);

/* SPI read and write functions */
int8_t combridge_spi_read_16bit(bmv080_sercom_handle_t handle, uint16_t header, uint16_t *payload, uint16_t payload_length);
int8_t combridge_spi_write_16bit(bmv080_sercom_handle_t handle, uint16_t header, const uint16_t *payload, uint16_t payload_length);

/* I2C read and write functions */
int8_t combridge_i2c_read_16bit(bmv080_sercom_handle_t handle, uint16_t header, uint16_t *payload, uint16_t payload_length);
int8_t combridge_i2c_write_16bit(bmv080_sercom_handle_t handle, uint16_t header, const uint16_t *payload, uint16_t payload_length);

/* Delay function */
int8_t combridge_delay(uint32_t period);


#ifdef __cplusplus
}
#endif

#endif /* COMBRIDGE_H_ */
