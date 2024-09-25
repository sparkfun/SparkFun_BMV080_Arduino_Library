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
 * @file combridge.cpp
 *
 * @brief This file contains the serial communication interface (e.g. SPI and I2C) functions. 
 * Messages can be read and written. In addition, there is a waiting mechanism that waits for a defined time.
 */


/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include "combridge.h"

/* Private define ------------------------------------------------------------*/
#define SPI_CLK_FREQ          ((uint32_t)(1e6))

#define I2C_CLK_FREQ          ((uint32_t)(100e3))

/* BMV080 I2C address
 * note that the BMV080 pins are connected such that I2C Address Bit 0 = 0 and I2C Address Bit 1 = 0
 */
#define BMV080_I2C_ADDRESS    0x57


/* Exported functions --------------------------------------------------------*/
void spi_init(spi_device_t *spi_device)
{
    SPISettings spi_settings(SPI_CLK_FREQ, MSBFIRST, SPI_MODE0);
    spi_device->instance = &SPI;
    spi_device->settings = spi_settings;

    pinMode(SS, OUTPUT);
    pinMode(MOSI, OUTPUT);
    pinMode(SCK, OUTPUT);
    pinMode(MISO, INPUT);
    digitalWrite(SS, HIGH);
    spi_device->instance->begin();
}


void i2c_init(i2c_device_t *i2c_device)
{
    i2c_device->instance = &Wire;
    i2c_device->instance->begin();
    i2c_device->instance->setClock(I2C_CLK_FREQ);
}


int8_t combridge_spi_read_16bit(bmv080_sercom_handle_t handle, uint16_t header, uint16_t *payload, uint16_t payload_length)
{
    int8_t return_value = E_COMBRIDGE_OK;
    spi_device_t *spi_device = (spi_device_t *)handle;

    digitalWrite(SS, LOW);
    spi_device->instance->beginTransaction(spi_device->settings);    
    spi_device->instance->transfer16(header);

    uint16_t payload_index = 0;
    for (; payload_index < payload_length; payload_index++)
    {
        payload[payload_index] = spi_device->instance->transfer16(0);
    }
    spi_device->instance->endTransaction();
    digitalWrite(SS, HIGH);
    
    return return_value;
}


int8_t combridge_spi_write_16bit(bmv080_sercom_handle_t handle, uint16_t header, const uint16_t *payload, uint16_t payload_length)
{
    int8_t return_value = E_COMBRIDGE_OK;
    spi_device_t *spi_device = (spi_device_t *)handle;

    digitalWrite(SS, LOW);
    spi_device->instance->beginTransaction(spi_device->settings);
    spi_device->instance->transfer16(header);

    uint16_t payload_index = 0;
    for (; payload_index < payload_length; payload_index++)
    {
        spi_device->instance->transfer16(payload[payload_index]);
    }
    spi_device->instance->endTransaction();
    digitalWrite(SS, HIGH);

    return E_COMBRIDGE_OK;
}


int8_t combridge_i2c_read_16bit(bmv080_sercom_handle_t handle, uint16_t header, uint16_t *payload, uint16_t payload_length)
{
    int8_t return_value = E_COMBRIDGE_OK;
    i2c_device_t *i2c_device = (i2c_device_t *)handle;
    uint8_t * payload_byte = (uint8_t*)payload;

    /* 16-bit header left shifted by 1, since the R/W bit (most significant bit) is passed along with the 7-bit device address  */
    uint16_t header_adjusted = header << 1;

    i2c_device->instance->beginTransmission(BMV080_I2C_ADDRESS);
    i2c_device->instance->write((header_adjusted >> 8) & 0xFF); 
    i2c_device->instance->write(header_adjusted & 0xFF);
    
    if(i2c_device->instance->endTransmission(true) != 0)
    {
        return_value = E_COMBRIDGE_ERROR_WRITE_HEADER;
        return return_value;
    }

    i2c_device->instance->requestFrom(BMV080_I2C_ADDRESS, payload_length * 2);

    uint16_t payload_index = 0;
    while(i2c_device->instance->available() && (payload_index < (payload_length * 2) )) 
    {
        payload_byte[payload_index++] = i2c_device->instance->read();    
    }

    if(payload_index != (payload_length * 2))
    {
       return_value = E_COMBRIDGE_ERROR_READ;
    }

    /* Conversion of payload from big endian to little endian */
    for (payload_index = 0; payload_index < payload_length; payload_index++)
    {
        uint16_t swapped_word = ((payload[payload_index] << 8) | (payload[payload_index] >> 8)) & 0xffff;
        payload[payload_index] = swapped_word;
    }

    return return_value;
}


int8_t combridge_i2c_write_16bit(bmv080_sercom_handle_t handle, uint16_t header, const uint16_t *payload, uint16_t payload_length)
{
    int8_t return_value = E_COMBRIDGE_OK;

    i2c_device_t *i2c_device = (i2c_device_t *)handle;
    uint8_t * payload_byte = (uint8_t*)payload;

     /* 16-bit header left shifted by 1, since the R/W bit (most significant bit) is passed along with the 7-bit device address  */
    uint16_t header_adjusted = header << 1;

    /* Conversion of payload from little endian to big endian (dynamic allocation is used) */
    uint16_t *payload_swapped = (uint16_t *)calloc(payload_length, sizeof(uint16_t));
    if(payload_swapped)
    {
        for (uint16_t payload_index = 0; payload_index < payload_length; payload_index++)
        {
            payload_swapped[payload_index] = ((payload[payload_index] << 8) | (payload[payload_index] >> 8)) & 0xffff;
        }
    } 
    else 
    {
        return_value = E_COMBRIDGE_ERROR_WRITE_HEADER;
        return return_value;
    }

    i2c_device->instance->beginTransmission(BMV080_I2C_ADDRESS);
    i2c_device->instance->write((header_adjusted >> 8) & 0xFF); 
    i2c_device->instance->write(header_adjusted & 0xFF);
    i2c_device->instance->write((uint8_t *)payload_swapped, payload_length * 2);
    
    if(i2c_device->instance->endTransmission(true) != 0)
    {
        return_value = E_COMBRIDGE_ERROR_WRITE;
    }

    free(payload_swapped);

    return return_value;
}


int8_t combridge_delay(uint32_t period)
{
    delay(period);

    return E_COMBRIDGE_OK;
}
