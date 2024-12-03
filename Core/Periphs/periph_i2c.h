/*
 * periph_i2c.h
 *
 *  Created on: Oct 11, 2024
 *      Author: markwilkinson
 */

#ifndef PERIPHS_PERIPH_I2C_H_
#define PERIPHS_PERIPH_I2C_H_

#include "stm32f3xx_hal.h"

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#define I2C_MAX_DATA_SIZE 1u

/*!
 * \brief     This function will write the given data to a given register address on the i2c bus.
 * \param[in] device_add - Address of the device we want to write data to.
 * \param[in] reg_add - Address of the register we want to write data to.
 * \param[in] data - Pointer to the data that we want to write over I2C.
 * \param[in] data_len - length of the data in bytes to be sent.
 * \return    tx_okay - True if the transmit of the register data was okay, false if not.
 */
bool periph_i2c_tx(uint16_t device_add, uint8_t reg_add, uint8_t *data, uint16_t data_len);

/*!
 * \brief     This function will read data from a device on the i2c bus.
 * \param[in] device_add - Address of the device we want to read from.
 * \param[in] reg_add - Address of the register we want to read data from.
 * \param[in] data - Pointer to the data buffer that will be written to.
 * \param[in] size - length of the data in bytes to read.
 * \return    rx_okay - True if the read of the register data was okay, false if not.
 */
bool periph_i2c_rx(uint16_t device_add, uint8_t reg_add, uint8_t *rx_data, uint8_t size);


#endif /* PERIPHS_PERIPH_I2C_H_ */
