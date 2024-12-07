/*
 * periph_i2c.c
 *
 *  Created on: Oct 11, 2024
 *      Author: markwilkinson
 */

#include "periph_i2c.h"
#include "ssd1306.h"
#include "ccs811.h"
#include <string.h>

extern I2C_HandleTypeDef hi2c1;

// Max timeout in microseconds
#define I2C_MAX_TIMEOUT 100u

bool periph_i2c_tx(uint16_t device_add, uint8_t reg_add, uint8_t *data, uint16_t data_len)
{
	HAL_StatusTypeDef i2c_tx_okay = HAL_I2C_Mem_Write(&hi2c1, device_add, reg_add, 1, data, data_len, I2C_MAX_TIMEOUT);
	return i2c_tx_okay == HAL_OK;
}

bool periph_i2c_rx(uint16_t device_add, uint8_t reg_add, uint8_t *rx_data, uint8_t size)
{
	HAL_StatusTypeDef i2c_rx_okay = HAL_I2C_Mem_Read(&hi2c1, device_add, reg_add, 1, rx_data, size, I2C_MAX_TIMEOUT);
	return i2c_rx_okay == HAL_OK;
}

