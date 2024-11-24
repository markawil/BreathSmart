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
static bool module_init = false;

// Max timeout in microseconds
#define I2C_MAX_TIMEOUT 100u

// init the i2c handle by setting the local variable, then init the i2c devices.
bool periph_i2c_init()
{
	// MPU6050 doesn't need an init, just the ssd1306 display.
	module_init = true;
	ssd1306_Init(); // init the OLED display driver.
	module_init = ccs811_init(); // init the air quality sensor
	return module_init;
}

bool periph_i2c_tx(uint16_t device_add, uint8_t reg_add, uint8_t *data, uint16_t data_len)
{
	bool tx_okay = false;
	if (module_init)
	{
		HAL_StatusTypeDef i2c_tx_okay = HAL_I2C_Mem_Write(&hi2c1, device_add, reg_add, 1, data, data_len, I2C_MAX_TIMEOUT);
		// only if we get the HAL_OK result
		if(i2c_tx_okay == HAL_OK)
		{
			tx_okay = true;
		}
	}

	return tx_okay;
}

bool periph_i2c_rx(uint16_t device_add, uint8_t reg_add, uint8_t *rx_data, uint8_t size)
{
	bool rx_okay = false;
	if(module_init)
	{
		HAL_StatusTypeDef i2c_rx_okay = HAL_I2C_Mem_Read(&hi2c1, device_add, reg_add, 1, rx_data, size, I2C_MAX_TIMEOUT);
		assert(i2c_rx_okay == HAL_OK);
		if(i2c_rx_okay == HAL_OK)
		{
			rx_okay = true;
		}
	}
	return rx_okay;
}

