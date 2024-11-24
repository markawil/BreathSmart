/*
 * mpu6050.c
 *
 *  Created on: Oct 11, 2024
 *      Author: markwilkinson
 */

#include "periph_i2c.h"
#include "mpu6050_reg_map.h"
#include "mpu6050.h"

#include <string.h>

// private variables
#define MPU6050_ADDRESS MPU6050_DEV_ADD_LOW // device address because pin is set to low.
#define WHO_AM_I_VALUE 0x68u // default value of the device register

static bool device_init_complete_s = false;

// private function prototypes
bool who_am_i(void);
bool write_accel_config(void);
uint8_t read_register(uint8_t reg_add);
bool write_register(uint8_t reg_add, uint8_t reg_val);

/******************* PUBLIC FUNCTIONS *******************/
/*!
 * \brief     Init function used to setup this module and the accelerometer for sampling.
 * \return    device_init_complete_s - True is the device is initialized okay, false if not.
 */
bool mpu6050_init(void)
{
	device_init_complete_s = (who_am_i() == true) ? true : false;
	if (device_init_complete_s == false)
	{
		return false; // early termination if get_who_am_i() failed
	}

	// Need to take the accelerometer out of sleep mode.
	uint8_t data = 0x00;
	bool pwr_mgmt_write = write_register(MPU6050_REG_PWR_MGMT_1, data);
	assert(pwr_mgmt_write);

	// Set data rate to 1KHz by writing MPU6050_REG_SMPRT_DIV register.
	data = 0x07;
	bool set_data_rate = write_register(MPU6050_REG_SMPRT_DIV, data);
	assert(set_data_rate);

	// Set accelerometer configuration to +-2g.
	data = 0x00;
	bool set_accel = write_register(MPU6050_REG_ACCEL_CONFIG, data);
	assert(set_accel);

	return true;
}

/*!
 * \brief     Reads and formats the X Axis data from the MPU6050.
 * \return    x_axis - X axis value.
 */
float mpu6050_get_x_axis_data(void)
{
	if (!device_init_complete_s)
	{
		return 0u;
	}

	uint16_t x_axis = 0u; // resultant value holder used for combining L and H 8 bit values
	uint8_t x_axis_l = read_register(MPU6050_REG_ACCEL_XOUT_L);
	uint8_t x_axis_h = read_register(MPU6050_REG_ACCEL_XOUT_H);
	// combine the 2 with a bit shift and an or operator
	x_axis = ((int16_t)x_axis_h << 8) | x_axis_l;

	return (float)x_axis/16384.0;
}

/*!
 * \brief     Reads and formats the Y Axis data from the MPU6050.
 * \return    y_axis - Y axis value.
 */
float mpu6050_get_y_axis_data(void)
{
	if (!device_init_complete_s)
	{
		return 0u;
	}

	uint16_t y_axis = 0u; // resultant value holder used for combining L and H 8 bit values
	uint8_t y_axis_l = read_register(MPU6050_REG_ACCEL_YOUT_L);
	uint8_t y_axis_h = read_register(MPU6050_REG_ACCEL_YOUT_H);
	// combine the 2 with a bit shift and an or operator
	y_axis = ((int16_t)y_axis_h << 8) | y_axis_l;

	return (float)y_axis/16384.0;
}

/*!
 * \brief     Reads and formats the Z Axis data from the MPU6050.
 * \return    z_axis - Z axis value.
 */
float mpu6050_get_z_axis_data(void)
{
	if (!device_init_complete_s)
	{
		return 0u;
	}

	uint16_t z_axis = 0u; // resultant value holder used for combining L and H 8 bit values
	uint8_t z_axis_l = read_register(MPU6050_REG_ACCEL_ZOUT_L);
	uint8_t z_axis_h = read_register(MPU6050_REG_ACCEL_ZOUT_H);
	// combine the 2 with a bit shift and an or operator
	z_axis = ((int16_t)z_axis_h << 8) | z_axis_l;

	return (float)z_axis/16384.0;
}

float mpu6050_get_temperature_data(void)
{
	if (!device_init_complete_s)
	{
		return 0u;
	}

	short temperature = 0; // resultant value holder used for combining L and H 8 bit values
	uint8_t temp_l = read_register(MPU6050_REG_TEMP_OUT_L);
	uint8_t temp_h = read_register(MPU6050_REG_TEMP_OUT_H);
	// combine the 2 with a bit shift and an or operator
	temperature = (temp_h << 8) | temp_l;

	// convert to Celcius per mpu6050 datasheet
	return (float)temperature/340.0 + 36.53;
}

/****************** PRIVATE FUNCTIONS *******************/

// From MPU6050 documentation:
/*This register is used to verify the identity of the device. The contents of WHO_AM_I are the upper 6 bits of the
 * MPU-60X0’s 7-bit I2C address. The least significant bit of the MPU-60X0’s I2C address is determined by the value
 * of the AD0 pin. The value of the AD0 pin is not reflected in this register. The default value of the register is 0x68.
 */
/*!
 * \brief     Reads the WHO_AM_I register of the MPU6050.
 * \return    device_present - True is the register value was read correctly, false if not.
 */
bool who_am_i(void)
{
	bool device_present = false;
	uint8_t reg_value = 0x00;

	reg_value = read_register(MPU6050_REG_WHO_AM_I);
	if (reg_value == WHO_AM_I_VALUE)
	{
		device_present = true;
	}

	return device_present;
}

/*!
 * \brief     Reads from a register address passed into the function.
 * \param[in] reg_add - Address of the register we want to read.
 * \return    reg_val - Value of the register we read from.
 */
uint8_t read_register(uint8_t reg_add)
{
	uint8_t reg_value = 0x00;
	(void)periph_i2c_rx(MPU6050_ADDRESS, reg_add, &reg_value, 1);
	return reg_value;
}

/*!
 * \brief     Writes a register value to a given register address.
 * \param[in] reg_add - Address of the register we want to write to.
 * \param[in] reg_val - Value of the register we want to write.
 * \return    True is the register write was successful, false if not.
 */
bool write_register(uint8_t reg_add, uint8_t reg_val)
{
	return periph_i2c_tx(MPU6050_ADDRESS, reg_add, &reg_val, 1);
}
