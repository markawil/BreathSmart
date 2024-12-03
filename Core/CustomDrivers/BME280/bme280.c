/*
 * bme280.c
 *
 *  Created on: Nov 18, 2024
 *      Author: markwilkinson
 *
 *      based off of Bosch's API:
 *      https://github.com/boschsensortec/BME280_SensorAPI.git
 */

#include "bme280.h"
#include "periph_i2c.h"


bool bme280_init(void)
{
	bool success = false;
	uint8_t chip_id = 0;

	 /* Read the chip-id of bme280 sensor */
	success = periph_i2c_rx(BME280_I2C_ADDRESS1, BME280_CHIP_ID_ADDR, &chip_id, 1);

	if (success)
	{
		success = false; // reset flag
		if (chip_id == BME280_CHIP_ID)
		{
			/* Reset the sensor */
			success = bme280_soft_reset();

			if (success)
			{
				/* Read the calibration data */
			//	result = get_calib_data(dev);
			}
		}
	}

	return success;
}

bool bme280_soft_reset()
{
	uint8_t buffer = 0;
	bool result = periph_i2c_tx(BME280_I2C_ADDRESS1, BME280_REG_RESET, &buffer, 1);
	HAL_Delay(20);
	return result;
}

void bme280_get_temp_pressure_humidity(struct bme280_data *data)
{

}

bool get_calib_data(struct bme280_dev *dev)
{
    bool success = false;
    uint8_t reg_addr = BME280_REG_TEMP_PRESS_CALIB_DATA;

    /* Array to store calibration data */
    uint8_t calib_data[BME280_LEN_TEMP_PRESS_CALIB_DATA] = { 0 };

    /* Read the calibration data from the sensor */
    success = periph_i2c_rx(BME280_I2C_ADDRESS1, reg_addr, calib_data, BME280_LEN_TEMP_PRESS_CALIB_DATA);

    if (success)
    {
        /* Parse temperature and pressure calibration data and store
         * it in device structure
         */
        parse_temp_press_calib_data(calib_data, dev);
        reg_addr = BME280_REG_HUMIDITY_CALIB_DATA;

        /* Read the humidity calibration data from the sensor */
        rslt = bme280_get_regs(reg_addr, calib_data, BME280_LEN_HUMIDITY_CALIB_DATA, dev);

        if (rslt == BME280_OK)
        {
            /* Parse humidity calibration data and store it in
             * device structure
             */
            parse_humidity_calib_data(calib_data, dev);
        }
    }

    return success;
}
