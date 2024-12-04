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

/* Private variables */
static struct bme280_device bme280;
static bool device_init_complete_s = false;

/* Private methods */
bool get_calib_data();
void parse_temp_press_calib_data(const uint8_t *reg_data);
void parse_humidity_calib_data(const uint8_t *reg_data);
void parse_sensor_data(const uint8_t *reg_data, struct bme280_uncomp_data *uncomp_data);
bool bme280_compensate_data(const struct bme280_uncomp_data *uncomp_data,
                            	  struct bme280_data *comp_data,
                                  struct bme280_calib_data *calib_data);
static double compensate_temperature(const struct bme280_uncomp_data *uncomp_data,
									       struct bme280_calib_data *calib_data);
static double compensate_pressure(const struct bme280_uncomp_data *uncomp_data,
                                  const struct bme280_calib_data *calib_data);
static double compensate_humidity(const struct bme280_uncomp_data *uncomp_data,
                                  const struct bme280_calib_data *calib_data);


bool bme280_init(void)
{
	bool success = false;
	uint8_t chip_id = 0;

	 /* Read the chip-id of bme280 sensor */
	success = periph_i2c_rx(BME280_I2C_ADDRESS1, BME280_CHIP_ID_ADDR, &chip_id, 1);

	if (!success || chip_id != BME280_CHIP_ID)
	{
		return false;
	}

	success = bme280_soft_reset();

	if (success)
	{
		/* Read the calibration data */
		success = get_calib_data();
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
	if (data == NULL)
	{
		// TODO: need a way to propogate errors
		return;
	}

	bool success = false;
	/* Array to store the pressure, temperature and humidity data read from
	 * the sensor
	 */
	uint8_t reg_data[BME280_LEN_P_T_H_DATA] = { 0 };
	struct bme280_uncomp_data uncomp_data = { 0 };

	success = periph_i2c_rx(BME280_I2C_ADDRESS1, BME280_REG_DATA, reg_data, BME280_LEN_TEMP_PRESS_CALIB_DATA);

	if (success)
	{
		/* Parse the read data from the sensor */
		parse_sensor_data(reg_data, &uncomp_data);

		/* Compensate the pressure and/or temperature and/or
		 * humidity data from the sensor
		 */
		(void)bme280_compensate_data(&uncomp_data, data, &bme280.calib_data);
	}
}

bool get_calib_data()
{
	// TODO: need a way to propogate errors
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
    	success = false; // reset the flag
        parse_temp_press_calib_data(calib_data);

        /* Read the humidity calibration data from the sensor */
        success = periph_i2c_rx(BME280_I2C_ADDRESS1, reg_addr, calib_data, BME280_LEN_HUMIDITY_CALIB_DATA);

        if (success)
        {
            /* Parse humidity calibration data and store it in
             * device structure
             */
            parse_humidity_calib_data(calib_data);
        }
    }

    return success;
}

void parse_temp_press_calib_data(const uint8_t *reg_data)
{
	struct bme280_calib_data *calib_data = &bme280.calib_data;

	calib_data->dig_t1 = BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
	calib_data->dig_t2 = (int16_t)BME280_CONCAT_BYTES(reg_data[3], reg_data[2]);
	calib_data->dig_t3 = (int16_t)BME280_CONCAT_BYTES(reg_data[5], reg_data[4]);
	calib_data->dig_p1 = BME280_CONCAT_BYTES(reg_data[7], reg_data[6]);
	calib_data->dig_p2 = (int16_t)BME280_CONCAT_BYTES(reg_data[9], reg_data[8]);
	calib_data->dig_p3 = (int16_t)BME280_CONCAT_BYTES(reg_data[11], reg_data[10]);
	calib_data->dig_p4 = (int16_t)BME280_CONCAT_BYTES(reg_data[13], reg_data[12]);
	calib_data->dig_p5 = (int16_t)BME280_CONCAT_BYTES(reg_data[15], reg_data[14]);
	calib_data->dig_p6 = (int16_t)BME280_CONCAT_BYTES(reg_data[17], reg_data[16]);
	calib_data->dig_p7 = (int16_t)BME280_CONCAT_BYTES(reg_data[19], reg_data[18]);
	calib_data->dig_p8 = (int16_t)BME280_CONCAT_BYTES(reg_data[21], reg_data[20]);
	calib_data->dig_p9 = (int16_t)BME280_CONCAT_BYTES(reg_data[23], reg_data[22]);
	calib_data->dig_h1 = reg_data[25];
}

void parse_humidity_calib_data(const uint8_t *reg_data)
{
	struct bme280_calib_data *calib_data = &bme280.calib_data;
	int16_t dig_h4_lsb;
	int16_t dig_h4_msb;
	int16_t dig_h5_lsb;
	int16_t dig_h5_msb;

	calib_data->dig_h2 = (int16_t)BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
	calib_data->dig_h3 = reg_data[2];
	dig_h4_msb = (int16_t)(int8_t)reg_data[3] * 16;
	dig_h4_lsb = (int16_t)(reg_data[4] & 0x0F);
	calib_data->dig_h4 = dig_h4_msb | dig_h4_lsb;
	dig_h5_msb = (int16_t)(int8_t)reg_data[5] * 16;
	dig_h5_lsb = (int16_t)(reg_data[4] >> 4);
	calib_data->dig_h5 = dig_h5_msb | dig_h5_lsb;
	calib_data->dig_h6 = (int8_t)reg_data[6];
}

void parse_sensor_data(const uint8_t *reg_data, struct bme280_uncomp_data *uncomp_data)
{
    /* Variables to store the sensor data */
    uint32_t data_xlsb;
    uint32_t data_lsb;
    uint32_t data_msb;

    /* Store the parsed register values for pressure data */
    data_msb = (uint32_t)reg_data[0] << BME280_12_BIT_SHIFT;
    data_lsb = (uint32_t)reg_data[1] << BME280_4_BIT_SHIFT;
    data_xlsb = (uint32_t)reg_data[2] >> BME280_4_BIT_SHIFT;
    uncomp_data->pressure = data_msb | data_lsb | data_xlsb;

    /* Store the parsed register values for temperature data */
    data_msb = (uint32_t)reg_data[3] << BME280_12_BIT_SHIFT;
    data_lsb = (uint32_t)reg_data[4] << BME280_4_BIT_SHIFT;
    data_xlsb = (uint32_t)reg_data[5] >> BME280_4_BIT_SHIFT;
    uncomp_data->temperature = data_msb | data_lsb | data_xlsb;

    /* Store the parsed register values for humidity data */
    data_msb = (uint32_t)reg_data[6] << BME280_8_BIT_SHIFT;
    data_lsb = (uint32_t)reg_data[7];
    uncomp_data->humidity = data_msb | data_lsb;
}

bool bme280_compensate_data(const struct bme280_uncomp_data *uncomp_data,
                              	  struct bme280_data *comp_data,
								  struct bme280_calib_data *calib_data)
{
	if ((uncomp_data == NULL) || (comp_data == NULL) || (calib_data == NULL))
	{
		return false;
	}

	/* Initialize to zero */
	comp_data->temperature = 0;
	comp_data->pressure = 0;
	comp_data->humidity = 0;

	/* Compensate the all 3 readings */
	comp_data->temperature = compensate_temperature(uncomp_data, calib_data);
	comp_data->pressure = compensate_pressure(uncomp_data, calib_data);
	comp_data->humidity = compensate_humidity(uncomp_data, calib_data);

    return true;
}

static double compensate_temperature(const struct bme280_uncomp_data *uncomp_data,
									       struct bme280_calib_data *calib_data)
{
    double var1;
    double var2;
    double temperature;
    double temperature_min = -40;
    double temperature_max = 85;

    var1 = (((double)uncomp_data->temperature) / 16384.0 - ((double)calib_data->dig_t1) / 1024.0);
    var1 = var1 * ((double)calib_data->dig_t2);
    var2 = (((double)uncomp_data->temperature) / 131072.0 - ((double)calib_data->dig_t1) / 8192.0);
    var2 = (var2 * var2) * ((double)calib_data->dig_t3);
    calib_data->t_fine = (int32_t)(var1 + var2);
    temperature = (var1 + var2) / 5120.0;

    if (temperature < temperature_min)
    {
        temperature = temperature_min;
    }
    else if (temperature > temperature_max)
    {
        temperature = temperature_max;
    }

    return temperature;
}

static double compensate_pressure(const struct bme280_uncomp_data *uncomp_data,
                                  const struct bme280_calib_data *calib_data)
{
    double var1;
    double var2;
    double var3;
    double pressure;
    double pressure_min = 30000.0;
    double pressure_max = 110000.0;

    var1 = ((double)calib_data->t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double)calib_data->dig_p6) / 32768.0;
    var2 = var2 + var1 * ((double)calib_data->dig_p5) * 2.0;
    var2 = (var2 / 4.0) + (((double)calib_data->dig_p4) * 65536.0);
    var3 = ((double)calib_data->dig_p3) * var1 * var1 / 524288.0;
    var1 = (var3 + ((double)calib_data->dig_p2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double)calib_data->dig_p1);

    /* Avoid exception caused by division by zero */
    if (var1 > (0.0))
    {
        pressure = 1048576.0 - (double) uncomp_data->pressure;
        pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
        var1 = ((double)calib_data->dig_p9) * pressure * pressure / 2147483648.0;
        var2 = pressure * ((double)calib_data->dig_p8) / 32768.0;
        pressure = pressure + (var1 + var2 + ((double)calib_data->dig_p7)) / 16.0;

        if (pressure < pressure_min)
        {
            pressure = pressure_min;
        }
        else if (pressure > pressure_max)
        {
            pressure = pressure_max;
        }
    }
    else /* Invalid case */
    {
        pressure = pressure_min;
    }

    return pressure;
}

static double compensate_humidity(const struct bme280_uncomp_data *uncomp_data,
                                  const struct bme280_calib_data *calib_data)
{
    double humidity;
    double humidity_min = 0.0;
    double humidity_max = 100.0;
    double var1;
    double var2;
    double var3;
    double var4;
    double var5;
    double var6;

    var1 = ((double)calib_data->t_fine) - 76800.0;
    var2 = (((double)calib_data->dig_h4) * 64.0 + (((double)calib_data->dig_h5) / 16384.0) * var1);
    var3 = uncomp_data->humidity - var2;
    var4 = ((double)calib_data->dig_h2) / 65536.0;
    var5 = (1.0 + (((double)calib_data->dig_h3) / 67108864.0) * var1);
    var6 = 1.0 + (((double)calib_data->dig_h6) / 67108864.0) * var1 * var5;
    var6 = var3 * var4 * (var5 * var6);
    humidity = var6 * (1.0 - ((double)calib_data->dig_h1) * var6 / 524288.0);

    if (humidity > humidity_max)
    {
        humidity = humidity_max;
    }
    else if (humidity < humidity_min)
    {
        humidity = humidity_min;
    }

    return humidity;
}
