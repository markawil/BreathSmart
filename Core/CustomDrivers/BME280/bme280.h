/*
 * bme280.h
 *
 *  Created on: Nov 18, 2024
 *      Author: markwilkinson
 *
 *      based off of Bosch's API:
 *      https://github.com/boschsensortec/BME280_SensorAPI.git
 */

#ifndef CUSTOMDRIVERS_BME280_BME280_H_
#define CUSTOMDRIVERS_BME280_BME280_H_

#include <stdbool.h>
#include <stdint.h>

#define BME280_I2C_ADDRESS1                     0x76
#define BME280_I2C_ADDRESS2                     0x77

/* BME280 chip identifier */
#define BME280_CHIP_ID                          0x60

/* BME280 chip ID address */
#define BME280_CHIP_ID_ADDR						0xD0

// fastest to just read from 0xF7 to 0xFC and use a length to get the values you want
#define BME280_REG_DATA 						0xF7

// soft reset command address
#define BME280_REG_RESET					    0xE0

// lengths for the values you want
#define BME280_LEN_TEMP_PRESS_CALIB_DATA     	UINT8_C(26)
#define BME280_LEN_HUMIDITY_CALIB_DATA          UINT8_C(7)
#define BME280_LEN_P_T_H_DATA                 	UINT8_C(8)

// addresses
#define BME280_REG_TEMP_PRESS_CALIB_DATA		UINT8_C(0x88)
#define BME280_REG_HUMIDITY_CALIB_DATA			UINT8_C(0xE1)

// offsets to get the values
#define BME280_PRESS                            UINT8_C(1)
#define BME280_TEMP                             UINT8_C(1 << 1)
#define BME280_HUM                              UINT8_C(1 << 2)
#define BME280_ALL                              UINT8_C(0x07)

/*! @name Bit shift macros */
#define BME280_12_BIT_SHIFT                       UINT8_C(12)
#define BME280_8_BIT_SHIFT                        UINT8_C(8)
#define BME280_4_BIT_SHIFT                        UINT8_C(4)

/*! @name Macro to combine two 8 bit data's to form a 16 bit data */
#define BME280_CONCAT_BYTES(msb, lsb)           (((uint16_t)msb << 8) | (uint16_t)lsb)

/* Defined data holder structs from Bosch's API example */

/* structs to contain 3 calculated and compensated values */
struct bme280_data
{
    /*! Compensated pressure */
    double pressure;

    /*! Compensated temperature */
    double temperature;

    /*! Compensated humidity */
    double humidity;
};

/* which comprises of uncompensated temperature, pressure and humidity data */
struct bme280_uncomp_data
{
    /*! Un-compensated pressure */
    uint32_t pressure;

    /*! Un-compensated temperature */
    uint32_t temperature;

    /*! Un-compensated humidity */
    uint32_t humidity;
};

/* data holder for settings chosen and sent to device */
struct bme280_settings
{
    /*! Pressure oversampling */
    uint8_t osr_p;

    /*! Temperature oversampling */
    uint8_t osr_t;

    /*! Humidity oversampling */
    uint8_t osr_h;

    /*! Filter coefficient */
    uint8_t filter;

    /*! Standby time */
    uint8_t standby_time;
};

/* data holder for calibration data */
struct bme280_calib_data
{
    /*! Calibration coefficient for the temperature sensor */
    uint16_t dig_t1;

    /*! Calibration coefficient for the temperature sensor */
    int16_t dig_t2;

    /*! Calibration coefficient for the temperature sensor */
    int16_t dig_t3;

    /*! Calibration coefficient for the pressure sensor */
    uint16_t dig_p1;

    /*! Calibration coefficient for the pressure sensor */
    int16_t dig_p2;

    /*! Calibration coefficient for the pressure sensor */
    int16_t dig_p3;

    /*! Calibration coefficient for the pressure sensor */
    int16_t dig_p4;

    /*! Calibration coefficient for the pressure sensor */
    int16_t dig_p5;

    /*! Calibration coefficient for the pressure sensor */
    int16_t dig_p6;

    /*! Calibration coefficient for the pressure sensor */
    int16_t dig_p7;

    /*! Calibration coefficient for the pressure sensor */
    int16_t dig_p8;

    /*! Calibration coefficient for the pressure sensor */
    int16_t dig_p9;

    /*! Calibration coefficient for the humidity sensor */
    uint8_t dig_h1;

    /*! Calibration coefficient for the humidity sensor */
    int16_t dig_h2;

    /*! Calibration coefficient for the humidity sensor */
    uint8_t dig_h3;

    /*! Calibration coefficient for the humidity sensor */
    int16_t dig_h4;

    /*! Calibration coefficient for the humidity sensor */
    int16_t dig_h5;

    /*! Calibration coefficient for the humidity sensor */
    int8_t dig_h6;

    /*! Variable to store the intermediate temperature coefficient */
    int32_t t_fine;
};

/* bme280 device structure data holder */
struct bme280_device
{
    /*! Chip Id */
    uint8_t chip_id;

    /*! Trim data */
    struct bme280_calib_data calib_data;
};


/* public methods */

/* Initializes the device over i2c, true if succeeds */
bool bme280_init(void);

/* soft resets the device, true if succeeds */
bool bme280_soft_reset(void);

/* gives back compensated and calibrated values for all 3 */
void bme280_get_temp_pressure_humidity(struct bme280_data *data);


#endif /* CUSTOMDRIVERS_BME280_BME280_H_ */
