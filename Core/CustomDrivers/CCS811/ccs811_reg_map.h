/*
 * ccs811_reg_map.h
 *
 *  Created on: Nov 6, 2024
 *      Author: markwilkinson
 *
 *   Addresses and conversion from Arduino library
 *   courtesy of https://github.com/DFRobot/DFRobot_CCS811
 */

#ifndef CUSTOMDRIVERS_CCS811_CCS811_REG_MAP_H_
#define CUSTOMDRIVERS_CCS811_CCS811_REG_MAP_H_

/*I2C ADDRESSES*/
#define CCS811_I2C_ADDRESS1                      0x5A
#define CCS811_I2C_ADDRESS2                      0x5B

#define CCS811_REG_STATUS                        0x00
#define CCS811_REG_MEAS_MODE                     0x01
#define CCS811_REG_ALG_RESULT_DATA               0x02
#define CCS811_REG_RAW_DATA                      0x03
#define CCS811_REG_ENV_DATA                      0x05
#define CCS811_REG_NTC                           0x06
#define CCS811_REG_THRESHOLDS                    0x10
#define CCS811_REG_BASELINE                      0x11
#define CCS811_REG_HW_ID                         0x20
#define CCS811_REG_HW_VERSION                    0x21
#define CCS811_REG_FW_BOOT_VERSION               0x23
#define CCS811_REG_FW_APP_VERSION                0x24
#define CCS811_REG_INTERNAL_STATE                0xA0
#define CCS811_REG_ERROR_ID                      0xE0
#define CCS811_REG_SW_RESET                      0xFF

#define CCS811_BOOTLOADER_APP_ERASE              0xF1
#define CCS811_BOOTLOADER_APP_DATA               0xF2
#define CCS811_BOOTLOADER_APP_VERIFY             0xF3
#define CCS811_BOOTLOADER_APP_START              0xF4

#define CCS811_HW_ID                             0x81

/* From datasheet:
 *
 * The CCS811 has 5 modes of operation as follows
	• Mode 0: Idle, low current mode
	• Mode 1: Constant power mode, IAQ measurement every
	second
	• Mode 2: Pulse heating mode IAQ measurement every 10
	seconds
	• Mode 3: Low power pulse heating mode IAQ
	measurement every 60 seconds
	• Mode 4: Constant power mode, sensor measurement
	every 250ms
 */

typedef enum {
	CCS811_MODE_IDLE,
	CCS811_MODE_1SEC,
	CCS811_MODE_10SEC,
	CCS811_MODE_60SEC,
	CCS811_MODE_250ms,
}CCS811_MODE;

#endif /* CUSTOMDRIVERS_CCS811_CCS811_REG_MAP_H_ */
