/*
 * ccs811.h
 *
 *  Created on: Oct 30, 2024
 *      Author: markwilkinson
 *
 *      From DFRobot's documentation:
 *
 *      CCS811 Air Quality Sensor can measure the eCO2 (equivalent CO2) and TVOC (Total Volatile Organic Compounds) density. It can be widely used
		in many applications, such as air quality detection, air purifiers, ventilation system and so on. This CCS811 air quality sensor uses AMS's unique
		micro-hot plate technology. Compared to conventional gas sensors, this sensor is lower power consumption, heating faster, and smaller.
		Internally integrated ADCs and MCUs allow data to be collected, calculated, and returned via IIC. The CCS811 supports a concentration alarm,
		which is triggered when the concentration exceeds the user-set threshold.

		The CCS811 air quality sensor supports multiple modes, such as detect in every second, in every 10s, in every one minute, in every 250ms and
		sleep mode. These modes are optimized for low power consumption during the sensor measurement, so the CCS811 is also suitable for
		portable applications.
 */

#ifndef CUSTOMDRIVERS_CCS811_CCS811_H_
#define CUSTOMDRIVERS_CCS811_CCS811_H_

#include <stdint.h>
#include <stdbool.h>

#define CCS811_DEV_ADD = 0x5A // device address in default mode

/*
 *  Initialize the CCS811, returns false if something in init failed.
 *
 */
bool ccs811_init(void);

/*
 * Resets the sensor, clears all configured data.
 *
 */
void ccs811_reset(void);

/*
 * Checks if the sensor is ready to take a reading.
 *
 */
bool ccs811_check_data_ready(void);

/*
 * CO2 concentration(ppm)	Impact on human-beings
 * 	    < 500					normal
 *		500-1000                uncomfortable
 *	    1000-2500 				tired
 *	    2500-5000 			    unhealthy
 */
uint16_t ccs811_get_CO2_PPM(void);

/*
 * TVOC Concentration(PPB)   Effect on Human
 * 		< 50 					 Normal
 * 		50 - 750 				 Anxious,uncomfortable
 * 		750 - 6000 				 depressive, headache
 * 		> 6000 					 headaches, nerve problems
 */
uint16_t ccs811_get_TVOC_PPB(void);

/*
 * Reads the current baseline value
 * The baseline is considered the clean air point to base the
 * pollution air values off of.
 */
uint16_t readBaseLine(void);

 /*
  * Write back the saved clean air baseline to the device when
  * starting back up.
  */
void write_baseline(uint16_t baseline);

#endif /* CUSTOMDRIVERS_CCS811_CCS811_H_ */
