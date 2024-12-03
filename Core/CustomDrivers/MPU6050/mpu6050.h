/*
 * mpu6050.h
 *
 *  Created on: Oct 11, 2024
 *      Author: markwilkinson
 */

#ifndef CUSTOMDRIVERS_MPU6050_MPU6050_H_
#define CUSTOMDRIVERS_MPU6050_MPU6050_H_

#include <stdint.h>
#include <stdbool.h>

#define MPU6050_DEV_ADD_LOW  0xD0u // Device address of the MPU6050 when pin AD0 is pulled low
#define MPU6050_DEV_ADD_HIGH 0xE0u // Device address of the MPU6050 when pin AD0 is pulled high

bool mpu6050_init(void);

/*** Gyro/Accel measurements ***/
float mpu6050_get_x_axis_data(void);
float mpu6050_get_y_axis_data(void);
float mpu6050_get_z_axis_data(void);

/*** Temperature measurements ***/
float mpu6050_get_temperature_data(void);

#endif /* CUSTOMDRIVERS_MPU6050_H_ */
