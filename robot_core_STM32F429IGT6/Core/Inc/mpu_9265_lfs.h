/*
 * mpu_9265_lfs.h
 *
 *  Created on: Jan 30, 2020
 *      Author: Luciano Salvatore
 */

#ifndef MPU_9265_LFS_H_
#define MPU_9265_LFS_H_

#include "main.h"
#include "i2c.h"

typedef struct {
	int16_t Accel_X_RAW;
	int16_t Accel_Y_RAW;
	int16_t Accel_Z_RAW;
	int16_t Gyro_X_RAW;
	int16_t Gyro_Y_RAW;
	int16_t Gyro_Z_RAW;
	int16_t Magnet_X_RAW;
	int16_t Magnet_Y_RAW;
	int16_t Magnet_Z_RAW;
} mpuData_t;

void mpu9265_Init(I2C_HandleTypeDef*);
void mpu9265_Read_Accel(mpuData_t*);
void mpu9265_Read_Gyro(mpuData_t*);
void mpu9265_Read_Magnet(mpuData_t*);

#endif /* MPU_9265_LFS_H_ */
