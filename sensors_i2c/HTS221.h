/*
 * HTS221.h
 *
 *  Created on: 1. Nov 2024
 *      Author: vendelin
 */

#include "main.h"
#include "i2c.h"

#ifndef HTS221_H_
#define HTS221_H_


#define 	HTS221_DEVICE_ADDRESS_0		0xBE
#define 	HTS221_DEVICE_ADDRESS_1		0xBF

#define 	HTS221_WHO_AM_I_VALUE		0xBC
#define 	HTS221_WHO_AM_I_ADDRESS		0x0F

#define 	HTS221_CTRL1				0x20

#define 	HTS221_HUMIDITY_OUT_L 		0x28

#define 	HTS221_TEMP_OUT_L			0x2A

#define 	HTS221_H0_T0_OUT_L			0x36
#define		HTS221_H0_T0_OUT_H			0x37

#define 	HTS221_T0_degC_x8			0x32
#define 	HTS221_H0_rH_x2				0x30
#define 	HTS221_T0_OUT_L				0x3C
#define		HTS221_T0_OUT_H				0x3D

void HTS221_rb(uint8_t reg_addr, uint8_t* values, size_t length);
void HTS221_wb(uint8_t reg_addr, uint8_t value);
void HTS221_init(void);

void HTS221_get_hc(void);
void HTS221_hum(float* hum);

void HTS221_get_tc(void);
void HTS221_temp(float* temp);

#endif /* HTS221_H_ */
