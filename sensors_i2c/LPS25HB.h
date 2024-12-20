/*
 * milujem Lucku
 *
 * LPS25HB.h
 *
 *  Created on: 1. Nov 2024
 *      Author: vendelin
 */
#include "main.h"

#ifndef LPS25HB_H_
#define LPS25HB_H_

#define 	LPS25HB_DEVICE_ADDRESS_0		0xB8
#define 	LPS25HB_DEVICE_ADDRESS_1		0xBA

#define 	LPS25HB_WHO_AM_I_VALUE			0xBD
#define 	LPS25HB_WHO_AM_I_ADDRESS		0x0F

#define 	LPS25HB_ADDRESS_CTRL1			0x20

#define 	LPS25HB_PRESSURE_OUT			0x28

#define		LPS25HB_ADDRESS_RPDS_L			0x39

#define 	LPS25HB_PRESSURE_OUT_L		0x29

void LPS25HB_rb(uint8_t reg_addr);
void LPS25HB_wb(uint8_t reg_addr, uint8_t value);
void LPS25HB_init(void);

void LPS25HB_pc(void);
void LPS25HB_press(float* press);
#endif /* LPS25HB_H_ */
