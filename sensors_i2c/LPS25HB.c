/*
 * LPS25HB.c
 *
 *  Created on: 1. Nov 2024
 *      Author: vendelin
 */

#include "LPS25HB.h"

uint8_t LPS25HB_address = LPS25HB_DEVICE_ADDRESS_0;
uint16_t LPS25HB_PO;



void LPS25HB_rb(uint8_t reg_addr, uint8_t* values, size_t length) {
    i2c_master_read(values, length, reg_addr, LPS25HB_address, 1);
}

void LPS25HB_wb(uint8_t reg_addr, uint8_t* values, size_t length) {
    i2c_master_write_multi(values, length, reg_addr, LPS25HB_address, 0);
}




void LPS25HB_init() {
    uint8_t whoAmI = 0;
    LPS25HB_rb(LPS25HB_WHO_AM_I_ADDRESS, &whoAmI, 1);

    if (!(whoAmI == LPS25HB_WHO_AM_I_VALUE)) {
        LPS25HB_address = LPS25HB_DEVICE_ADDRESS_1;

        LPS25HB_rb(LPS25HB_WHO_AM_I_ADDRESS, &whoAmI, 1);
        if (whoAmI == LPS25HB_WHO_AM_I_VALUE) {
            uint8_t ctrl1 = 148;
            LPS25HB_wb(LPS25HB_ADDRESS_CTRL1, &ctrl1, 1);
            LPS25HB_pc();
        }
    }
}


void LPS25HB_pc(void){
    uint8_t buffer[2];

    LPS25HB_rb(LPS25HB_ADDRESS_RPDS_L | 0x80, buffer, 2);
    LPS25HB_PO = (int16_t)(buffer[0] | (buffer[1] << 8));
}

void LPS25HB_p(float* pressure) {
    uint8_t buffer[3];

    LPS25HB_rb(LPS25HB_ADDRESS_PRESS_OUT_XL | 0x80, buffer, 3);
    uint32_t p_out = (uint32_t)(buffer[2] << 16) | (buffer[1] << 8) | buffer[0];

    *pressure = p_out / 4096.0f;
}



