/*
 * HTS221.c
 *
 *  Created on: 1. Nov 2024
 *      Author: vendelin
 */

#include "HTS221.h"

uint8_t HTS221_address = HTS221_DEVICE_ADDRESS_0;
float HTS221_TS;
float HTS221_TZ;
float HTS221_HS;
float HTS221_HZ;

// Write multiple bytes
void LPS25HB_wb(uint8_t reg_add, uint8_t* val, size_t len) {
    i2c_master_write_multi(val, len, reg_add, HTS221_address, 0);
}

// Read multiple bytes
void LPS25HB_rb(uint8_t reg_add, uint8_t* val, size_t len) {
    i2c_master_read(val, len, reg_add, HTS221_address, 1);
}


void HTS221_init(void) {
    uint8_t whoAmI;

    LPS25HB_rb(HTS221_WHO_AM_I_ADDRESS, &whoAmI, 1);

    if (!(whoAmI == HTS221_WHO_AM_I_VALUE)) {
    	HTS221_address = HTS221_DEVICE_ADDRESS_1;
    	LPS25HB_rb(HTS221_WHO_AM_I_ADDRESS, &whoAmI, 1);
    }

    uint8_t ctrl_s = 134;
    LPS25HB_wb(HTS221_CTRL1, &ctrl_s, 1);

    HTS221_get_tc();
    HTS221_get_hc();
}


void HTS221_hum(float* humidity_out) {
    uint8_t h_out_data[2];
    int16_t h_out;

    LPS25HB_rb(HTS221_HUMIDITY_OUT_L, h_out_data, 2);
    h_out = h_out_data[0] | (h_out_data[1] << 8);

    *humidity_out = (h_out * HTS221_HS + HTS221_HZ);
}


void HTS221_temp(float* temperature_out) {
    uint8_t t_out_buffer[2];
    int16_t t_out;


    LPS25HB_rb(HTS221_TEMP_OUT_L | 0x80, t_out_buffer, 2);


    t_out = (int16_t)((uint16_t)t_out_buffer[1] << 8 | t_out_buffer[0]);


    *temperature_out = (t_out * -HTS221_TS) + HTS221_TZ;
}


void HTS221_get_hc(void) {
    uint8_t cd[2];
    uint8_t h0T0Out_data[2], h1T0Out_data[2];
    int16_t h0_t0_out, h1_t0_out;


    LPS25HB_rb(HTS221_H0_rH_x2 | 0x80, cd, 2);


    uint8_t h0_rH = cd[0] >> 1;
    uint8_t h1_rH = cd[1] >> 1;


    LPS25HB_rb(HTS221_H0_T0_OUT_H | 0x80, h0T0Out_data, 2);
    LPS25HB_rb(HTS221_H0_T0_OUT_L | 0x80, h1T0Out_data, 2);


    h0_t0_out = (int16_t)((uint16_t)h0T0Out_data[1] << 8 | (uint16_t)h0T0Out_data[0]);
    h1_t0_out = (int16_t)((uint16_t)h1T0Out_data[1] << 8 | (uint16_t)h1T0Out_data[0]);


    HTS221_HS = (float)(h1_rH - h0_rH) / (h1_t0_out - h0_t0_out);
    HTS221_HZ = h0_rH - HTS221_HS * h0_t0_out;
}

void HTS221_get_tc(void) {
    uint8_t cd[4];
    uint8_t t_out_data[4];
    int16_t t0_out, t1_out;
    uint16_t t0_degC, t1_degC;


    LPS25HB_rb(HTS221_T0_degC_x8 | 0x80, cd, 4);


    t0_degC = ((uint16_t)(cd[0]) | ((uint16_t)(cd[2] & 0x03) << 8));
    t1_degC = ((uint16_t)(cd[1]) | ((uint16_t)(cd[2] & 0x0C) << 8));


    LPS25HB_rb(HTS221_T0_OUT_L | 0x80, t_out_data, 4);


    t0_out = (int16_t)(t_out_data[1] << 8 | t_out_data[0]);
    t1_out = (int16_t)(t_out_data[3] << 8 | t_out_data[2]);


    HTS221_TS = (t1_degC - t0_degC) / (8.0 * t1_out - t0_out);
    HTS221_TZ = (t0_degC / 8.0) - HTS221_TS * t0_out;
    LL_mDelay(100);
}


