/*
 * MIT License
 *
 * Copyright (c) 2026 orangejuiceplz
 * CREATED on 5/19/26 
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


#ifndef AP_PHYS_1_EOY_BMP280_H
#define AP_PHYS_1_EOY_BMP280_H

#include <stdint.h>
#include "hardware/i2c.h"
#include <math.h>

#define REG_CTRL_MEAS 0xF4
#define CMD_WAKE_UP   0x27

int32_t t_fine; // temp

// this friggen thing SUCKS bosch
// apparently this is how they store 24 bytes of calibration
// and a mix of signed and unsigned ints

// source: https://www.instructables.com/Library-for-BMP280/
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} bmp280_calibration_data;

/**
 * read 24 bytes of calibration data from the BMP280 starting at register 0x88.
 * does this even work?
 *
 * @param i2c i2c instance
 * @param addr I2C address of the BMP280 (0x77)
 * @param calibration_struct pointer to the calibration struct
 */
static inline void bmp280_read_calib(i2c_inst_t *i2c, uint8_t addr, bmp280_calibration_data *calibration_struct);

/**
 *
 * writes a single byte to ctrl_meas (0xF4)
 *
 * @param i2c i2c instance
 * @param addr I2c address of the BMP280 (0x77)
 */
void bmp280_wake_up(i2c_inst_t *i2c, uint8_t addr);

/**
 *
 * Calculate true temperature
 *
 * @param adc_t analog to digital temperature
 * @param calibration_struct pointer to the calibration struct
 *
 * @return true temperature in Celsius
 */
double bmp280_compensate_temp(int32_t adc_t, bmp280_calibration_data *calibration_struct);

/**
 *
 * Calculate true pressure (pascals)
 *
 * @param adc_P analog to digital pressure
 * @param calibration_struct pointer to the calibration struct
 *
 * @return true pressure in Pascals
 */
double bmp280_compensate_pressure(int32_t adc_P, bmp280_calibration_data *calibration_struct);

/**
 *
 * calculate altitude via nonsensical stuff
 *
 * @param pressure current pressure
 * @return current altitude
 */
double calculate_altitude(double pressure);

/**
 *
 * get raw measurements from the BMP
 *
 * @param i2c i2c instance
 * @param addr i2c address
 * @param adc_T analog to digital temperature
 * @param adc_P analog to digital pressure
 */
void bmp280_get_raw_measurements(i2c_inst_t *i2c, uint8_t addr, int32_t *adc_T, int32_t *adc_P);


#endif