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

#include "../../include/3.3v/bmp280.h"

#define atmospheric_pressure 101325.0
#define barometric_scale_height 44330
#define first_pressure_register 0xF7

int32_t t_fine;

void bmp280_read_calibration(i2c_inst_t* i2c, uint8_t addr, bmp280_calibration_data *calibration_struct) {
    uint8_t reg = 0x88;
    i2c_write_blocking(i2c, addr, &reg, 1, true);
    i2c_read_blocking(i2c, addr, (uint8_t*)calibration_struct, sizeof(bmp280_calibration_data), false);
}

void bmp280_configure(i2c_inst_t* i2c, uint8_t addr) {
    uint8_t payload[2];
    payload[0] = REG_CONFIG;
    payload[1] = CMD_CONFIG;
    i2c_write_blocking(i2c, addr, payload, 2, false);
}

void bmp280_wake_up(i2c_inst_t* i2c,uint8_t addr) {
    uint8_t payload[2];
    payload[0] = REG_CTRL_MEAS;
    payload[1] = CMD_WAKE_UP;
    i2c_write_blocking(i2c, addr, payload, 2, false);
}


// Below are Bosch specific code. (BSC START)
// What are these magic numbers?
// ??????????????????????
// apparently a second order taylor series approximation
// i'm just copying this
// source: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp388-ds001.pdf
double bmp280_compensate_temp(int32_t adc_T, bmp280_calibration_data *calibration_struct) {
    double var1 = (((double) adc_T) / 16384.0 - ((double) calibration_struct->dig_T1) / 1024.0) * ((double) calibration_struct->dig_T2);
    double var2 = ((((double) adc_T) / 131072.0 - ((double) calibration_struct->dig_T1) / 8192.0) *
            (((double) adc_T) / 131072.0 - ((double) calibration_struct->dig_T1) / 8192.0)) * ((double) calibration_struct->dig_T3);
    t_fine = (int32_t)(var1 + var2);
    return (var1 + var2) / 5120.0;
}

// this is even more bs
// source: https://www.bosch-sensoartec.com/media/boschsensortec/downloads/datasheets/bst-bmp388-ds001.pdf
double bmp280_compensate_pressure(int32_t adc_P, bmp280_calibration_data *calibration_struct) {
    double var1 = ((double) t_fine / 2.0) - 64000.0;
    double var2 = var1 * var1 * ((double) calibration_struct->dig_P6) / 32768.0;
    var2 = var2 + var1 * ((double) calibration_struct->dig_P5) * 2.0;
    var2 = (var2/4.0) + (((double) calibration_struct->dig_P4) * 65536.0);
    var1 = (((double) calibration_struct->dig_P3) * var1 * var1 / 524288.0 + ((double) calibration_struct->dig_P2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double) calibration_struct->dig_P1);

    if (var1 == 0.0) {
        return 0;
    }

    double p = 1048576.0 - (double) adc_P;
    p = (p - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = ((double) calibration_struct->dig_P9) * p * p / 2147483648.0;
    var2 = p * ((double) calibration_struct->dig_P8) / 32768.0;
    p = p + (var1 + var2 + ((double) calibration_struct->dig_P7)) / 16.0;
    return p;
}


void bmp280_get_raw_measurements(i2c_inst_t *i2c, uint8_t addr, int32_t *adc_T, int32_t *adc_P) {
    uint8_t reg = first_pressure_register;
    uint8_t buffer[6];

    i2c_write_blocking(i2c, addr, &reg, 1, true);

    // read bytes P_MSB, P_LSB, P_XLSB, T_MSB, T_LSB, T_XLSB
    i2c_read_blocking(i2c, addr, buffer, 6, false);

    // heres some REAL bs calculations:
    // take the first byte, shift it left by 12 bits take the second, shift it left by 4
    // Take the third, shift it right by 4. Compare them together with bitwise OR
    *adc_P = (int32_t)((buffer[0] << 12) | (buffer[1] << 4) | (buffer[2] >> 4));

    // reassemble the 20-bit temp number from the remaining 3 bytes
    *adc_T = (int32_t)((buffer[3] << 12) | (buffer[4] << 4) | (buffer[5] >> 4));
}
