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

#include "../include/bmp280.h"


static inline void bmp280_read_calib(i2c_inst_t* i2c, uint8_t addr, bmp280_calibration_data *calibration_struct) {
    uint8_t reg = 0x88;
    i2c_write_blocking(i2c, addr, &reg, 1, true);
    i2c_read_blocking(i2c, addr, (uint8_t*)calibration_struct, sizeof(bmp280_calibration_data), false);
}

void bmp280_wake_up(i2c_inst_t* i2c,uint8_t addr) {
    uint8_t payload[2];
    payload[0] = REG_CTRL_MEAS;
    payload[1] = CMD_WAKE_UP;
    i2c_write_blocking(i2c, addr, payload, 2, false);
}
