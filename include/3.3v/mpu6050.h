/*
 * MIT License
 *
 * Copyright (c) 2026 orangejuiceplz
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

#ifndef AP_PHYS_1_EOY_MPU6050_H
#define AP_PHYS_1_EOY_MPU6050_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/i2c.h"

#define MPU6050_ADDR 0x68

#define MPU6050_REG_WHO_AM_I     0x75
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_GYRO_XOUT_H  0x43

#define MPU6050_WHO_AM_I_VAL 0x68

#define MPU6050_ACCEL_SENSITIVITY 16384.0 // LSB/g at ±2g
#define MPU6050_GYRO_SENSITIVITY  131.0   // LSB/(°/s) at ±250°/s

typedef struct {
    double accel_x, accel_y, accel_z;
    double gyro_x, gyro_y, gyro_z;
} mpu6050_data;

bool mpu6050_init(i2c_inst_t *i2c, uint8_t addr);
bool mpu6050_read(i2c_inst_t *i2c, uint8_t addr, mpu6050_data *data);

// |a| = sqrt(ax² + ay² + az²) < threshold → freefall
bool mpu6050_detect_freefall(mpu6050_data *data, double threshold_g);

#endif
