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

#include <math.h>
#include "../include/mpu6050.h"

bool mpu6050_init(i2c_inst_t *i2c, uint8_t addr) {
    uint8_t reg = MPU6050_REG_WHO_AM_I;
    uint8_t who_am_i;

    int ret = i2c_write_blocking(i2c, addr, &reg, 1, true);
    if (ret < 0) return false;

    ret = i2c_read_blocking(i2c, addr, &who_am_i, 1, false);
    if (ret < 0) return false;

    if (who_am_i != MPU6050_WHO_AM_I_VAL) return false;

    // wake from sleep — chip powers on in sleep mode for some reason
    uint8_t payload[2] = { MPU6050_REG_PWR_MGMT_1, 0x00 };
    ret = i2c_write_blocking(i2c, addr, payload, 2, false);
    if (ret < 0) return false;

    return true;
}

bool mpu6050_read(i2c_inst_t *i2c, uint8_t addr, mpu6050_data *data) {
    uint8_t reg = MPU6050_REG_ACCEL_XOUT_H;
    uint8_t buffer[14];

    int ret = i2c_write_blocking(i2c, addr, &reg, 1, true);
    if (ret < 0) return false;

    // 14 bytes: accel XYZ (6), temp (2, ignored), gyro XYZ (6)
    ret = i2c_read_blocking(i2c, addr, buffer, 14, false);
    if (ret < 14) return false;

    int16_t raw_ax = (int16_t)((buffer[0] << 8) | buffer[1]);
    int16_t raw_ay = (int16_t)((buffer[2] << 8) | buffer[3]);
    int16_t raw_az = (int16_t)((buffer[4] << 8) | buffer[5]);
    int16_t raw_gx = (int16_t)((buffer[8] << 8) | buffer[9]);
    int16_t raw_gy = (int16_t)((buffer[10] << 8) | buffer[11]);
    int16_t raw_gz = (int16_t)((buffer[12] << 8) | buffer[13]);

    data->accel_x = raw_ax / MPU6050_ACCEL_SENSITIVITY;
    data->accel_y = raw_ay / MPU6050_ACCEL_SENSITIVITY;
    data->accel_z = raw_az / MPU6050_ACCEL_SENSITIVITY;

    data->gyro_x = raw_gx / MPU6050_GYRO_SENSITIVITY;
    data->gyro_y = raw_gy / MPU6050_GYRO_SENSITIVITY;
    data->gyro_z = raw_gz / MPU6050_GYRO_SENSITIVITY;

    return true;
}

bool mpu6050_detect_freefall(mpu6050_data *data, double threshold_g) {
    double magnitude = sqrt(
        data->accel_x * data->accel_x +
        data->accel_y * data->accel_y +
        data->accel_z * data->accel_z
    );
    return magnitude < threshold_g;
}
