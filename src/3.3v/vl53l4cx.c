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

#include "../../include/3.3v/vl53l4cx.h"
#include "../../lib/vl53l4cd/vl53l4cd_api.h"
#include <stdio.h>

static uint16_t _dev_addr_8bit = 0;

bool vl53l4cx_init(i2c_inst_t *i2c, uint8_t addr) {
    _dev_addr_8bit = (uint16_t)(addr << 1);
    vl53l4cd_set_i2c(i2c);

    uint16_t sensor_id = 0;
    if (VL53L4CD_GetSensorId(_dev_addr_8bit, &sensor_id) != VL53L4CD_ERROR_NONE) {
        printf("[ToF] FAIL: could not read sensor ID\n");
        return false;
    }
    printf("[ToF] Sensor ID: 0x%04X\n", sensor_id);

    if (VL53L4CD_SensorInit(_dev_addr_8bit) != VL53L4CD_ERROR_NONE) {
        printf("[ToF] FAIL: SensorInit failed\n");
        return false;
    }

    // 30ms timing budget, continuous mode
    if (VL53L4CD_SetRangeTiming(_dev_addr_8bit, 30, 0) != VL53L4CD_ERROR_NONE) {
        printf("[ToF] FAIL: SetRangeTiming failed\n");
        return false;
    }

    printf("[ToF] VL53L4CX initialized (VL53L4CD ULD mode, ~1.3m range)\n");
    return true;
}

bool vl53l4cx_start_ranging(void) {
    return VL53L4CD_StartRanging(_dev_addr_8bit) == VL53L4CD_ERROR_NONE;
}

bool vl53l4cx_stop_ranging(void) {
    return VL53L4CD_StopRanging(_dev_addr_8bit) == VL53L4CD_ERROR_NONE;
}

bool vl53l4cx_is_ready(void) {
    uint8_t ready = 0;
    VL53L4CD_CheckForDataReady(_dev_addr_8bit, &ready);
    return ready == 1;
}

uint16_t vl53l4cx_read_distance_mm(void) {
    VL53L4CD_Result_t result = {0};
    VL53L4CD_GetResult(_dev_addr_8bit, &result);
    VL53L4CD_ClearInterrupt(_dev_addr_8bit);

    if (result.range_status != 0) return 0; // invalid measurement

    return result.distance_mm;
}
