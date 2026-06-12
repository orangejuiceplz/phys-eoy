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
#include "../../lib/vl53l4cx/core/vl53lx_api.h"
#include <stdio.h>

static VL53LX_Dev_t _dev;

bool vl53l4cx_init(i2c_inst_t *i2c, uint8_t addr) {
    _dev.i2c = i2c;
    _dev.i2c_addr = addr;

    if (VL53LX_WaitDeviceBooted(&_dev) != VL53LX_ERROR_NONE) {
        printf("[ToF] FAIL: device boot timeout\n");
        return false;
    }

    if (VL53LX_DataInit(&_dev) != VL53LX_ERROR_NONE) {
        printf("[ToF] FAIL: DataInit failed\n");
        return false;
    }

    // 6m
    if (VL53LX_SetDistanceMode(&_dev, VL53LX_DISTANCEMODE_LONG) != VL53LX_ERROR_NONE) {
        printf("[ToF] FAIL: SetDistanceMode failed\n");
        return false;
    }

    // 33ms budget = one fresh range every ~2 flight loops
    if (VL53LX_SetMeasurementTimingBudgetMicroSeconds(&_dev, 33000) != VL53LX_ERROR_NONE) {
        printf("[ToF] FAIL: SetTimingBudget failed\n");
        return false;
    }

    printf("[ToF] VL53L4CX initialized (full API, LONG mode, ~6m range)\n");
    return true;
}

bool vl53l4cx_start_ranging(void) {
    return VL53LX_StartMeasurement(&_dev) == VL53LX_ERROR_NONE;
}

bool vl53l4cx_stop_ranging(void) {
    return VL53LX_StopMeasurement(&_dev) == VL53LX_ERROR_NONE;
}

bool vl53l4cx_is_ready(void) {
    uint8_t ready = 0;
    VL53LX_GetMeasurementDataReady(&_dev, &ready);
    return ready == 1;
}

uint16_t vl53l4cx_read_distance_mm(void) {
    VL53LX_MultiRangingData_t data;

    VL53LX_Error status = VL53LX_GetMultiRangingData(&_dev, &data);
    VL53LX_ClearInterruptAndStartMeasurement(&_dev);

    if (status != VL53LX_ERROR_NONE) return 0;

    // CX can report several targets but the ground is the closest valid one
    uint16_t best = 0;
    for (int i = 0; i < data.NumberOfObjectsFound; i++) {
        uint8_t rs = data.RangeData[i].RangeStatus;
        if ((rs == VL53LX_RANGESTATUS_RANGE_VALID ||
             rs == VL53LX_RANGESTATUS_RANGE_VALID_MERGED_PULSE) &&
            data.RangeData[i].RangeMilliMeter > 0) {
            uint16_t d = (uint16_t)data.RangeData[i].RangeMilliMeter;
            if (best == 0 || d < best) best = d;
        }
    }

    return best;
}
