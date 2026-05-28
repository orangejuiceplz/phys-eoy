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

#ifndef AP_PHYS_1_EOY_VL53L4CX_H
#define AP_PHYS_1_EOY_VL53L4CX_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/i2c.h"

#define VL53L4CX_ADDR 0x29
#define VL53L4CX_MAX_RANGE_MM 6000

// needs ST's ULD adapted for Pico I2C — not a simple register sensor
bool vl53l4cx_init(i2c_inst_t *i2c, uint8_t addr);
bool vl53l4cx_read_distance_mm(i2c_inst_t *i2c, uint8_t addr, int32_t *distance_mm);

#endif
