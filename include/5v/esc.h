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

#ifndef AP_PHYS_1_EOY_ESC_H
#define AP_PHYS_1_EOY_ESC_H

#include "pico/stdlib.h"

// ESC uses same 50Hz PWM as servos, pulse width controls throttle
// 1000μs = off, 2000μs = full throttle
#define ESC_THROTTLE_MIN 1000
#define ESC_THROTTLE_MAX 2000

void esc_init(uint pin);

// arms the ESC by holding minimum throttle for 3 seconds
// MUST be called before any throttle commands
void esc_arm(uint pin);

// throttle: 0.0 (off) to 1.0 (full)
void esc_set_throttle(uint pin, float throttle);

void esc_kill(uint pin);

#endif
