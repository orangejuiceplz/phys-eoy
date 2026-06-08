/*
 * MIT License
 *
 * Copyright (c) 2026 orangejuiceplz
 * CREATED on 5/12/26 
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


#ifndef AP_PHYS_1_EOY_VARIABLES_H
#define AP_PHYS_1_EOY_VARIABLES_H

#include <stdbool.h>
#include <stdint.h>

#define g 9.8
#define air_density 1.225

#define FREEFALL_VELOCITY_THRESHOLD -0.5  // m/s
#define FREEFALL_G_THRESHOLD          0.4  // g — below this = freefall
#define VELOCITY_DEAD_ZONE            0.05 // m/s — below this = stationary
#define CHUTE_OPEN_TIME               1.5  // seconds for parachute to inflate
#define MIN_DEPLOY_AGL                3.0  // absolute minimum deploy altitude (safety floor)

#define ESC_PIN 16
#define BURN_ALTITUDE_MM 800  // ToF distance to trigger suicide burn
#define GROUND_THRESHOLD_MM 50  // ToF distance below which = on the ground
#define LANDED_VELOCITY  0.3  // m/s — below this + burn active = landed

typedef struct {

    double mass;
    double velocity;
    double drag_coefficient;
    double altitude;
    double area;
    double acceleration;
    char name[8];
    bool deployed;
    double thrust;
    double suicide_altitude;
    bool motor_active;
    double ground_altitude;
    bool imu_freefall;
    uint16_t tof_distance_mm;
    double chute_cd;          // parachute drag coefficient (~1.0 for round)
    double chute_area;        // parachute area in m² (24" → ~0.292)
    double max_thrust;        // EDF max thrust in N (for braking distance calc)

} Lander;

#endif
