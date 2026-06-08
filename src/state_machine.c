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

#include "../include/state_machine.h"
#include "../include/flight_math.h"

const char* state_to_string(FlightState state) {
    switch (state) {
        case STATE_IDLE:     return "IDLE";
        case STATE_FREEFALL: return "FREEFALL";
        case STATE_CHUTE:    return "CHUTE";
        case STATE_BURN:     return "BURN";
        case STATE_LANDED:   return "LANDED";
        default:             return "UNKNOWN";
    }
}

FlightState evaluate_state(FlightState current, Lander *l) {
    double agl = l->altitude - l->ground_altitude;

    switch (current) {
        case STATE_IDLE:
            if (agl > 0 && (l->imu_freefall || l->velocity < FREEFALL_VELOCITY_THRESHOLD)) {
                return STATE_FREEFALL;
            }
            break;

        case STATE_FREEFALL: {
            double deploy_agl = calculate_deploy_altitude(l);
            if (agl <= deploy_agl) {
                return STATE_CHUTE;
            }
            break;
        }

        case STATE_CHUTE:
            if (l->tof_distance_mm > 0) {
                double tof_m = l->tof_distance_mm / 1000.0;
                double brake_dist = calculate_braking_distance(l, l->max_thrust);
                if (tof_m <= brake_dist) {
                    return STATE_BURN;
                }
            }
            break;

        case STATE_BURN:
            if (l->motor_active &&
                l->velocity > -LANDED_VELOCITY && l->velocity < LANDED_VELOCITY &&
                l->tof_distance_mm > 0 && l->tof_distance_mm < GROUND_THRESHOLD_MM) {
                return STATE_LANDED;
            }
            break;

        case STATE_LANDED:
            break;
    }

    return current;
}
