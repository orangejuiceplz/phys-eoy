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

#include <math.h>
#include "../include/flight_math.h"

double calculate_drag(double velocity, double drag_coefficient, double area) {
    // Force_drag = 1/2 * rho * v^2 * dC * A
    return 0.5 * air_density * (velocity * velocity) * drag_coefficient * area;
}

// stupid integrator
void update(Lander *l, double dt) {
    const double force_gravity = l->mass * g;
    double force_drag = calculate_drag(l->velocity, l->drag_coefficient, l->area);

    if (l->velocity > 0) {
        force_drag = -force_drag;
    }

    const double net_force = force_drag - force_gravity + l->thrust;

    l->acceleration = net_force / l->mass; //a = sum(F)/m
    l->velocity += l->acceleration * dt;
    l->altitude += l->velocity * dt;
}

double calculate_braking_distance(Lander *l, double max_thrust) {
    const double downward_net_force = max_thrust - (l->mass * g);

    const double max_braking_acceleration = downward_net_force / l->mass;
    if (max_braking_acceleration <= 0) return 9999.0; // can't brake — deploy ASAP
    return (l->velocity * l->velocity) / (2 * max_braking_acceleration);
}

double calculate_deploy_altitude(Lander *l) {
    double v = l->velocity;
    if (v > 0) v = 0; // only care about downward velocity
    double speed = -v; // positive magnitude

    // 1. distance covered while chute opens (falling at current speed)
    double d_open = speed * CHUTE_OPEN_TIME;

    // 2. terminal velocity under parachute: v_t = sqrt(2mg / (rho * Cd * A))
    double v_terminal = 0;
    if (l->chute_cd > 0 && l->chute_area > 0) {
        v_terminal = sqrt(2.0 * l->mass * g / (air_density * l->chute_cd * l->chute_area));
    }

    // 3. deceleration distance from current speed to terminal velocity
    //    d = (v² - vt²) / (2 * a_avg), where a_avg from average drag
    double d_decel = 0;
    if (speed > v_terminal && l->chute_cd > 0) {
        double v_avg = (speed + v_terminal) / 2.0;
        double f_drag = 0.5 * air_density * v_avg * v_avg * l->chute_cd * l->chute_area;
        double a_net = (f_drag / l->mass) - g;
        if (a_net > 0) {
            d_decel = (speed * speed - v_terminal * v_terminal) / (2.0 * a_net);
        }
    }

    // 4. motor braking distance (how far the burn needs)
    double d_brake = calculate_braking_distance(l, l->max_thrust);

    // 5. ToF acquisition buffer (sensor needs to see the ground)
    double d_tof = BURN_ALTITUDE_MM / 1000.0;

    double total = d_open + d_decel + d_brake + d_tof;

    // safety floor
    if (total < MIN_DEPLOY_AGL) total = MIN_DEPLOY_AGL;

    return total;
}

#define atmospheric_pressure 101325.0
#define barometric_scale_height 44330

double calculate_altitude(double pressure) {
    return barometric_scale_height * (1.0 - pow(pressure / atmospheric_pressure, 0.19029));
}
