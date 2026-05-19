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
    return (l->velocity * l->velocity) / (2 * max_braking_acceleration);
}



