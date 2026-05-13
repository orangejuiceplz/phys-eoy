#include <stdio.h>
#include "../include/flight_math.h"
int main(void) {
    Lander lander = {
        .mass = 0,
        .velocity = 0,
        .drag_coefficient = 0,
        .parachute_altitude = 0,
        .altitude = 0,
        .area = 0,
        .acceleration = 0,
        .name= "sky-lander",
        .deployed = false,
        .thrust = 0,
        .suicide_altitude = 0,
        .motor_active = false,
    };

    return 0;
}