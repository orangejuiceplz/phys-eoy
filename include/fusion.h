#ifndef FUSION_H
#define FUSION_H

#include "variables.h"
#include "3.3v/mpu6050.h"

typedef struct {
    double velocity;        // fused vertical velocity (m/s, negative = down)
} FusionState;

void fusion_init(FusionState *fs);

// call every loop tick:
//   baro_velocity = (filtered_alt - prev_alt) / dt
//   imu_az        = vertical accel from MPU-6050 in m/s² (subtract 1g)
//   dt            = loop period in seconds
void fusion_update(FusionState *fs, double baro_velocity, double imu_az, double dt);

#endif
