#include "../include/fusion.h"

// alpha = 0.95 -> 95% accelerometer (short term), 5% barometer (long term correction)
// lowered from 0.98 to 0.95 to account for EDF vibration on the MPU-6050
#define FUSION_ALPHA 0.95

void fusion_init(FusionState *fs) {
    fs->velocity = 0.0;
}

void fusion_update(FusionState *fs, double baro_velocity, double imu_az, double dt) {
    double accel_velocity = fs->velocity + imu_az * dt;
    fs->velocity = FUSION_ALPHA * accel_velocity + (1.0 - FUSION_ALPHA) * baro_velocity;
}
