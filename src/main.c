#include <stdio.h>
#include "pico/stdlib.h"
#include "../include/flight_math.h"
#include "../include/3.3v/bmp280.h"
#include "../include/state_machine.h"
#include "../include/5v/servo.h"
#include "../include/3.3v/mpu6050.h"

#define I2C_PORT i2c0
#define SDA_PIN 0
#define SCL_PIN 1

#define BMP280_ADDR 0x77

#define ALPHA 0.15
#define LOOP_DELAY_MS 100
#define BASELINE_SAMPLES 50

int main(void) {
    stdio_init_all();

    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    sleep_ms(5000);

    bmp280_calibration_data cal;
    bmp280_read_calibration(I2C_PORT, BMP280_ADDR, &cal);
    bmp280_configure(I2C_PORT, BMP280_ADDR);
    bmp280_wake_up(I2C_PORT, BMP280_ADDR);
    sleep_ms(1000);

    servo_init(SERVO_PIN);

    // boot sweep — if the servo moves, wiring is good
    printf("Servo test...\n");
    servo_set_angle(SERVO_PIN, 0.0f);
    sleep_ms(500);
    servo_set_angle(SERVO_PIN, 90.0f);
    sleep_ms(500);
    servo_set_angle(SERVO_PIN, 0.0f);
    sleep_ms(500);
    printf("Servo test done\n");

    bool imu_available = mpu6050_init(I2C_PORT, MPU6050_ADDR);

    // throw away first 20 reads so the BMP280 thermally stabilizes
    int32_t dummy_t, dummy_p;
    for (int i = 0; i < 20; i++) {
        bmp280_get_raw_measurements(I2C_PORT, BMP280_ADDR, &dummy_t, &dummy_p);
        bmp280_compensate_temp(dummy_t, &cal);
        bmp280_compensate_pressure(dummy_p, &cal);
        sleep_ms(100);
    }

    double baseline_sum = 0.0;
    for (int i = 0; i < BASELINE_SAMPLES; i++) {
        int32_t raw_t, raw_p;
        bmp280_get_raw_measurements(I2C_PORT, BMP280_ADDR, &raw_t, &raw_p);
        bmp280_compensate_temp(raw_t, &cal);
        double p = bmp280_compensate_pressure(raw_p, &cal);
        baseline_sum += calculate_altitude(p);
        sleep_ms(50);
    }
    double ground_alt = baseline_sum / BASELINE_SAMPLES;

    Lander lander = {
        .mass = 0.5,
        .velocity = 0.0,
        .drag_coefficient = 0.5,
        .parachute_altitude = CHUTE_DEPLOY_AGL,
        .altitude = ground_alt,
        .area = 0.01,
        .acceleration = 0.0,
        .name = "NII",
        .deployed = false,
        .thrust = 0.0,
        .suicide_altitude = 0.0,
        .motor_active = false,
        .ground_altitude = ground_alt,
        .imu_freefall = false
    };

    FlightState state = STATE_IDLE;
    double prev_altitude = ground_alt;
    double filtered_altitude = ground_alt;
    const double dt = LOOP_DELAY_MS / 1000.0;

    printf("Ground altitude: %.2f m ASL\n", ground_alt);
    printf("IMU (MPU-6050): %s\n", imu_available ? "OK" : "NOT FOUND");
    printf("Chute deploy: %.1f m AGL\n", CHUTE_DEPLOY_AGL);
    printf("State: %s\n", state_to_string(state));
    printf("Keys: b=baseline  r=reset  s=status  ?=help\n\n");

    while (true) {
        int ch = getchar_timeout_us(0);
        if (ch != PICO_ERROR_TIMEOUT) {
            switch (ch) {
                case 'b':
                    lander.ground_altitude = filtered_altitude;
                    printf("[CMD] Baseline reset -> %.2f m\n", lander.ground_altitude);
                    break;
                case 'r':
                    state = STATE_IDLE;
                    lander.deployed = false;
                    lander.imu_freefall = false;
                    printf("[CMD] Reset -> IDLE\n");
                    break;
                case 's':
                    printf("[STATUS] State:%s Alt:%.2f Ground:%.2f AGL:%.2f Vel:%.2f FF:%s\n",
                           state_to_string(state), lander.altitude, lander.ground_altitude,
                           lander.altitude - lander.ground_altitude, lander.velocity,
                           lander.imu_freefall ? "YES" : "no");
                    break;
                case '1': state = STATE_IDLE; lander.deployed = false; printf("[CMD] -> IDLE\n"); break;
                case '2': state = STATE_FREEFALL; printf("[CMD] -> FREEFALL\n"); break;
                case '3':
                    state = STATE_CHUTE;
                    lander.deployed = true;
                    servo_set_angle(SERVO_PIN, 90.0f);
                    printf("[CMD] -> CHUTE (servo fired)\n");
                    break;
                case '4': state = STATE_BURN; printf("[CMD] -> BURN\n"); break;
                case '5': state = STATE_LANDED; printf("[CMD] -> LANDED\n"); break;
                case 't':
                    printf("[CMD] Servo sweep test\n");
                    servo_set_angle(SERVO_PIN, 0.0f);
                    sleep_ms(500);
                    servo_set_angle(SERVO_PIN, 90.0f);
                    sleep_ms(500);
                    servo_set_angle(SERVO_PIN, 0.0f);
                    printf("[CMD] Servo test done\n");
                    break;
                case '?':
                    printf("[HELP] b=baseline r=reset s=status t=servo test 1=IDLE 2=FF 3=CHUTE 4=BURN 5=LAND\n");
                    break;
                default:
                    break;
            }
        }

        int32_t raw_t, raw_p;
        bmp280_get_raw_measurements(I2C_PORT, BMP280_ADDR, &raw_t, &raw_p);
        bmp280_compensate_temp(raw_t, &cal);
        double pressure = bmp280_compensate_pressure(raw_p, &cal);
        double raw_alt = calculate_altitude(pressure);

        filtered_altitude = (ALPHA * raw_alt) + ((1.0 - ALPHA) * filtered_altitude);

        lander.altitude = filtered_altitude;
        lander.velocity = (filtered_altitude - prev_altitude) / dt;
        prev_altitude = filtered_altitude;

        // barometer noise produces tiny velocities when stationary
        if (lander.velocity > -VELOCITY_DEAD_ZONE && lander.velocity < VELOCITY_DEAD_ZONE) {
            lander.velocity = 0.0;
        }

        double agl = lander.altitude - lander.ground_altitude;

        mpu6050_data imu;
        if (imu_available && mpu6050_read(I2C_PORT, MPU6050_ADDR, &imu)) {
            lander.imu_freefall = mpu6050_detect_freefall(&imu, FREEFALL_G_THRESHOLD);
        } else {
            lander.imu_freefall = false;
        }

        FlightState next = evaluate_state(state, &lander);
        if (next != state) {
            printf(">>> STATE: %s -> %s <<<\n",
                   state_to_string(state), state_to_string(next));

            if (next == STATE_CHUTE && !lander.deployed) {
                lander.deployed = true;
                servo_set_angle(SERVO_PIN, 90.0f);
                printf(">>> PARACHUTE DEPLOYED <<<\n");
            }

            state = next;
        }

        printf("[%s] AGL: %.2f m | Vel: %.2f m/s | FF: %s\n",
               state_to_string(state), agl, lander.velocity,
               lander.imu_freefall ? "YES" : "no");

        sleep_ms(LOOP_DELAY_MS);
    }

    return 0;
}