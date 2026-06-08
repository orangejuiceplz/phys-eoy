#include <stdio.h>
#include "pico/stdlib.h"
#include "../include/flight_math.h"
#include "../include/3.3v/bmp280.h"
#include "../include/state_machine.h"
#include "../include/5v/servo.h"
#include "../include/5v/esc.h"
#include "../include/3.3v/mpu6050.h"
#include "../include/3.3v/vl53l4cx.h"
#include "../include/fusion.h"
#include "../include/flash_state.h"

#define I2C_PORT i2c0
#define SDA_PIN 0
#define SCL_PIN 1

#define BMP280_ADDR 0x77
#define LED_PIN 25

#define ALPHA 0.15
#define LOOP_DELAY_MS 20
#define BASELINE_SAMPLES 50

// suicide burn throttle — full power for TWR > 1.0 
#define BURN_THROTTLE 1.0f

static void led_blink(int count, int ms) {
    for (int i = 0; i < count; i++) {
        gpio_put(LED_PIN, 1);
        sleep_ms(ms);
        gpio_put(LED_PIN, 0);
        sleep_ms(ms);
    }
}

int main(void) {
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

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

    printf("Servo test...\n");
    servo_set_angle(SERVO_PIN, 0.0f);
    sleep_ms(500);
    servo_set_angle(SERVO_PIN, 90.0f);
    sleep_ms(500);
    servo_set_angle(SERVO_PIN, 0.0f);
    sleep_ms(500);
    printf("Servo test done\n");

    bool imu_available = mpu6050_init(I2C_PORT, MPU6050_ADDR);

    bool tof_available = vl53l4cx_init(I2C_PORT, VL53L4CX_ADDR);
    if (tof_available) {
        vl53l4cx_start_ranging();
    }

    esc_init(ESC_PIN);
    printf("[ESC] Arming...\n");
    esc_arm(ESC_PIN);
    printf("[ESC] Armed\n");

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
        .altitude = ground_alt,
        .area = 0.01,
        .acceleration = 0.0,
        .name = "NII",
        .deployed = false,
        .thrust = 0.0,
        .suicide_altitude = 0.0,
        .motor_active = false,
        .ground_altitude = ground_alt,
        .imu_freefall = false,
        .tof_distance_mm = 0,
        .chute_cd = 1.0,          // round parachute
        .chute_area = 0.292,      // 24" StratoChute → pi × 0.305 ^2
        .max_thrust = 7.0         // ~700g EDF thrust in Newtons
    };

    FlightState state = STATE_IDLE;
    double prev_altitude = ground_alt;
    double filtered_altitude = ground_alt;
    const double dt = LOOP_DELAY_MS / 1000.0;

    FusionState fusion;
    fusion_init(&fusion);

    FlashState saved;
    if (flash_state_read(&saved) && saved.chute_deployed) {
        printf("[RECOVERY] Brownout detected — chute was deployed, resuming CHUTE\n");
        state = STATE_CHUTE;
        lander.deployed = true;
        servo_set_angle(SERVO_PIN, 90.0f);
    }

    double initial_deploy = calculate_deploy_altitude(&lander);
    printf("Ground altitude: %.2f m ASL\n", ground_alt);
    printf("IMU (MPU-6050): %s\n", imu_available ? "OK" : "NOT FOUND");
    printf("ToF (VL53L4CX): %s\n", tof_available ? "OK" : "NOT FOUND");
    printf("Chute deploy: %.1f m AGL (dynamic, v=0 estimate)\n", initial_deploy);
    printf("Burn trigger: %d mm ToF\n", BURN_ALTITUDE_MM);
    printf("State: %s\n", state_to_string(state));
    printf("Keys: b=baseline  r=reset  s=status  t=servo  m=motor  ?=help\n\n");

    led_blink(3, 200);

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
                    lander.motor_active = false;
                    esc_kill(ESC_PIN);
                    flash_state_clear();
                    printf("[CMD] Reset -> IDLE (flash cleared)\n");
                    break;
                case 's':
                    printf("[STATUS] State:%s Alt:%.2f Ground:%.2f AGL:%.2f Vel:%.2f FF:%s ToF:%dmm\n",
                           state_to_string(state), lander.altitude, lander.ground_altitude,
                           lander.altitude - lander.ground_altitude, lander.velocity,
                           lander.imu_freefall ? "YES" : "no",
                           lander.tof_distance_mm);
                    break;
                case '1': state = STATE_IDLE; lander.deployed = false; esc_kill(ESC_PIN); lander.motor_active = false; printf("[CMD] -> IDLE\n"); break;
                case '2': state = STATE_FREEFALL; printf("[CMD] -> FREEFALL\n"); break;
                case '3':
                    state = STATE_CHUTE;
                    lander.deployed = true;
                    servo_set_angle(SERVO_PIN, 90.0f);
                    printf("[CMD] -> CHUTE (servo fired)\n");
                    break;
                case '4':
                    state = STATE_BURN;
                    lander.motor_active = true;
                    esc_set_throttle(ESC_PIN, BURN_THROTTLE);
                    printf("[CMD] -> BURN (motor on)\n");
                    break;
                case '5':
                    state = STATE_LANDED;
                    lander.motor_active = false;
                    esc_kill(ESC_PIN);
                    printf("[CMD] -> LANDED\n");
                    break;
                case 't':
                    printf("[CMD] Servo sweep test\n");
                    servo_set_angle(SERVO_PIN, 0.0f);
                    sleep_ms(500);
                    servo_set_angle(SERVO_PIN, 90.0f);
                    sleep_ms(500);
                    servo_set_angle(SERVO_PIN, 0.0f);
                    printf("[CMD] Servo test done\n");
                    break;
                case 'm':
                    printf("[CMD] Motor test — ramping to 50%%\n");
                    for (float t = 0.1f; t <= 0.5f; t += 0.1f) {
                        esc_set_throttle(ESC_PIN, t);
                        sleep_ms(300);
                    }
                    esc_kill(ESC_PIN);
                    printf("[CMD] Motor test done\n");
                    break;
                case 'k':
                    if (!lander.motor_active) {
                        lander.motor_active = true;
                        esc_set_throttle(ESC_PIN, 0.5f);
                        printf("[CMD] Motor ON (50%%) — press k or r to stop\n");
                    } else {
                        lander.motor_active = false;
                        esc_kill(ESC_PIN);
                        printf("[CMD] Motor OFF\n");
                    }
                    break;
                case '?':
                    printf("[HELP] b=baseline r=reset s=status t=servo m=motor k=motor toggle 1-5=states\n");
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

        double baro_vel = (filtered_altitude - prev_altitude) / dt;
        double imu_vert_accel = 0.0;
        if (imu_available) {
            imu_vert_accel = (imu.accel_z - 1.0) * 9.8;  // subtract 1g, convert to m/s^2
        }
        fusion_update(&fusion, baro_vel, imu_vert_accel, dt);
        lander.velocity = fusion.velocity;
        prev_altitude = filtered_altitude;

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

        if (tof_available && (state == STATE_CHUTE || state == STATE_BURN)) {
            if (vl53l4cx_is_ready()) {
                uint16_t raw_tof = vl53l4cx_read_distance_mm();
                if (raw_tof > 0) {
                    lander.tof_distance_mm = raw_tof;
                } else {
                    double agl_m = lander.altitude - lander.ground_altitude;
                    lander.tof_distance_mm = (agl_m > 0) ? (uint16_t)(agl_m * 1000) : 0;
                    printf("[ToF] FAILSAFE: using baro AGL %dmm\n", lander.tof_distance_mm);
                }
            }
        } else if (state == STATE_CHUTE || state == STATE_BURN) {
            double agl_m = lander.altitude - lander.ground_altitude;
            lander.tof_distance_mm = (agl_m > 0) ? (uint16_t)(agl_m * 1000) : 0;
            printf("[ToF] NO SENSOR: using baro AGL %dmm\n", lander.tof_distance_mm);
        } else {
            lander.tof_distance_mm = 0;
        }

        FlightState next = evaluate_state(state, &lander);
        if (next != state) {
            printf(">>> STATE: %s -> %s <<<\n",
                   state_to_string(state), state_to_string(next));

            if (next == STATE_CHUTE && !lander.deployed) {
                flash_state_write(STATE_CHUTE, 1);  
                lander.deployed = true;
                servo_set_angle(SERVO_PIN, 90.0f);
                printf(">>> PARACHUTE DEPLOYED <<<\n");
            }

            if (next == STATE_BURN) {
                lander.motor_active = true;
                esc_set_throttle(ESC_PIN, BURN_THROTTLE);
                printf(">>> SUICIDE BURN IGNITION <<<\n");
            }

            if (next == STATE_LANDED) {
                lander.motor_active = false;
                esc_kill(ESC_PIN);
                printf(">>> LANDED — MOTOR OFF <<<\n");
            }

            state = next;
        }

        printf("[%s] AGL: %.2f m | Vel: %.2f m/s | FF: %s",
               state_to_string(state), agl, lander.velocity,
               lander.imu_freefall ? "YES" : "no");
        if (state == STATE_FREEFALL) {
            printf(" | Deploy: %.1fm", calculate_deploy_altitude(&lander));
        }
        if (state == STATE_CHUTE || state == STATE_BURN) {
            printf(" | ToF: %dmm", lander.tof_distance_mm);
        }
        printf("\n");

        gpio_put(LED_PIN, state == STATE_BURN ? 1 : 0);

        sleep_ms(LOOP_DELAY_MS);
    }

    return 0;
}