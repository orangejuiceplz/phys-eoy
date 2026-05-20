#include <stdio.h>
#include "pico/stdlib.h"
#include "../include/flight_math.h"
#include "../include/bmp280.h"

// breadboard
#define I2C_PORT i2c0
#define SDA_PIN 0
#define SCL_PIN 1

// BMP280 registers
#define BMP280_ADDR 0x77

#define AVG_WINDOW 10

int main(void) {
    stdio_init_all();

    // second param is kHz
    i2c_init(I2C_PORT, 400 * 1000);

    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    sleep_ms(3000);

    bmp280_calibration_data calibration_struct;
    bmp280_read_calibration(I2C_PORT, BMP280_ADDR, &calibration_struct);

    bmp280_configure(I2C_PORT, BMP280_ADDR);
    bmp280_wake_up(I2C_PORT, BMP280_ADDR);

    sleep_ms(500);

    double alt_buffer[AVG_WINDOW];
    int sample_count = 0;
    int buf_index = 0;

    while (true) {

        // i need some sort of EC, cause i'm varying a little
        int32_t raw_temp, raw_pressure;

        bmp280_get_raw_measurements(I2C_PORT, BMP280_ADDR, &raw_temp, &raw_pressure);

        double true_temp = bmp280_compensate_temp(raw_temp, &calibration_struct);
        double true_pressure = bmp280_compensate_pressure(raw_pressure, &calibration_struct);

        double current_altitude = calculate_altitude(true_pressure);

        alt_buffer[buf_index] = current_altitude;
        buf_index = (buf_index + 1) % AVG_WINDOW;
        if (sample_count < AVG_WINDOW) sample_count++;

        double avg_altitude = 0.0;
        for (int i = 0; i < sample_count; i++) {
            avg_altitude += alt_buffer[i];
        }
        avg_altitude /= sample_count;

        printf("Altitude: %.2f meters (Pressure: %.2f Pa)\n", avg_altitude, true_pressure);

        sleep_ms(100);
    }

    return 0;
}