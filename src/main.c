#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "../include/flight_math.h"

// breadboard
#define I2C_PORT i2c0
#define SDA_PIN 0
#define SCL_PIN 1

// BMP280 registers
#define BMP280_ADDR 0x77
#define REG_ID 0xD0      // chipID register

int main(void) {
    stdio_init_all();

    // second param is kHz
    i2c_init(I2C_PORT, 400 * 1000);

    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    uint8_t reg = REG_ID;
    uint8_t chip_id[1];
    int i = 0;
    while (true) {
        i++;
        chip_id[0] = 0;

        i2c_write_blocking(I2C_PORT, BMP280_ADDR, &reg, 1, true);
        i2c_read_blocking(I2C_PORT, BMP280_ADDR, chip_id, 1, false);

        printf("\n Diagnostics: \n");
        if (chip_id[0] == 0x58) {
            printf("[SYSTEM] BMP280 Altimeter Found! (ID: 0x%X)\n", chip_id[0]);
            printf("[SYSTEM] I2C Data Link: STABLE\n");
            printf("[SYSTEM] Iteration: ");
            printf("%d", i);
        } else {
            printf("[ERROR] Altimeter failed to respond. (Read ID: 0x%X)\n", chip_id[0]);
            printf("[ERROR] check wiring");
        }

        sleep_ms(1);
    }

    return 0;
}