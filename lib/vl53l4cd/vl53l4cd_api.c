/*
 * VL53L4CD ULD API — Ported to C for Raspberry Pi Pico SDK
 * Original: STMicroelectronics (STSW-IMG026), stm32duino/VL53L4CD
 *
 * Copyright (c) 2021 STMicroelectronics — BSD-3-Clause (API logic + config blob)
 * Pico port (c) 2026 orangejuiceplz — MIT License (platform layer)
 */

#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "vl53l4cd_api.h"

/* ── platform layer (Pico SDK I2C) ──────────────────────────────────────── */

static i2c_inst_t *_i2c = NULL;

void vl53l4cd_set_i2c(i2c_inst_t *i2c) {
    _i2c = i2c;
}

static uint8_t i2c_read(uint8_t addr7, uint16_t reg, uint8_t *buf, uint32_t len) {
    uint8_t reg_buf[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };
    if (i2c_write_blocking(_i2c, addr7, reg_buf, 2, true) < 0) return 1;
    if (i2c_read_blocking(_i2c, addr7, buf, len, false) < 0) return 1;
    return 0;
}

static uint8_t i2c_write(uint8_t addr7, uint16_t reg, uint8_t *data, uint32_t len) {
    uint8_t buf[6]; // max 2 addr + 4 data
    buf[0] = (uint8_t)(reg >> 8);
    buf[1] = (uint8_t)(reg & 0xFF);
    for (uint32_t i = 0; i < len && i < 4; i++) buf[2 + i] = data[i];
    if (i2c_write_blocking(_i2c, addr7, buf, 2 + len, false) < 0) return 1;
    return 0;
}

// ST ULD uses 8-bit addresses; Pico SDK uses 7-bit
#define ADDR7(dev) ((uint8_t)((dev >> 1) & 0x7F))

uint8_t VL53L4CD_RdByte(uint16_t dev, uint16_t reg, uint8_t *value) {
    return i2c_read(ADDR7(dev), reg, value, 1);
}

uint8_t VL53L4CD_WrByte(uint16_t dev, uint16_t reg, uint8_t value) {
    return i2c_write(ADDR7(dev), reg, &value, 1);
}

uint8_t VL53L4CD_RdWord(uint16_t dev, uint16_t reg, uint16_t *value) {
    uint8_t buf[2] = {0, 0};
    uint8_t status = i2c_read(ADDR7(dev), reg, buf, 2);
    if (!status) *value = ((uint16_t)buf[0] << 8) | buf[1];
    return status;
}

uint8_t VL53L4CD_WrWord(uint16_t dev, uint16_t reg, uint16_t value) {
    uint8_t buf[2] = { (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
    return i2c_write(ADDR7(dev), reg, buf, 2);
}

uint8_t VL53L4CD_RdDWord(uint16_t dev, uint16_t reg, uint32_t *value) {
    uint8_t buf[4] = {0};
    uint8_t status = i2c_read(ADDR7(dev), reg, buf, 4);
    if (!status) {
        *value = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16)
               | ((uint32_t)buf[2] << 8)  | (uint32_t)buf[3];
    }
    return status;
}

uint8_t VL53L4CD_WrDWord(uint16_t dev, uint16_t reg, uint32_t value) {
    uint8_t buf[4] = {
        (uint8_t)((value >> 24) & 0xFF),
        (uint8_t)((value >> 16) & 0xFF),
        (uint8_t)((value >>  8) & 0xFF),
        (uint8_t)((value >>  0) & 0xFF)
    };
    return i2c_write(ADDR7(dev), reg, buf, 4);
}

void VL53L4CD_WaitMs(uint32_t ms) {
    sleep_ms(ms);
}

/* ── ST default configuration blob (0x2D–0x87) ─────────────────────────── */

static const uint8_t VL53L4CD_DEFAULT_CONFIGURATION[] = {
    0x12, 0x00, 0x00, 0x11, 0x02, 0x00, 0x02, 0x08,
    0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00,
    0x00, 0xff, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x20, 0x0b, 0x00, 0x00, 0x02, 0x14, 0x21,
    0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xc8,
    0x00, 0x00, 0x38, 0xff, 0x01, 0x00, 0x08, 0x00,
    0x00, 0x01, 0xcc, 0x07, 0x01, 0xf1, 0x05, 0x00,
    0xa0, 0x00, 0x80, 0x08, 0x38, 0x00, 0x00, 0x00,
    0x00, 0x0f, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x07, 0x05, 0x06, 0x06, 0x00,
    0x00, 0x02, 0xc7, 0xff, 0x9B, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00
};

/* ── API functions (C port of stm32duino/VL53L4CD) ─────────────────────── */

VL53L4CD_ERROR VL53L4CD_GetSensorId(uint16_t dev, uint16_t *id) {
    return VL53L4CD_RdWord(dev, VL53L4CD_IDENTIFICATION_MODEL_ID, id);
}

VL53L4CD_ERROR VL53L4CD_ClearInterrupt(uint16_t dev) {
    return VL53L4CD_WrByte(dev, VL53L4CD_SYSTEM_INTERRUPT_CLEAR, 0x01);
}

VL53L4CD_ERROR VL53L4CD_StopRanging(uint16_t dev) {
    return VL53L4CD_WrByte(dev, VL53L4CD_SYSTEM_START, 0x00);
}

VL53L4CD_ERROR VL53L4CD_CheckForDataReady(uint16_t dev, uint8_t *ready) {
    VL53L4CD_ERROR status = VL53L4CD_ERROR_NONE;
    uint8_t temp, int_pol;

    status |= VL53L4CD_RdByte(dev, VL53L4CD_GPIO_HV_MUX_CTRL, &temp);
    int_pol = ((temp & 0x10) >> 4) ? 0 : 1;

    status |= VL53L4CD_RdByte(dev, VL53L4CD_GPIO_TIO_HV_STATUS, &temp);
    *ready = ((temp & 1) == int_pol) ? 1 : 0;

    return status;
}

VL53L4CD_ERROR VL53L4CD_SensorInit(uint16_t dev) {
    VL53L4CD_ERROR status = VL53L4CD_ERROR_NONE;
    uint8_t tmp;
    uint16_t i = 0;

    // wait for sensor boot (firmware status == 0x3)
    do {
        status |= VL53L4CD_RdByte(dev, VL53L4CD_FIRMWARE_SYSTEM_STATUS, &tmp);
        if (tmp == 0x3) break;
        if (i++ >= 1000) return VL53L4CD_ERROR_TIMEOUT;
        VL53L4CD_WaitMs(1);
    } while (1);

    // load default configuration blob
    for (uint8_t addr = 0x2D; addr <= 0x87; addr++) {
        status |= VL53L4CD_WrByte(dev, addr, VL53L4CD_DEFAULT_CONFIGURATION[addr - 0x2D]);
    }

    // run VHV calibration
    status |= VL53L4CD_WrByte(dev, VL53L4CD_SYSTEM_START, 0x40);
    i = 0;
    do {
        status |= VL53L4CD_CheckForDataReady(dev, &tmp);
        if (tmp == 1) break;
        if (i++ >= 1000) return VL53L4CD_ERROR_TIMEOUT;
        VL53L4CD_WaitMs(1);
    } while (1);

    status |= VL53L4CD_ClearInterrupt(dev);
    status |= VL53L4CD_StopRanging(dev);
    status |= VL53L4CD_WrByte(dev, VL53L4CD_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, 0x09);
    status |= VL53L4CD_WrByte(dev, 0x0B, 0);
    status |= VL53L4CD_WrWord(dev, 0x0024, 0x500);

    status |= VL53L4CD_SetRangeTiming(dev, 50, 0);

    return status;
}

VL53L4CD_ERROR VL53L4CD_SetRangeTiming(uint16_t dev,
    uint32_t timing_budget_ms, uint32_t inter_measurement_ms) {
    VL53L4CD_ERROR status = VL53L4CD_ERROR_NONE;
    uint16_t clock_pll, osc_frequency, ms_byte;
    uint32_t macro_period_us = 0, timing_budget_us = 0, ls_byte, tmp;
    float inter_measurement_factor = 1.055f;

    status |= VL53L4CD_RdWord(dev, 0x0006, &osc_frequency);
    if (osc_frequency != 0) {
        timing_budget_us = timing_budget_ms * 1000;
        macro_period_us = (uint32_t)(2304 * ((uint32_t)0x40000000 / (uint32_t)osc_frequency)) >> 6;
    } else {
        return VL53L4CD_ERROR_INVALID_ARGUMENT;
    }

    if (timing_budget_ms < 10 || timing_budget_ms > 200) {
        return VL53L4CD_ERROR_INVALID_ARGUMENT;
    }

    if (inter_measurement_ms == 0) {
        status |= VL53L4CD_WrDWord(dev, VL53L4CD_INTERMEASUREMENT_MS, 0);
        timing_budget_us -= 2500;
    } else if (inter_measurement_ms > timing_budget_ms) {
        status |= VL53L4CD_RdWord(dev, VL53L4CD_RESULT_OSC_CALIBRATE_VAL, &clock_pll);
        clock_pll &= 0x3FF;
        inter_measurement_factor *= (float)inter_measurement_ms * (float)clock_pll;
        status |= VL53L4CD_WrDWord(dev, VL53L4CD_INTERMEASUREMENT_MS,
                                   (uint32_t)inter_measurement_factor);
        timing_budget_us -= 4300;
        timing_budget_us /= 2;
    } else {
        return VL53L4CD_ERROR_INVALID_ARGUMENT;
    }

    ms_byte = 0;
    timing_budget_us <<= 12;
    tmp = macro_period_us * 16;
    ls_byte = ((timing_budget_us + ((tmp >> 6) >> 1)) / (tmp >> 6)) - 1;

    while ((ls_byte & 0xFFFFFF00U) > 0U) {
        ls_byte >>= 1;
        ms_byte++;
    }
    ms_byte = (ms_byte << 8) + (uint16_t)(ls_byte & 0xFF);
    status |= VL53L4CD_WrWord(dev, VL53L4CD_RANGE_CONFIG_A, ms_byte);

    ms_byte = 0;
    tmp = macro_period_us * 12;
    ls_byte = ((timing_budget_us + ((tmp >> 6) >> 1)) / (tmp >> 6)) - 1;

    while ((ls_byte & 0xFFFFFF00U) > 0U) {
        ls_byte >>= 1;
        ms_byte++;
    }
    ms_byte = (ms_byte << 8) + (uint16_t)(ls_byte & 0xFF);
    status |= VL53L4CD_WrWord(dev, VL53L4CD_RANGE_CONFIG_B, ms_byte);

    return status;
}

VL53L4CD_ERROR VL53L4CD_StartRanging(uint16_t dev) {
    VL53L4CD_ERROR status = VL53L4CD_ERROR_NONE;
    uint8_t data_ready;
    uint16_t i = 0;
    uint32_t tmp;

    status |= VL53L4CD_RdDWord(dev, VL53L4CD_INTERMEASUREMENT_MS, &tmp);

    if (tmp == 0) {
        status |= VL53L4CD_WrByte(dev, VL53L4CD_SYSTEM_START, 0x21);
    } else {
        status |= VL53L4CD_WrByte(dev, VL53L4CD_SYSTEM_START, 0x40);
    }

    do {
        status |= VL53L4CD_CheckForDataReady(dev, &data_ready);
        if (data_ready == 1) break;
        if (i++ >= 1000) return VL53L4CD_ERROR_TIMEOUT;
        VL53L4CD_WaitMs(1);
    } while (1);

    status |= VL53L4CD_ClearInterrupt(dev);
    return status;
}

VL53L4CD_ERROR VL53L4CD_GetResult(uint16_t dev, VL53L4CD_Result_t *result) {
    VL53L4CD_ERROR status = VL53L4CD_ERROR_NONE;
    uint16_t temp_16;
    uint8_t temp_8;
    uint8_t status_rtn[24] = {
        255, 255, 255, 5, 2, 4, 1, 7, 3,
        0, 255, 255, 9, 13, 255, 255, 255, 255, 10, 6,
        255, 255, 11, 12
    };

    status |= VL53L4CD_RdByte(dev, VL53L4CD_RESULT_RANGE_STATUS, &temp_8);
    temp_8 &= 0x1F;
    if (temp_8 < 24) temp_8 = status_rtn[temp_8];
    result->range_status = temp_8;

    status |= VL53L4CD_RdWord(dev, VL53L4CD_RESULT_SPAD_NB, &temp_16);
    result->number_of_spad = temp_16 / 256;

    if (result->number_of_spad == 0) {
        result->range_status = 255;
        return status;
    }

    status |= VL53L4CD_RdWord(dev, VL53L4CD_RESULT_SIGNAL_RATE, &temp_16);
    result->signal_rate_kcps = temp_16 * 8;

    status |= VL53L4CD_RdWord(dev, VL53L4CD_RESULT_AMBIENT_RATE, &temp_16);
    result->ambient_rate_kcps = temp_16 * 8;

    status |= VL53L4CD_RdWord(dev, VL53L4CD_RESULT_SIGMA, &temp_16);
    result->sigma_mm = temp_16 / 4;

    status |= VL53L4CD_RdWord(dev, VL53L4CD_RESULT_DISTANCE, &temp_16);
    result->distance_mm = temp_16;

    result->signal_per_spad_kcps = result->signal_rate_kcps / result->number_of_spad;
    result->ambient_per_spad_kcps = result->ambient_rate_kcps / result->number_of_spad;

    return status;
}
