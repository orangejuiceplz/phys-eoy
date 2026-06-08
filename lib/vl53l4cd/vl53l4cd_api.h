/*
 * VL53L4CD ULD API — Ported to C for Raspberry Pi Pico SDK
 * Original: STMicroelectronics (STSW-IMG026), stm32duino/VL53L4CD
 *
 * Copyright (c) 2021 STMicroelectronics — BSD-3-Clause
 * Pico port (c) 2026 orangejuiceplz — MIT License
 */

#ifndef VL53L4CD_API_H
#define VL53L4CD_API_H

#include <stdint.h>
#include "hardware/i2c.h"

typedef uint8_t VL53L4CD_ERROR;

#define VL53L4CD_ERROR_NONE             ((uint8_t)0U)
#define VL53L4CD_ERROR_INVALID_ARGUMENT ((uint8_t)254U)
#define VL53L4CD_ERROR_TIMEOUT          ((uint8_t)255U)

// registers
#define VL53L4CD_SOFT_RESET                        ((uint16_t)0x0000)
#define VL53L4CD_I2C_SLAVE_DEVICE_ADDRESS          ((uint16_t)0x0001)
#define VL53L4CD_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND ((uint16_t)0x0008)
#define VL53L4CD_XTALK_PLANE_OFFSET_KCPS           ((uint16_t)0x0016)
#define VL53L4CD_XTALK_X_PLANE_GRADIENT_KCPS       ((uint16_t)0x0018)
#define VL53L4CD_XTALK_Y_PLANE_GRADIENT_KCPS       ((uint16_t)0x001A)
#define VL53L4CD_RANGE_OFFSET_MM                   ((uint16_t)0x001E)
#define VL53L4CD_INNER_OFFSET_MM                   ((uint16_t)0x0020)
#define VL53L4CD_OUTER_OFFSET_MM                   ((uint16_t)0x0022)
#define VL53L4CD_I2C_FAST_MODE_PLUS                ((uint16_t)0x002D)
#define VL53L4CD_GPIO_HV_MUX_CTRL                  ((uint16_t)0x0030)
#define VL53L4CD_GPIO_TIO_HV_STATUS                ((uint16_t)0x0031)
#define VL53L4CD_SYSTEM_INTERRUPT                   ((uint16_t)0x0046)
#define VL53L4CD_RANGE_CONFIG_A                    ((uint16_t)0x005E)
#define VL53L4CD_RANGE_CONFIG_B                    ((uint16_t)0x0061)
#define VL53L4CD_RANGE_CONFIG_SIGMA_THRESH         ((uint16_t)0x0064)
#define VL53L4CD_MIN_COUNT_RATE_RTN_LIMIT_MCPS     ((uint16_t)0x0066)
#define VL53L4CD_INTERMEASUREMENT_MS               ((uint16_t)0x006C)
#define VL53L4CD_THRESH_HIGH                       ((uint16_t)0x0072)
#define VL53L4CD_THRESH_LOW                        ((uint16_t)0x0074)
#define VL53L4CD_SYSTEM_INTERRUPT_CLEAR            ((uint16_t)0x0086)
#define VL53L4CD_SYSTEM_START                      ((uint16_t)0x0087)
#define VL53L4CD_RESULT_RANGE_STATUS               ((uint16_t)0x0089)
#define VL53L4CD_RESULT_SPAD_NB                    ((uint16_t)0x008C)
#define VL53L4CD_RESULT_SIGNAL_RATE                ((uint16_t)0x008E)
#define VL53L4CD_RESULT_AMBIENT_RATE               ((uint16_t)0x0090)
#define VL53L4CD_RESULT_SIGMA                      ((uint16_t)0x0092)
#define VL53L4CD_RESULT_DISTANCE                   ((uint16_t)0x0096)
#define VL53L4CD_RESULT_OSC_CALIBRATE_VAL          ((uint16_t)0x00DE)
#define VL53L4CD_FIRMWARE_SYSTEM_STATUS            ((uint16_t)0x00E5)
#define VL53L4CD_IDENTIFICATION_MODEL_ID           ((uint16_t)0x010F)

typedef struct {
    uint8_t  range_status;
    uint16_t distance_mm;
    uint16_t ambient_rate_kcps;
    uint16_t ambient_per_spad_kcps;
    uint16_t signal_rate_kcps;
    uint16_t signal_per_spad_kcps;
    uint16_t number_of_spad;
    uint16_t sigma_mm;
} VL53L4CD_Result_t;

// platform — must be called first to set the I2C peripheral
void vl53l4cd_set_i2c(i2c_inst_t *i2c);

// platform I/O (16-bit register addresses)
uint8_t VL53L4CD_RdByte(uint16_t dev, uint16_t reg, uint8_t *value);
uint8_t VL53L4CD_WrByte(uint16_t dev, uint16_t reg, uint8_t value);
uint8_t VL53L4CD_RdWord(uint16_t dev, uint16_t reg, uint16_t *value);
uint8_t VL53L4CD_WrWord(uint16_t dev, uint16_t reg, uint16_t value);
uint8_t VL53L4CD_RdDWord(uint16_t dev, uint16_t reg, uint32_t *value);
uint8_t VL53L4CD_WrDWord(uint16_t dev, uint16_t reg, uint32_t value);
void    VL53L4CD_WaitMs(uint32_t ms);

// API (dev = 8-bit I2C address, e.g. 0x52 for default)
VL53L4CD_ERROR VL53L4CD_SensorInit(uint16_t dev);
VL53L4CD_ERROR VL53L4CD_GetSensorId(uint16_t dev, uint16_t *id);
VL53L4CD_ERROR VL53L4CD_SetRangeTiming(uint16_t dev, uint32_t timing_budget_ms, uint32_t inter_measurement_ms);
VL53L4CD_ERROR VL53L4CD_StartRanging(uint16_t dev);
VL53L4CD_ERROR VL53L4CD_StopRanging(uint16_t dev);
VL53L4CD_ERROR VL53L4CD_CheckForDataReady(uint16_t dev, uint8_t *ready);
VL53L4CD_ERROR VL53L4CD_GetResult(uint16_t dev, VL53L4CD_Result_t *result);
VL53L4CD_ERROR VL53L4CD_ClearInterrupt(uint16_t dev);

#endif
