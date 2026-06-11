/*
 * Pico SDK implementation of the ST VL53L4CX platform layer.
 * I2C protocol: 16-bit big-endian register index followed by data bytes.
 */

#include <string.h>

#include "vl53lx_platform.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C_TIMEOUT_US 20000

/* largest single write the driver issues is the device config blob (<200 B) */
#define MAX_I2C_XFER 512

static uint8_t _xfer_buf[MAX_I2C_XFER + 2];

VL53LX_Error VL53LX_CommsInitialise(VL53LX_DEV Dev, uint8_t comms_type,
                                    uint16_t comms_speed_khz) {
    (void)Dev; (void)comms_type; (void)comms_speed_khz;
    /* I2C bus is initialised by the application (main.c) */
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_CommsClose(VL53LX_DEV Dev) {
    (void)Dev;
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WriteMulti(VL53LX_DEV Dev, uint16_t index,
                               uint8_t *pdata, uint32_t count) {
    if (count > MAX_I2C_XFER)
        return VL53LX_ERROR_INVALID_PARAMS;

    _xfer_buf[0] = (uint8_t)(index >> 8);
    _xfer_buf[1] = (uint8_t)(index & 0xFF);
    memcpy(&_xfer_buf[2], pdata, count);

    int ret = i2c_write_timeout_us(Dev->i2c, Dev->i2c_addr, _xfer_buf,
                                   count + 2, false, I2C_TIMEOUT_US);
    return (ret == (int)(count + 2)) ? VL53LX_ERROR_NONE
                                     : VL53LX_ERROR_CONTROL_INTERFACE;
}

VL53LX_Error VL53LX_ReadMulti(VL53LX_DEV Dev, uint16_t index,
                              uint8_t *pdata, uint32_t count) {
    uint8_t reg[2] = { (uint8_t)(index >> 8), (uint8_t)(index & 0xFF) };

    int ret = i2c_write_timeout_us(Dev->i2c, Dev->i2c_addr, reg, 2, true,
                                   I2C_TIMEOUT_US);
    if (ret != 2)
        return VL53LX_ERROR_CONTROL_INTERFACE;

    ret = i2c_read_timeout_us(Dev->i2c, Dev->i2c_addr, pdata, count, false,
                              I2C_TIMEOUT_US);
    return (ret == (int)count) ? VL53LX_ERROR_NONE
                               : VL53LX_ERROR_CONTROL_INTERFACE;
}

VL53LX_Error VL53LX_WrByte(VL53LX_DEV Dev, uint16_t index, uint8_t data) {
    return VL53LX_WriteMulti(Dev, index, &data, 1);
}

VL53LX_Error VL53LX_WrWord(VL53LX_DEV Dev, uint16_t index, uint16_t data) {
    uint8_t buf[2] = { (uint8_t)(data >> 8), (uint8_t)(data & 0xFF) };
    return VL53LX_WriteMulti(Dev, index, buf, 2);
}

VL53LX_Error VL53LX_WrDWord(VL53LX_DEV Dev, uint16_t index, uint32_t data) {
    uint8_t buf[4] = { (uint8_t)(data >> 24), (uint8_t)(data >> 16),
                       (uint8_t)(data >> 8),  (uint8_t)(data & 0xFF) };
    return VL53LX_WriteMulti(Dev, index, buf, 4);
}

VL53LX_Error VL53LX_RdByte(VL53LX_DEV Dev, uint16_t index, uint8_t *pdata) {
    return VL53LX_ReadMulti(Dev, index, pdata, 1);
}

VL53LX_Error VL53LX_RdWord(VL53LX_DEV Dev, uint16_t index, uint16_t *pdata) {
    uint8_t buf[2];
    VL53LX_Error status = VL53LX_ReadMulti(Dev, index, buf, 2);
    if (status == VL53LX_ERROR_NONE)
        *pdata = ((uint16_t)buf[0] << 8) | buf[1];
    return status;
}

VL53LX_Error VL53LX_RdDWord(VL53LX_DEV Dev, uint16_t index, uint32_t *pdata) {
    uint8_t buf[4];
    VL53LX_Error status = VL53LX_ReadMulti(Dev, index, buf, 4);
    if (status == VL53LX_ERROR_NONE)
        *pdata = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
                 ((uint32_t)buf[2] << 8)  | buf[3];
    return status;
}

VL53LX_Error VL53LX_WaitUs(VL53LX_DEV Dev, int32_t wait_us) {
    (void)Dev;
    sleep_us((uint64_t)wait_us);
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WaitMs(VL53LX_DEV Dev, int32_t wait_ms) {
    (void)Dev;
    sleep_ms((uint32_t)wait_ms);
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GetTimerFrequency(int32_t *ptimer_freq_hz) {
    *ptimer_freq_hz = 1000000; /* Pico system timer ticks at 1 MHz */
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GetTimerValue(int32_t *ptimer_count) {
    *ptimer_count = (int32_t)time_us_32();
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GetTickCount(VL53LX_DEV Dev, uint32_t *ptick_count_ms) {
    (void)Dev;
    *ptick_count_ms = to_ms_since_boot(get_absolute_time());
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GpioSetMode(uint8_t pin, uint8_t mode) {
    (void)pin; (void)mode;
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GpioSetValue(uint8_t pin, uint8_t value) {
    (void)pin; (void)value;
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GpioGetValue(uint8_t pin, uint8_t *pvalue) {
    (void)pin;
    *pvalue = 0;
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GpioXshutdown(uint8_t value) {
    (void)value;
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GpioCommsSelect(uint8_t value) {
    (void)value;
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GpioPowerEnable(uint8_t value) {
    (void)value;
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GpioInterruptEnable(void (*function)(void),
                                        uint8_t edge_type) {
    (void)function; (void)edge_type;
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GpioInterruptDisable(void) {
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WaitValueMaskEx(VL53LX_DEV Dev, uint32_t timeout_ms,
                                    uint16_t index, uint8_t value,
                                    uint8_t mask, uint32_t poll_delay_ms) {
    uint32_t start_ms = to_ms_since_boot(get_absolute_time());
    uint8_t byte_value = 0;

    while ((to_ms_since_boot(get_absolute_time()) - start_ms) < timeout_ms) {
        VL53LX_Error status = VL53LX_RdByte(Dev, index, &byte_value);
        if (status != VL53LX_ERROR_NONE)
            return status;
        if ((byte_value & mask) == value)
            return VL53LX_ERROR_NONE;
        sleep_ms(poll_delay_ms);
    }

    return VL53LX_ERROR_TIME_OUT;
}
