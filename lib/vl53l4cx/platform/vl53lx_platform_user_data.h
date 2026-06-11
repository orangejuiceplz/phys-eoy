/*
 * Pico SDK port of the ST VL53L4CX bare driver device handle.
 * Replaces the X-CUBE-TOF1 BSP-coupled version.
 */

#ifndef _VL53LX_PLATFORM_USER_DATA_H_
#define _VL53LX_PLATFORM_USER_DATA_H_

#include <stdlib.h>

#include "vl53lx_def.h"
#include "hardware/i2c.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct {
    VL53LX_DevData_t Data;       /* ST driver state — must stay first */
    i2c_inst_t      *i2c;        /* Pico I2C instance */
    uint8_t          i2c_addr;   /* 7-bit device address */
} VL53LX_Dev_t;

typedef VL53LX_Dev_t *VL53LX_DEV;

#define VL53LXDevDataGet(Dev, field)           (Dev->Data.field)
#define VL53LXDevDataSet(Dev, field, data)     ((Dev->Data.field) = (data))
#define PALDevDataGet(Dev, field)              (Dev->Data.field)
#define PALDevDataSet(Dev, field, value)       (Dev->Data.field) = (value)
#define VL53LXDevStructGetLLDriverHandle(Dev)  (&Dev->Data.LLData)
#define VL53LXDevStructGetLLResultsHandle(Dev) (&Dev->Data.llresults)

#ifdef __cplusplus
}
#endif

#endif
