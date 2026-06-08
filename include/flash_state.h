#ifndef FLASH_STATE_H
#define FLASH_STATE_H

#include <stdint.h>
#include <stdbool.h>

#define FLASH_STATE_MAGIC 0x4E455032   // "NEP2"

typedef struct {
    uint32_t magic;         
    uint8_t  last_state;     
    uint8_t  chute_deployed; // 1 if parachute was fired
} FlashState;

void flash_state_write(uint8_t state, uint8_t chute_deployed);
bool flash_state_read(FlashState *out);
void flash_state_clear(void);

#endif
