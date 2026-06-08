#include "../include/flash_state.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include <string.h>

// Use last sector of 2MB flash
#define FLASH_STATE_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

void flash_state_write(uint8_t state, uint8_t chute_deployed) {
    FlashState fs = {
        .magic = FLASH_STATE_MAGIC,
        .last_state = state,
        .chute_deployed = chute_deployed
    };

    uint8_t buf[FLASH_PAGE_SIZE];
    memset(buf, 0xFF, sizeof(buf));
    memcpy(buf, &fs, sizeof(fs));

    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_STATE_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_STATE_OFFSET, buf, FLASH_PAGE_SIZE);
    restore_interrupts(ints);
}

bool flash_state_read(FlashState *out) {
    const uint8_t *flash_ptr = (const uint8_t *)(XIP_BASE + FLASH_STATE_OFFSET);
    memcpy(out, flash_ptr, sizeof(FlashState));
    return out->magic == FLASH_STATE_MAGIC;
}

void flash_state_clear(void) {
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_STATE_OFFSET, FLASH_SECTOR_SIZE);
    restore_interrupts(ints);
}
