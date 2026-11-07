#ifndef CARTEST_WIRELESS_FLASH_STM32_H
#define CARTEST_WIRELESS_FLASH_STM32_H

#include <stdint.h>

#include "wireless_settings.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    WIRELESS_SETTINGS_FLASH_SLOT_A = 0x081c0000u,
    WIRELESS_SETTINGS_FLASH_SLOT_B = 0x081e0000u,
    WIRELESS_SETTINGS_FLASH_SECTOR_A = 6u,
    WIRELESS_SETTINGS_FLASH_SECTOR_B = 7u
};

typedef enum {
    WIRELESS_FLASH_IDLE = 0u,
    WIRELESS_FLASH_ERASING = 1u,
    WIRELESS_FLASH_PROGRAMMING = 2u,
    WIRELESS_FLASH_VERIFYING = 3u,
    WIRELESS_FLASH_DONE = 4u,
    WIRELESS_FLASH_ERROR = 5u
} wireless_flash_state_t;

typedef struct {
    wireless_flash_state_t state;
    uint32_t target_address;
    uint32_t generation;
    uint32_t program_offset;
    uint8_t target_slot;
    uint8_t program_inflight;
    uint32_t record_words[WIRELESS_SETTINGS_SLOT_BYTES / sizeof(uint32_t)];
    uint32_t error_code;
} wireless_flash_writer_t;

void wireless_flash_writer_init(wireless_flash_writer_t *writer);
/* Read and validate both reserved Bank-2 slots without changing flash. */
uint8_t wireless_flash_load(wireless_settings_t *settings,
                            uint32_t *generation,
                            uint8_t *selected_slot);
/* Starts an interrupt-driven erase. The function never waits for flash and
   returns 0 when the writer is busy, the settings are invalid, or the target
   slot is not 0/1. */
uint8_t wireless_flash_begin_save(wireless_flash_writer_t *writer,
                                  const wireless_settings_t *settings,
                                  uint32_t generation,
                                  uint8_t target_slot);
/* Call from the main loop. It advances at most one flash word per call and
   services the HAL interrupt state; it never spins or disables interrupts. */
wireless_flash_state_t wireless_flash_service(wireless_flash_writer_t *writer);
wireless_flash_state_t wireless_flash_state(const wireless_flash_writer_t *writer);
uint8_t wireless_flash_busy(const wireless_flash_writer_t *writer);
uint32_t wireless_flash_error(const wireless_flash_writer_t *writer);
uint8_t wireless_flash_take_done(wireless_flash_writer_t *writer);

#ifdef __cplusplus
}
#endif

#endif /* CARTEST_WIRELESS_FLASH_STM32_H */
