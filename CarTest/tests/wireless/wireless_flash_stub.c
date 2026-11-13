#include "wireless_flash_stm32.h"

#include <string.h>

void wireless_flash_writer_init(wireless_flash_writer_t *writer)
{
    if (writer != NULL) (void)memset(writer, 0, sizeof(*writer));
}

uint8_t wireless_flash_load_profile(uint8_t profile_key,
                                    wireless_settings_t *settings,
                                    uint32_t *generation,
                                    uint8_t *selected_slot)
{
    (void)profile_key;
    (void)settings;
    (void)generation;
    (void)selected_slot;
    return 0u;
}

uint8_t wireless_flash_load(wireless_settings_t *settings,
                            uint32_t *generation,
                            uint8_t *selected_slot)
{
    return wireless_flash_load_profile(0u, settings, generation, selected_slot);
}

uint8_t wireless_flash_begin_save_profile(wireless_flash_writer_t *writer,
                                           const wireless_settings_t *settings,
                                           uint32_t generation,
                                           uint8_t profile_key,
                                           uint8_t target_slot)
{
    (void)writer;
    (void)settings;
    (void)generation;
    (void)profile_key;
    (void)target_slot;
    return 0u;
}

uint8_t wireless_flash_begin_save(wireless_flash_writer_t *writer,
                                  const wireless_settings_t *settings,
                                  uint32_t generation,
                                  uint8_t target_slot)
{
    return wireless_flash_begin_save_profile(writer, settings, generation,
                                             0u, target_slot);
}

wireless_flash_state_t wireless_flash_service(wireless_flash_writer_t *writer)
{
    return writer == NULL ? WIRELESS_FLASH_ERROR : writer->state;
}

wireless_flash_state_t wireless_flash_state(const wireless_flash_writer_t *writer)
{
    return writer == NULL ? WIRELESS_FLASH_ERROR : writer->state;
}

uint8_t wireless_flash_busy(const wireless_flash_writer_t *writer)
{
    (void)writer;
    return 0u;
}

uint32_t wireless_flash_error(const wireless_flash_writer_t *writer)
{
    return writer == NULL ? 1u : writer->error_code;
}

uint8_t wireless_flash_take_done(wireless_flash_writer_t *writer)
{
    (void)writer;
    return 0u;
}
