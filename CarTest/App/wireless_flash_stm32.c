#include "wireless_flash_stm32.h"

#include <string.h>

#if defined(CARTEST_WIRELESS_ENABLED) && CARTEST_WIRELESS_ENABLED
#include "stm32h7xx_hal.h"

enum { FLASHWORD_BYTES = 32u };

static void fail_writer(wireless_flash_writer_t *writer, uint32_t error)
{
    writer->error_code = error != 0u ? error : 1u;
    writer->state = WIRELESS_FLASH_ERROR;
    writer->program_inflight = 0u;
    (void)HAL_FLASHEx_Lock_Bank2();
}

void wireless_flash_writer_init(wireless_flash_writer_t *writer)
{
    if (writer != NULL) (void)memset(writer, 0, sizeof(*writer));
}

uint8_t wireless_flash_load(wireless_settings_t *settings,
                            uint32_t *generation, uint8_t *selected_slot)
{
    return wireless_settings_select_slot(
        (const uint8_t *)(uintptr_t)WIRELESS_SETTINGS_FLASH_SLOT_A,
        (const uint8_t *)(uintptr_t)WIRELESS_SETTINGS_FLASH_SLOT_B,
        WIRELESS_SETTINGS_SLOT_BYTES, settings, generation, selected_slot);
}

uint8_t wireless_flash_begin_save(wireless_flash_writer_t *writer,
                                  const wireless_settings_t *settings,
                                  uint32_t generation, uint8_t target_slot)
{
    FLASH_EraseInitTypeDef erase = {0};
    if (writer == NULL || settings == NULL || target_slot > 1u ||
        writer->state == WIRELESS_FLASH_ERASING ||
        writer->state == WIRELESS_FLASH_PROGRAMMING ||
        writer->state == WIRELESS_FLASH_VERIFYING ||
        !wireless_settings_is_valid(settings)) return 0u;
    if (wireless_settings_encode_slot(settings, generation,
                                      (uint8_t *)writer->record_words,
                                      sizeof(writer->record_words)) == 0u) return 0u;
    writer->target_slot = target_slot;
    writer->target_address = target_slot == 0u
                                 ? WIRELESS_SETTINGS_FLASH_SLOT_A
                                 : WIRELESS_SETTINGS_FLASH_SLOT_B;
    writer->generation = generation;
    writer->program_offset = 0u;
    writer->program_inflight = 0u;
    writer->error_code = 0u;
    if (HAL_FLASHEx_Unlock_Bank2() != HAL_OK) {
        fail_writer(writer, HAL_FLASH_GetError());
        return 0u;
    }
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Banks = FLASH_BANK_2;
    erase.Sector = target_slot == 0u ? WIRELESS_SETTINGS_FLASH_SECTOR_A
                                    : WIRELESS_SETTINGS_FLASH_SECTOR_B;
    erase.NbSectors = 1u;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    if (HAL_FLASHEx_Erase_IT(&erase) != HAL_OK) {
        fail_writer(writer, HAL_FLASH_GetError());
        return 0u;
    }
    writer->state = WIRELESS_FLASH_ERASING;
    return 1u;
}

static uint8_t flash_has_error(void)
{
    return HAL_FLASH_GetError() != HAL_FLASH_ERROR_NONE ? 1u : 0u;
}

static uint8_t flash_bank2_busy(void)
{
    return (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY_BANK2) ||
            __HAL_FLASH_GET_FLAG(FLASH_FLAG_WBNE_BANK2) ||
            __HAL_FLASH_GET_FLAG(FLASH_FLAG_QW_BANK2) ||
            __HAL_FLASH_GET_FLAG(FLASH_FLAG_CRC_BUSY_BANK2)) ? 1u : 0u;
}

wireless_flash_state_t wireless_flash_service(wireless_flash_writer_t *writer)
{
    HAL_StatusTypeDef status;
    if (writer == NULL) return WIRELESS_FLASH_ERROR;
    if (writer->state != WIRELESS_FLASH_ERASING &&
        writer->state != WIRELESS_FLASH_PROGRAMMING &&
        writer->state != WIRELESS_FLASH_VERIFYING) return writer->state;

    /* The project deliberately does not enable a flash IRQ vector. Calling
       the HAL handler from the bounded main-loop service still completes the
       interrupt-mode state machine without a long blocking wait. */
    HAL_FLASH_IRQHandler();
    if (flash_bank2_busy()) return writer->state;
    if (flash_has_error()) {
        fail_writer(writer, HAL_FLASH_GetError());
        return writer->state;
    }

    if (writer->state == WIRELESS_FLASH_ERASING) {
        writer->state = WIRELESS_FLASH_PROGRAMMING;
        writer->program_inflight = 0u;
    }
    if (writer->state == WIRELESS_FLASH_PROGRAMMING) {
        if (writer->program_offset < WIRELESS_SETTINGS_SLOT_BYTES) {
            status = HAL_FLASH_Program_IT(
                FLASH_TYPEPROGRAM_FLASHWORD,
                writer->target_address + writer->program_offset,
                (uint32_t)(uintptr_t)((uint8_t *)writer->record_words +
                                      writer->program_offset));
            if (status != HAL_OK) {
                fail_writer(writer, HAL_FLASH_GetError());
                return writer->state;
            }
            writer->program_inflight = 1u;
            writer->program_offset += FLASHWORD_BYTES;
            return writer->state;
        }
        writer->state = WIRELESS_FLASH_VERIFYING;
    }
    if (writer->state == WIRELESS_FLASH_VERIFYING) {
        const volatile uint8_t *flash =
            (const volatile uint8_t *)(uintptr_t)writer->target_address;
        size_t index;
        for (index = 0u; index < sizeof(writer->record_words); ++index) {
            if (flash[index] != ((const uint8_t *)writer->record_words)[index]) {
                fail_writer(writer, 2u);
                return writer->state;
            }
        }
        (void)HAL_FLASHEx_Lock_Bank2();
        writer->program_inflight = 0u;
        writer->state = WIRELESS_FLASH_DONE;
    }
    return writer->state;
}

wireless_flash_state_t wireless_flash_state(const wireless_flash_writer_t *writer)
{
    return writer == NULL ? WIRELESS_FLASH_ERROR : writer->state;
}

uint8_t wireless_flash_busy(const wireless_flash_writer_t *writer)
{
    if (writer == NULL) return 0u;
    return writer->state == WIRELESS_FLASH_ERASING ||
           writer->state == WIRELESS_FLASH_PROGRAMMING ||
           writer->state == WIRELESS_FLASH_VERIFYING;
}

uint32_t wireless_flash_error(const wireless_flash_writer_t *writer)
{
    return writer == NULL ? 1u : writer->error_code;
}

uint8_t wireless_flash_take_done(wireless_flash_writer_t *writer)
{
    if (writer == NULL || writer->state != WIRELESS_FLASH_DONE) return 0u;
    writer->state = WIRELESS_FLASH_IDLE;
    return 1u;
}

#else

void wireless_flash_writer_init(wireless_flash_writer_t *writer)
{
    if (writer != NULL) (void)memset(writer, 0, sizeof(*writer));
}

uint8_t wireless_flash_load(wireless_settings_t *settings,
                            uint32_t *generation, uint8_t *selected_slot)
{
    (void)settings; (void)generation; (void)selected_slot;
    return 0u;
}

uint8_t wireless_flash_begin_save(wireless_flash_writer_t *writer,
                                  const wireless_settings_t *settings,
                                  uint32_t generation, uint8_t target_slot)
{
    (void)writer; (void)settings; (void)generation; (void)target_slot;
    return 0u;
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

#endif
