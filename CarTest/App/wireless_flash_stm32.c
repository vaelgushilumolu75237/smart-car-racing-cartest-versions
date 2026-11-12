#include "wireless_flash_stm32.h"

#include <string.h>

#if defined(CARTEST_WIRELESS_ENABLED) && CARTEST_WIRELESS_ENABLED
#include "stm32h7xx_hal.h"

enum {
    FLASHWORD_BYTES = 32u,
    TABLE_MAGIC = 0x31425457u, /* WTB1 */
    TABLE_SCHEMA = 1u,
    TABLE_GENERATION_OFFSET = 8u,
    TABLE_RECORD_BYTES_OFFSET = 12u,
    TABLE_PROFILE_COUNT_OFFSET = 16u,
    TABLE_RECORD_CRC_OFFSET = 20u,
    TABLE_HEADER_CRC_OFFSET = 24u
};

static uint32_t get_u32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8u) |
           ((uint32_t)data[2] << 16u) | ((uint32_t)data[3] << 24u);
}

static void put_u32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8u);
    data[2] = (uint8_t)(value >> 16u);
    data[3] = (uint8_t)(value >> 24u);
}

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

static uint32_t flash_sector_base(uint8_t slot)
{
    return slot == 0u ? WIRELESS_SETTINGS_FLASH_SLOT_A :
                        WIRELESS_SETTINGS_FLASH_SLOT_B;
}

static void read_sector_bytes(uint8_t slot, uint8_t *destination, size_t length)
{
    const volatile uint8_t *source =
        (const volatile uint8_t *)(uintptr_t)flash_sector_base(slot);
    size_t index;
    for (index = 0u; index < length; ++index) destination[index] = source[index];
}

static uint8_t table_image_valid(const uint8_t *image, uint32_t *generation)
{
    const uint8_t *header = image + WIRELESS_SETTINGS_FLASH_RECORD_BYTES;
    if (get_u32(header) != TABLE_MAGIC || get_u32(header + 4u) != TABLE_SCHEMA ||
        get_u32(header + TABLE_RECORD_BYTES_OFFSET) !=
            WIRELESS_SETTINGS_FLASH_RECORD_BYTES ||
        get_u32(header + TABLE_PROFILE_COUNT_OFFSET) !=
            WIRELESS_SETTINGS_PROFILE_COUNT ||
        get_u32(header + TABLE_RECORD_CRC_OFFSET) !=
            wireless_settings_crc32(image, WIRELESS_SETTINGS_FLASH_RECORD_BYTES) ||
        get_u32(header + TABLE_HEADER_CRC_OFFSET) !=
            wireless_settings_crc32(header, TABLE_HEADER_CRC_OFFSET)) return 0u;
    if (generation != NULL) *generation = get_u32(header + TABLE_GENERATION_OFFSET);
    return 1u;
}

static uint8_t read_table_image(uint8_t slot, uint8_t *image, uint32_t *generation)
{
    read_sector_bytes(slot, image, WIRELESS_SETTINGS_FLASH_TABLE_BYTES);
    return table_image_valid(image, generation);
}

static void build_table_header(uint8_t *image, uint32_t generation)
{
    uint8_t *header = image + WIRELESS_SETTINGS_FLASH_RECORD_BYTES;
    (void)memset(header, 0xff, WIRELESS_SETTINGS_FLASH_TABLE_HEADER_BYTES);
    put_u32(header, TABLE_MAGIC);
    put_u32(header + 4u, TABLE_SCHEMA);
    put_u32(header + TABLE_GENERATION_OFFSET, generation);
    put_u32(header + TABLE_RECORD_BYTES_OFFSET,
            WIRELESS_SETTINGS_FLASH_RECORD_BYTES);
    put_u32(header + TABLE_PROFILE_COUNT_OFFSET,
            WIRELESS_SETTINGS_PROFILE_COUNT);
    put_u32(header + TABLE_RECORD_CRC_OFFSET,
            wireless_settings_crc32(image, WIRELESS_SETTINGS_FLASH_RECORD_BYTES));
    put_u32(header + TABLE_HEADER_CRC_OFFSET,
            wireless_settings_crc32(header, TABLE_HEADER_CRC_OFFSET));
}

static uint32_t next_generation(uint32_t generation)
{
    ++generation;
    return generation != 0u ? generation : 1u;
}

static uint8_t choose_active_table(uint8_t *image, uint8_t *slot,
                                   uint32_t *generation)
{
    uint8_t a_image[WIRELESS_SETTINGS_FLASH_TABLE_BYTES];
    uint8_t b_image[WIRELESS_SETTINGS_FLASH_TABLE_BYTES];
    uint32_t a_generation = 0u, b_generation = 0u;
    uint8_t valid_a = read_table_image(0u, a_image, &a_generation);
    uint8_t valid_b = read_table_image(1u, b_image, &b_generation);
    if (!valid_a && !valid_b) return 0u;
    if (valid_b && (!valid_a ||
                    wireless_settings_generation_is_newer(b_generation,
                                                          a_generation))) {
        memcpy(image, b_image, sizeof(b_image));
        if (slot != NULL) *slot = 1u;
        if (generation != NULL) *generation = b_generation;
    } else {
        memcpy(image, a_image, sizeof(a_image));
        if (slot != NULL) *slot = 0u;
        if (generation != NULL) *generation = a_generation;
    }
    return 1u;
}

uint8_t wireless_flash_load(wireless_settings_t *settings,
                            uint32_t *generation, uint8_t *selected_slot)
{
    return wireless_flash_load_profile(0u, settings, generation, selected_slot);
}

uint8_t wireless_flash_load_profile(uint8_t profile_key,
                                    wireless_settings_t *settings,
                                    uint32_t *generation,
                                    uint8_t *selected_slot)
{
    uint8_t image[WIRELESS_SETTINGS_FLASH_TABLE_BYTES];
    uint8_t slot;
    uint32_t table_generation, row_generation;
    if (profile_key >= WIRELESS_SETTINGS_PROFILE_COUNT || settings == NULL ||
        generation == NULL) return 0u;
    if (choose_active_table(image, &slot, &table_generation)) {
        if (selected_slot != NULL) *selected_slot = slot;
        if (!wireless_settings_decode_slot(
            image + (size_t)profile_key * WIRELESS_SETTINGS_SLOT_BYTES,
            WIRELESS_SETTINGS_SLOT_BYTES, settings, &row_generation)) return 0u;
        *generation = table_generation;
        return 1u;
    }
    /* A partially programmed table is never an old valid per-profile slot.
       This OpenMV extension has no earlier persistent layout to migrate. */
    return 0u;
}

uint8_t wireless_flash_begin_save(wireless_flash_writer_t *writer,
                                  const wireless_settings_t *settings,
                                  uint32_t generation, uint8_t target_slot)
{
    return wireless_flash_begin_save_profile(writer, settings, generation,
                                             0u, target_slot);
}

uint8_t wireless_flash_begin_save_profile(wireless_flash_writer_t *writer,
                                          const wireless_settings_t *settings,
                                          uint32_t generation,
                                          uint8_t profile_key,
                                          uint8_t target_slot)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint8_t active_slot = 0u;
    uint8_t active_image[WIRELESS_SETTINGS_FLASH_TABLE_BYTES];
    uint32_t active_generation = 0u;
    uint32_t table_generation;
    uint8_t have_active;
    uint8_t profile;
    (void)generation;
    if (writer == NULL || settings == NULL || target_slot > 1u ||
        profile_key >= WIRELESS_SETTINGS_PROFILE_COUNT ||
        writer->state == WIRELESS_FLASH_ERASING ||
        writer->state == WIRELESS_FLASH_PROGRAMMING ||
        writer->state == WIRELESS_FLASH_VERIFYING ||
        !wireless_settings_is_valid(settings)) return 0u;

    have_active = choose_active_table(active_image, &active_slot,
                                      &active_generation);
    if (have_active != 0u) {
        memcpy(writer->record_words, active_image, sizeof(active_image));
        /* Never erase the only committed table, even if the caller's hint is
           stale after a reconnect. */
        target_slot = (uint8_t)(active_slot ^ 1u);
        table_generation = next_generation(active_generation);
    } else {
        memset(writer->record_words, 0xff, sizeof(writer->record_words));
        for (profile = 0u; profile < WIRELESS_SETTINGS_PROFILE_COUNT; ++profile) {
            wireless_settings_t defaults;
            wireless_settings_defaults_for_profile(&defaults, profile);
            if (!wireless_settings_encode_slot(&defaults, 0u,
                (uint8_t *)writer->record_words +
                    (size_t)profile * WIRELESS_SETTINGS_SLOT_BYTES,
                WIRELESS_SETTINGS_SLOT_BYTES)) return 0u;
        }
        table_generation = 1u;
    }
    if (wireless_settings_encode_slot(settings, table_generation,
             (uint8_t *)writer->record_words +
                 (size_t)profile_key * WIRELESS_SETTINGS_SLOT_BYTES,
             WIRELESS_SETTINGS_SLOT_BYTES) == 0u) return 0u;
    build_table_header((uint8_t *)writer->record_words, table_generation);
    writer->target_slot = target_slot;
    writer->target_address = flash_sector_base(target_slot);
    writer->generation = table_generation;
    writer->table_generation = table_generation;
    writer->profile_key = profile_key;
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
       the HAL handler from the bounded main-loop service still completes
       the interrupt-mode state machine without a long blocking wait. */
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
        if (writer->program_offset < WIRELESS_SETTINGS_FLASH_TABLE_BYTES) {
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
        for (index = 0u; index < WIRELESS_SETTINGS_FLASH_TABLE_BYTES; ++index) {
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

uint8_t wireless_flash_load_profile(uint8_t profile_key,
                                    wireless_settings_t *settings,
                                    uint32_t *generation,
                                    uint8_t *selected_slot)
{
    (void)profile_key; (void)settings; (void)generation; (void)selected_slot;
    return 0u;
}

uint8_t wireless_flash_begin_save(wireless_flash_writer_t *writer,
                                  const wireless_settings_t *settings,
                                  uint32_t generation, uint8_t target_slot)
{
    (void)writer; (void)settings; (void)generation; (void)target_slot;
    return 0u;
}

uint8_t wireless_flash_begin_save_profile(wireless_flash_writer_t *writer,
                                          const wireless_settings_t *settings,
                                          uint32_t generation,
                                          uint8_t profile_key,
                                          uint8_t target_slot)
{
    (void)writer; (void)settings; (void)generation;
    (void)profile_key; (void)target_slot;
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
