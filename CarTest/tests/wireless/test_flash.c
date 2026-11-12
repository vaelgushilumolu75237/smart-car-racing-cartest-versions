/* Execute the production STM32 flash state machine against mapped host RAM.
   No probes, serial ports, or physical target are accessed. */
#include <windows.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "wireless_flash_stm32.h"
#include "stm32h7xx_hal.h"

enum { SECTOR_BYTES=128*1024, FLASH_BYTES=2*SECTOR_BYTES };
static uint8_t original[FLASH_BYTES];
static unsigned writes, erase_count, fail_at;
static uint32_t error_code;
static wireless_flash_writer_t *writer;

HAL_StatusTypeDef HAL_FLASHEx_Lock_Bank2(void) { return HAL_OK; }
HAL_StatusTypeDef HAL_FLASHEx_Unlock_Bank2(void) { error_code=0; return HAL_OK; }
uint32_t HAL_FLASH_GetError(void) { return error_code; }
void HAL_FLASH_IRQHandler(void) {}
HAL_StatusTypeDef HAL_FLASHEx_Erase_IT(FLASH_EraseInitTypeDef *erase)
{
    assert(erase->Banks==FLASH_BANK_2 && erase->NbSectors==1);
    assert(erase->Sector==6 || erase->Sector==7);
    memset((void *)(uintptr_t)(erase->Sector==6 ? WIRELESS_SETTINGS_FLASH_SLOT_A :
                             WIRELESS_SETTINGS_FLASH_SLOT_B), 0xff, SECTOR_BYTES);
    ++erase_count;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FLASH_Program_IT(uint32_t kind, uint32_t address, uint32_t data)
{
    unsigned i;
    uint8_t *dst=(uint8_t *)(uintptr_t)address;
    const uint8_t *src=(const uint8_t *)(uintptr_t)data;
    assert(kind==FLASH_TYPEPROGRAM_FLASHWORD && !(address & 31u));
    assert(address>=WIRELESS_SETTINGS_FLASH_SLOT_A &&
           address+32<=WIRELESS_SETTINGS_FLASH_SLOT_B+SECTOR_BYTES);
    ++writes;
    if (fail_at && writes==fail_at) { error_code=77u; return HAL_ERROR; }
    for (i=0;i<32;++i) { assert(dst[i]==0xff); dst[i]=src[i]; }
    return HAL_OK;
}
static uint8_t *flash(void) { return (uint8_t *)(uintptr_t)WIRELESS_SETTINGS_FLASH_SLOT_A; }
static void reset_writer(void)
{
    wireless_flash_writer_init(writer);
    error_code=writes=fail_at=0;
}
static void finish(void)
{
    unsigned limit=100;
    while(wireless_flash_busy(writer) && limit--) wireless_flash_service(writer);
    assert(limit && writer->state==WIRELESS_FLASH_DONE);
    assert(writes==WIRELESS_SETTINGS_FLASH_TABLE_BYTES/32);
}
static wireless_settings_t settings(unsigned key)
{
    wireless_settings_t value;
    wireless_settings_defaults_for_profile(&value,(uint8_t)key);
    value.wheel_scale_permille=(uint16_t)(950-key*50);
    return value;
}
static void expect_all(unsigned changed, int new_value_allowed)
{
    unsigned key;
    for(key=0;key<6;++key) {
        wireless_settings_t loaded;
        uint32_t generation;
        uint8_t slot;
        assert(wireless_flash_load_profile((uint8_t)key,&loaded,&generation,&slot));
        assert(generation==6 || (new_value_allowed && generation==7));
        assert(loaded.wheel_scale_permille==settings(key).wheel_scale_permille ||
               (new_value_allowed && key==changed && loaded.wheel_scale_permille==333));
        if(key!=changed) assert(loaded.wheel_scale_permille==settings(key).wheel_scale_permille);
    }
}
int main(void)
{
    unsigned key, step;
    wireless_settings_t value, loaded;
    uint32_t generation;
    uint8_t slot;
    assert(VirtualAlloc((void *)(uintptr_t)WIRELESS_SETTINGS_FLASH_SLOT_A,
           FLASH_BYTES,MEM_RESERVE|MEM_COMMIT,PAGE_READWRITE)==flash());
    writer=VirtualAlloc((void *)(uintptr_t)0x09000000u,65536,
                        MEM_RESERVE|MEM_COMMIT,PAGE_READWRITE);
    assert(writer==(void *)(uintptr_t)0x09000000u);
    memset(flash(),0xff,FLASH_BYTES);
    /* No commit marker: even a valid individual record is not accepted. */
    value=settings(0);
    assert(wireless_settings_encode_slot(&value,9,flash(),128));
    assert(!wireless_flash_load_profile(0,&loaded,&generation,&slot));
    memset(flash(),0xff,FLASH_BYTES);
    for(key=0;key<6;++key) {
        value=settings(key);
        reset_writer();
        assert(wireless_flash_begin_save_profile(writer,&value,999,(uint8_t)key,0));
        finish();
        assert(writer->generation==key+1);
    }
    expect_all(0,0);
    memcpy(original,flash(),FLASH_BYTES);
    /* Every possible interruption: old complete table or new complete table.
       No unrelated profile can revert when the alternate sector is erased. */
    for(step=0;step<=WIRELESS_SETTINGS_FLASH_TABLE_BYTES/32+1;++step) {
        unsigned n;
        memcpy(flash(),original,FLASH_BYTES);
        reset_writer(); value=settings(2); value.wheel_scale_permille=333;
        assert(wireless_flash_begin_save_profile(writer,&value,7,2,0));
        for(n=0;n<step;++n) wireless_flash_service(writer);
        expect_all(2,1);
    }
    for(step=1;step<=WIRELESS_SETTINGS_FLASH_TABLE_BYTES/32;++step) {
        unsigned limit=100;
        memcpy(flash(),original,FLASH_BYTES);
        reset_writer(); fail_at=step;
        value=settings(4); value.wheel_scale_permille=333;
        assert(wireless_flash_begin_save_profile(writer,&value,7,4,0));
        while(wireless_flash_busy(writer) && limit--) wireless_flash_service(writer);
        assert(limit && writer->state==WIRELESS_FLASH_ERROR);
        expect_all(4,0);
    }
    /* Torn/corrupt final marker cannot publish the new table. */
    memcpy(flash(),original,FLASH_BYTES); reset_writer();
    value=settings(1); value.wheel_scale_permille=333;
    assert(wireless_flash_begin_save_profile(writer,&value,7,1,0)); finish();
    ((uint8_t *)(uintptr_t)writer->target_address)[WIRELESS_SETTINGS_FLASH_RECORD_BYTES+24]^=1;
    expect_all(1,0);
    { unsigned before=erase_count;
      reset_writer(); value=settings(0); value.wheel_scale_permille=2000;
      assert(!wireless_flash_begin_save_profile(writer,&value,8,0,0));
      assert(erase_count==before); }
    puts("wireless production flash: six profiles, all interruption points, 25 write failures, CRC fallback PASS");
    return 0;
}
