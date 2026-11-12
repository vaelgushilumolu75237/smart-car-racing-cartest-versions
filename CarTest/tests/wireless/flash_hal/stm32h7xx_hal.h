#ifndef CARTEST_HOST_FLASH_HAL_H
#define CARTEST_HOST_FLASH_HAL_H
#include <stdint.h>
typedef enum { HAL_OK=0, HAL_ERROR=1 } HAL_StatusTypeDef;
typedef struct { uint32_t TypeErase, Banks, Sector, NbSectors, VoltageRange; } FLASH_EraseInitTypeDef;
#define FLASH_TYPEERASE_SECTORS 1u
#define FLASH_BANK_2 2u
#define FLASH_VOLTAGE_RANGE_3 3u
#define FLASH_TYPEPROGRAM_FLASHWORD 4u
#define FLASH_HAL_ERROR_NONE 0u
#define HAL_FLASH_ERROR_NONE 0u
#define FLASH_FLAG_BSY_BANK2 1u
#define FLASH_FLAG_WBNE_BANK2 2u
#define FLASH_FLAG_QW_BANK2 4u
#define FLASH_FLAG_CRC_BUSY_BANK2 8u
#define __HAL_FLASH_GET_FLAG(flag) (0u)
HAL_StatusTypeDef HAL_FLASHEx_Lock_Bank2(void);
HAL_StatusTypeDef HAL_FLASHEx_Unlock_Bank2(void);
HAL_StatusTypeDef HAL_FLASHEx_Erase_IT(FLASH_EraseInitTypeDef *erase);
HAL_StatusTypeDef HAL_FLASH_Program_IT(uint32_t kind, uint32_t address, uint32_t data);
uint32_t HAL_FLASH_GetError(void);
void HAL_FLASH_IRQHandler(void);
#endif
