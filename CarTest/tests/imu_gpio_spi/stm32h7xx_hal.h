#ifndef TEST_IMU_HAL_H
#define TEST_IMU_HAL_H
#include <stdint.h>
typedef struct { uint32_t BSRR, IDR; } GPIO_TypeDef;
typedef struct { uint32_t DEMCR; } CoreDebug_Type;
typedef struct { uint32_t CTRL, CYCCNT; } DWT_Type;
typedef struct { uint32_t Pin, Mode, Pull, Speed; } GPIO_InitTypeDef;
extern GPIO_TypeDef test_gpio;
extern CoreDebug_Type test_debug;
extern DWT_Type test_dwt;
#define GPIOB (&test_gpio)
#define CoreDebug (&test_debug)
#define DWT (&test_dwt)
#define GPIO_PIN_12 (1u << 12)
#define GPIO_PIN_13 (1u << 13)
#define GPIO_PIN_14 (1u << 14)
#define GPIO_PIN_15 (1u << 15)
#define GPIO_MODE_OUTPUT_PP 1u
#define GPIO_MODE_INPUT 0u
#define GPIO_NOPULL 0u
#define GPIO_SPEED_FREQ_HIGH 3u
#define CoreDebug_DEMCR_TRCENA_Msk (1u << 24)
#define DWT_CTRL_CYCCNTENA_Msk 1u
uint32_t HAL_RCC_GetHCLKFreq(void);
uint32_t __get_PRIMASK(void);
void HAL_GPIO_Init(GPIO_TypeDef *, GPIO_InitTypeDef *);
void test_clock_enable(void);
#define __HAL_RCC_GPIOB_CLK_ENABLE() test_clock_enable()
#define __DSB() ((void)0)
#define __NOP() ((void)0)
#endif
