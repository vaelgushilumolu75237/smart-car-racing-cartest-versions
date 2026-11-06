#include "gyro350_config.h"
#include "icm42688_stm32.h"
#if CARTEST_GYRO350_ENABLED
#include "stm32h7xx_hal.h"

static SPI_HandleTypeDef spi;
static uint8_t opened;

/* Main context only, interrupts enabled. A bounded 1ms HAL timeout is used
   for each <=17-byte exchange. CS is released on every success/error path. */
static int exchange(uint8_t *tx, uint8_t *rx, uint16_t length)
{
    HAL_StatusTypeDef status;
    if (!opened || !length || length > 17u || __get_PRIMASK()) return 0;
    GPIOB->BSRR = (uint32_t)GPIO_PIN_12 << 16u;
    __DSB();
    status = HAL_SPI_TransmitReceive(&spi, tx, rx, length, 1u);
    GPIOB->BSRR = GPIO_PIN_12;
    __DSB();
    return status == HAL_OK ? 1 : 0;
}

static int read_register(void *context, uint8_t reg, uint8_t *out, uint16_t length)
{
    uint8_t tx[17] = {0}, rx[17] = {0};
    uint16_t index;
    (void)context;
    if (!out || !length || length > 16u) return 0;
    tx[0] = reg | 0x80u;
    if (!exchange(tx, rx, length + 1u)) return 0;
    for (index = 0; index < length; ++index) out[index] = rx[index + 1u];
    return 1;
}

static int write_register(void *context, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {(uint8_t)(reg & 0x7fu), value}, rx[2];
    (void)context;
    return exchange(tx, rx, 2u);
}

int car_imu_spi2_open(car_imu_bus_t *bus)
{
    GPIO_InitTypeDef gpio = {0};
    RCC_PeriphCLKInitTypeDef clock = {0};
    if (!bus) return 0;
    /* Explicit 64MHz HSI -> CKPER -> SPI123; do not rely on an unused PLLQ. */
    clock.PeriphClockSelection = RCC_PERIPHCLK_SPI2 | RCC_PERIPHCLK_CKPER;
    clock.CkperClockSelection = RCC_CLKPSOURCE_HSI;
    clock.Spi123ClockSelection = RCC_SPI123CLKSOURCE_CLKP;
    if (HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK) return 0;
    if (HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI2) != 64000000u) return 0;
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIOB->BSRR = GPIO_PIN_12;
    gpio.Pin = GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &gpio);
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_SPI2_FORCE_RESET();
    __HAL_RCC_SPI2_RELEASE_RESET();
    spi.Instance = SPI2;
    spi.Init.Mode = SPI_MODE_MASTER;
    spi.Init.Direction = SPI_DIRECTION_2LINES;
    spi.Init.DataSize = SPI_DATASIZE_8BIT;
    spi.Init.CLKPolarity = SPI_POLARITY_LOW;
    spi.Init.CLKPhase = SPI_PHASE_1EDGE;
    spi.Init.NSS = SPI_NSS_SOFT;
    spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64; /* 1MHz */
    spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    spi.Init.TIMode = SPI_TIMODE_DISABLE;
    spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spi.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    spi.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
    spi.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    spi.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
    spi.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    spi.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    spi.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
    spi.Init.IOSwap = SPI_IO_SWAP_DISABLE;
    opened = HAL_SPI_Init(&spi) == HAL_OK;
    if (!opened) return 0;
    *bus = (car_imu_bus_t){0, read_register, write_register};
    return 1;
}
#else
int car_imu_spi2_open(car_imu_bus_t *bus) { (void)bus; return 0; }
#endif
