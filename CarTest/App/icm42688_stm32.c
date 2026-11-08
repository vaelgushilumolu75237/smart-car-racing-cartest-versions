#include "gyro350_config.h"
#include "icm42688_stm32.h"
#if CARTEST_GYRO350_ENABLED
#include "stm32h7xx_hal.h"

/* Existing SPI2 connector: PB12 CS, PB13 SCK, PB14 MISO, PB15 MOSI.
   An independent GPIO mode-0 read returned the real WHO_AM_I=0x47 on the
   user's vehicle while the old HAL SPI2 path returned zero. Use that mode-0
   path with a cycle-timed clock and an absolute transaction deadline.
   No interrupt masking, heap, DMA, or timer peripheral is needed. */
#ifndef CAR_IMU_SPI_WRITE
#define CAR_IMU_SPI_WRITE(value) (GPIOB->BSRR = (value))
#endif
#ifndef CAR_IMU_SPI_READ
#define CAR_IMU_SPI_READ() (GPIOB->IDR)
#endif
#ifndef CAR_IMU_SPI_CYCLES
#define CAR_IMU_SPI_CYCLES() (DWT->CYCCNT)
#endif

enum {
    CS = GPIO_PIN_12,
    SCK = GPIO_PIN_13,
    MISO = GPIO_PIN_14,
    MOSI = GPIO_PIN_15,
    CLOCK_HZ = 64000000u,
    HALF_PERIOD_CYCLES = 128u, /* >=2us at verified 64MHz: SCK <=250kHz. */
    TRANSACTION_BUDGET_CYCLES = 64000u, /* 1ms including interrupt time. */
    MAX_DELAY_SPINS = 4096u
};

static uint8_t opened;

static void idle_bus(void)
{
    CAR_IMU_SPI_WRITE((uint32_t)(SCK | MOSI) << 16u | CS);
    __DSB();
}

static int wait_half_period(uint32_t transaction_start)
{
    uint32_t start = CAR_IMU_SPI_CYCLES();
    uint32_t spins;
    for (spins = 0u; spins < MAX_DELAY_SPINS; ++spins) {
        uint32_t now = CAR_IMU_SPI_CYCLES();
        if ((uint32_t)(now - transaction_start) >= TRANSACTION_BUDGET_CYCLES)
            return 0;
        if ((uint32_t)(now - start) >= HALF_PERIOD_CYCLES) return 1;
        __NOP();
    }
    /* A stopped/locked DWT cannot turn a register read into an endless loop. */
    return 0;
}

/* Main context only. Interrupts stay enabled. Failed exchanges never expose
   partial register data, and every transaction releases CS and returns SCK low. */
static int exchange(const uint8_t *tx, uint8_t *rx, uint16_t length)
{
    uint16_t index;
    uint32_t start;
    if (!opened || !tx || !rx || !length || length > 17u || __get_PRIMASK())
        return 0;
    start = CAR_IMU_SPI_CYCLES();
    idle_bus();
    if (!wait_half_period(start)) goto failed;
    CAR_IMU_SPI_WRITE((uint32_t)CS << 16u);
    if (!wait_half_period(start)) goto failed;
    for (index = 0u; index < length; ++index) {
        uint8_t received = 0u;
        uint8_t bit;
        for (bit = 0u; bit < 8u; ++bit) {
            CAR_IMU_SPI_WRITE((tx[index] & (0x80u >> bit)) != 0u
                ? MOSI : (uint32_t)MOSI << 16u);
            if (!wait_half_period(start)) goto failed;
            CAR_IMU_SPI_WRITE(SCK);
            if (!wait_half_period(start)) goto failed;
            received = (uint8_t)((received << 1u) |
                       ((CAR_IMU_SPI_READ() & MISO) != 0u ? 1u : 0u));
            CAR_IMU_SPI_WRITE((uint32_t)SCK << 16u);
        }
        rx[index] = received;
    }
    if (!wait_half_period(start)) goto failed;
    idle_bus();
    return 1;
failed:
    idle_bus();
    return 0;
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
    uint32_t before;
    uint32_t attempts;
    opened = 0u;
    if (!bus) return 0;
    *bus = (car_imu_bus_t){0};
    if (__get_PRIMASK() || HAL_RCC_GetHCLKFreq() != CLOCK_HZ) return 0;
    __HAL_RCC_GPIOB_CLK_ENABLE();
    idle_bus(); /* Set the output latch before enabling the output drivers. */
    gpio.Pin = CS | SCK | MOSI;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = MISO;
    gpio.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(GPIOB, &gpio);

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    __DSB();
    before = CAR_IMU_SPI_CYCLES();
    for (attempts = 0u; attempts < MAX_DELAY_SPINS; ++attempts) {
        __NOP();
        if (CAR_IMU_SPI_CYCLES() != before) {
            opened = 1u;
            *bus = (car_imu_bus_t){0, read_register, write_register};
            return 1;
        }
    }
    idle_bus();
    return 0;
}
#else
int car_imu_spi2_open(car_imu_bus_t *bus) { (void)bus; return 0; }
#endif
