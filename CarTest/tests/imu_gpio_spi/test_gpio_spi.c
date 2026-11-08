#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32h7xx_hal.h"

GPIO_TypeDef test_gpio;
CoreDebug_Type test_debug;
DWT_Type test_dwt;
static uint32_t sim_cycles(void);
static void sim_write(uint32_t);
static uint32_t sim_read(void);
#define CAR_IMU_SPI_CYCLES() sim_cycles()
#define CAR_IMU_SPI_WRITE(v) sim_write(v)
#define CAR_IMU_SPI_READ() sim_read()
#ifndef CARTEST_TEST_DRIVER
#define CARTEST_TEST_DRIVER "App/icm42688_stm32.c"
#endif
#include CARTEST_TEST_DRIVER

static unsigned assertions;
#define CHECK(v) do { ++assertions; if (!(v)) { \
    fprintf(stderr,"line %d: %s\n",__LINE__,#v); exit(1); } } while(0)
static struct {
    uint32_t now, hclk, primask, odr, last_rise, last_fall, selected_at;
    unsigned cycle_calls, freeze_at, jump_at, jump_cycles, gpio_calls, clocks, samples;
    unsigned clock_enabled, selected, completed, bit_count, reads;
    unsigned char tx[17], regs[128];
} sim;

uint32_t HAL_RCC_GetHCLKFreq(void) { return sim.hclk; }
uint32_t __get_PRIMASK(void) { return sim.primask; }
void test_clock_enable(void) { ++sim.clock_enabled; }
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *gpio)
{
    CHECK(port==GPIOB && gpio->Pull==GPIO_NOPULL);
    if (sim.gpio_calls==0) {
        CHECK(gpio->Pin==(CS|SCK|MOSI));
        CHECK(gpio->Mode==GPIO_MODE_OUTPUT_PP);
        CHECK((sim.odr&(CS|SCK|MOSI))==CS);
    } else {
        CHECK(sim.gpio_calls==1 && gpio->Pin==MISO);
        CHECK(gpio->Mode==GPIO_MODE_INPUT);
    }
    ++sim.gpio_calls;
}
static uint32_t sim_cycles(void)
{
    ++sim.cycle_calls;
    if (!sim.freeze_at || sim.cycle_calls<sim.freeze_at) sim.now+=32u;
    if (sim.jump_at && sim.cycle_calls==sim.jump_at) sim.now+=sim.jump_cycles;
    return sim.now;
}
static void sim_write(uint32_t value)
{
    uint32_t prior=sim.odr;
    uint32_t affected=(value&0xffffu)|(value>>16u);
    CHECK(!(affected&~(CS|SCK|MOSI)));
    sim.odr=(prior|(value&0xffffu))&~(value>>16u);
    CHECK((sim.odr&~(CS|SCK|MOSI))==(prior&~(CS|SCK|MOSI)));
    if ((prior&CS) && !(sim.odr&CS)) {
        CHECK(!(sim.odr&SCK));
        sim.selected=1; sim.selected_at=sim.now;
        sim.bit_count=0; sim.reads=0;
        memset(sim.tx,0,sizeof(sim.tx));
    }
    if (!(prior&SCK) && (sim.odr&SCK)) {
        unsigned byte=sim.bit_count/8u;
        CHECK(sim.selected && byte<sizeof(sim.tx));
        CHECK((uint32_t)(sim.now-sim.selected_at)>=128u);
        if (sim.bit_count) CHECK((uint32_t)(sim.now-sim.last_fall)>=128u);
        sim.tx[byte]=(unsigned char)((sim.tx[byte]<<1u)|((sim.odr&MOSI)!=0));
        sim.last_rise=sim.now; ++sim.bit_count; ++sim.clocks;
    }
    if ((prior&SCK) && !(sim.odr&SCK)) sim.last_fall=sim.now;
    if (!(prior&CS) && (sim.odr&CS)) {
        CHECK(!(sim.odr&SCK));
        sim.selected=0; ++sim.completed;
    }
}
static uint32_t sim_read(void)
{
    unsigned offset=sim.reads/8u;
    unsigned bit=7u-(sim.reads%8u);
    unsigned value=0;
    CHECK(sim.selected && (sim.odr&SCK));
    CHECK((uint32_t)(sim.now-sim.last_rise)>=128u);
    CHECK(sim.bit_count==sim.reads+1u);
    if (offset && (sim.tx[0]&0x80u))
        value=sim.regs[((sim.tx[0]&0x7fu)+offset-1u)&0x7fu];
    ++sim.reads; ++sim.samples;
    return (value&(1u<<bit)) ? MISO : 0u;
}
static void reset(void)
{
    unsigned i;
    memset(&sim,0,sizeof(sim));
    memset(&test_gpio,0,sizeof(test_gpio));
    memset(&test_debug,0,sizeof(test_debug));
    memset(&test_dwt,0,sizeof(test_dwt));
    sim.hclk=64000000u; sim.odr=CS|0x123u;
    sim.jump_cycles=192000u;
    for(i=0;i<128u;++i)sim.regs[i]=(unsigned char)(i^0x5au);
    sim.regs[0x75]=0x47;
}
static void idle(void)
{
    CHECK((sim.odr&(CS|SCK|MOSI))==CS);
    CHECK((sim.odr&~(CS|SCK|MOSI))==0x123u);
}
static car_imu_bus_t open_bus(void)
{
    car_imu_bus_t bus;
    CHECK(car_imu_spi2_open(&bus));
    CHECK(bus.read && bus.write);
    CHECK(sim.clock_enabled==1 && sim.gpio_calls==2);
    CHECK(test_debug.DEMCR&CoreDebug_DEMCR_TRCENA_Msk);
    CHECK(test_dwt.CTRL&DWT_CTRL_CYCCNTENA_Msk);
    idle();return bus;
}
int main(void)
{
    car_imu_bus_t bus;
    unsigned char out[17];
    unsigned i,calls;
    reset();bus=open_bus();
    out[0]=0xaa;
    CHECK(bus.read(bus.context,0x75,out,1));
    CHECK(out[0]==0x47 && sim.tx[0]==0xf5 && sim.tx[1]==0);
    CHECK(sim.clocks==16 && sim.samples==16 && sim.completed==1);
#ifndef GPIO_SPI_BASELINE
    CHECK(car_imu_spi_diag.attempts==1 && car_imu_spi_diag.completed==1);
    CHECK(car_imu_spi_diag.failures==0 && car_imu_spi_diag.last_error==0);
#endif
    idle();
    CHECK(bus.write(bus.context,0xce,0x0f));
    CHECK(sim.tx[0]==0x4e && sim.tx[1]==0x0f);
    idle();
    CHECK(bus.read(bus.context,0x20,out,16));
    for(i=0;i<16;++i)CHECK(out[i]==sim.regs[0x20+i]);
    CHECK(sim.bit_count==136 && sim.reads==136);idle();
    calls=sim.cycle_calls;
    memset(out,0xa5,sizeof(out));
    CHECK(!bus.read(bus.context,0,out,0));
    CHECK(!bus.read(bus.context,0,out,17));
    CHECK(!bus.read(bus.context,0,NULL,1));
    sim.primask=1;
    CHECK(!bus.read(bus.context,0,out,1));
    CHECK(!bus.write(bus.context,0,1));
    CHECK(sim.cycle_calls==calls);
    sim.primask=0;idle();

    reset();sim.hclk=128000000u;
    memset(&bus,0xa5,sizeof(bus));
    CHECK(!car_imu_spi2_open(&bus));
    CHECK(!bus.read && !bus.write && !bus.context && !sim.gpio_calls);
    CHECK(!car_imu_spi2_open(NULL));
    reset();sim.primask=1;
    CHECK(!car_imu_spi2_open(&bus) && !sim.gpio_calls);
    reset();sim.freeze_at=1;
    CHECK(!car_imu_spi2_open(&bus));
    CHECK(sim.cycle_calls<=4097u);idle();

    reset();bus=open_bus();
    memset(out,0xa5,sizeof(out));sim.freeze_at=sim.cycle_calls+1u;
    calls=sim.cycle_calls;
    CHECK(!bus.read(bus.context,0x75,out,1));
    CHECK(sim.cycle_calls-calls<=4101u);
#ifndef GPIO_SPI_BASELINE
    CHECK(car_imu_spi_diag.last_error==CAR_IMU_SPI_ERROR_COUNTER_STOPPED);
#endif
    for(i=0;i<sizeof(out);++i)CHECK(out[i]==0xa5);idle();

    reset();bus=open_bus();memset(out,0xa5,sizeof(out));
    /* Abort after a complete received byte, without publishing partial output. */
    sim.jump_at=sim.cycle_calls+200u;
    CHECK(!bus.read(bus.context,0x20,out,16));
    CHECK(sim.clocks>=8 && sim.clocks<136);
#ifndef GPIO_SPI_BASELINE
    CHECK(car_imu_spi_diag.last_error==CAR_IMU_SPI_ERROR_DEADLINE);
    CHECK(car_imu_spi_diag.failed_command==0xa0 && car_imu_spi_diag.failed_length==17);
    CHECK(car_imu_spi_diag.failed_bits>0 && car_imu_spi_diag.failed_bits<136);
    CHECK(car_imu_spi_diag.failed_elapsed_cycles>=192000u);
    CHECK(car_imu_spi_diag.failures==1 && car_imu_spi_diag.completed==0);
#endif
    for(i=0;i<sizeof(out);++i)CHECK(out[i]==0xa5);idle();
    sim.jump_at=0;
    CHECK(bus.read(bus.context,0x75,out,1) && out[0]==0x47);idle();
#ifndef GPIO_SPI_BASELINE
    CHECK(car_imu_spi_diag.last_error==CAR_IMU_SPI_ERROR_DEADLINE);
    CHECK(car_imu_spi_diag.completed==1 && car_imu_spi_diag.failures==1);
#endif

    reset();bus=open_bus();memset(out,0xa5,sizeof(out));
    /* Model 1.25ms of instruction/peripheral/IRQ overhead during a 13-byte read.
       v8's 1ms cutoff rejects it; the bounded v9 3ms budget accepts it. This is
       a timing regression model, not a measured on-target interrupt duration. */
    sim.jump_at=sim.cycle_calls+200u;sim.jump_cycles=80000u;
#ifdef GPIO_SPI_BASELINE
    CHECK(!bus.read(bus.context,0x1f,out,12));
    for(i=0;i<sizeof(out);++i)CHECK(out[i]==0xa5);
#else
    CHECK(bus.read(bus.context,0x1f,out,12));
    for(i=0;i<12;++i)CHECK(out[i]==sim.regs[0x1f+i]);
    CHECK(car_imu_spi_diag.last_elapsed_cycles>64000u);
    CHECK(car_imu_spi_diag.last_elapsed_cycles<192000u);
    CHECK(car_imu_spi_diag.failures==0);
#endif
    idle();

    reset();bus=open_bus();sim.now=0xffffff00u;
    CHECK(bus.read(bus.context,0x75,out,1) && out[0]==0x47);
    CHECK(sim.now<0xffffff00u);idle();
    printf("GPIO SPI: 12 scenarios, %u assertions PASS\n",assertions);
    return 0;
}
