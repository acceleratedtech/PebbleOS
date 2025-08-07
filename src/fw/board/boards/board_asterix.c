#include <nrfx_i2s.h>

#include "board/board.h"
#include "drivers/flash/qspi_flash_definitions.h"
#include "drivers/gpio.h"
#include "drivers/i2c.h"
#include "drivers/i2c_definitions.h"
#include "drivers/mic.h"
#include "drivers/mic/nrf5/pdm_definitions.h"
#include "drivers/nrf5/i2c_hal_definitions.h"
#include "drivers/nrf5/spi_definitions.h"
#include "drivers/nrf5/uart_definitions.h"
#include "drivers/pmic/npm1300.h"
#include "drivers/pwm.h"
#include "drivers/qspi_definitions.h"
#include "drivers/rtc.h"
#include "flash_region/flash_region.h"
#include "kernel/util/sleep.h"
#include "system/passert.h"
#include "util/units.h"
#include "console/prompt.h"

// QSPI
#include <hal/nrf_clock.h>
#include <hal/nrf_gpio.h>
#include <nrfx_gpiote.h>
#include <nrfx_qspi.h>
#include <nrfx_spim.h>
#include <nrfx_twim.h>
#include <nrfx_pdm.h>

static QSPIPortState s_qspi_port_state;
static QSPIPort QSPI_PORT = {
    .state = &s_qspi_port_state,
    .auto_polling_interval = 16,
    .cs_gpio = NRF_GPIO_PIN_MAP(0, 17),
    .clk_gpio = NRF_GPIO_PIN_MAP(0, 19),
    .data_gpio =
        {
            NRF_GPIO_PIN_MAP(0, 20),
            NRF_GPIO_PIN_MAP(0, 21),
            NRF_GPIO_PIN_MAP(0, 22),
            NRF_GPIO_PIN_MAP(0, 23),
        },
};
QSPIPort *const QSPI = &QSPI_PORT;

static QSPIFlashState s_qspi_flash_state;
static QSPIFlash QSPI_FLASH_DEVICE = {
    .state = &s_qspi_flash_state,
    .qspi = &QSPI_PORT,
    .default_fast_read_ddr_enabled = false,
    .read_mode = QSPI_FLASH_READ_READ4IO,
    .write_mode = QSPI_FLASH_WRITE_PP4O,
    .reset_gpio = {GPIO_Port_NULL},
};
QSPIFlash *const QSPI_FLASH = &QSPI_FLASH_DEVICE;
IRQ_MAP_NRFX(QSPI, nrfx_qspi_irq_handler);
/* PERIPHERAL ID 43 */

static UARTDeviceState s_dbg_uart_state;
static UARTDevice DBG_UART_DEVICE = {
    .state = &s_dbg_uart_state,
    .tx_gpio = NRF_GPIO_PIN_MAP(0, 27),
    .rx_gpio = NRF_GPIO_PIN_MAP(0, 5),
    .rts_gpio = NRF_UARTE_PSEL_DISCONNECTED,
    .cts_gpio = NRF_UARTE_PSEL_DISCONNECTED,
    .periph = NRFX_UARTE_INSTANCE(0),
    .counter = NRFX_TIMER_INSTANCE(2),
};
UARTDevice *const DBG_UART = &DBG_UART_DEVICE;
IRQ_MAP_NRFX(UART0_UARTE0, nrfx_uarte_0_irq_handler);
/* PERIPHERAL ID 8 */

/* buttons */
IRQ_MAP_NRFX(TIMER1, nrfx_timer_1_irq_handler);
IRQ_MAP_NRFX(TIMER2, nrfx_timer_2_irq_handler);

/* display */
PwmState DISPLAY_EXTCOMIN_STATE;
IRQ_MAP_NRFX(SPIM3, nrfx_spim_3_irq_handler);

/* PERIPHERAL ID 10 */

/* EXTI */
IRQ_MAP_NRFX(GPIOTE, nrfx_gpiote_0_irq_handler);

/* nPM1300 */
static I2CBusState I2C_NPMC_IIC1_BUS_STATE = {};

static const I2CBusHal I2C_NPMC_IIC1_BUS_HAL = {
    .twim = NRFX_TWIM_INSTANCE(1),
    .frequency = NRF_TWIM_FREQ_400K,
};

static const I2CBus I2C_NPMC_IIC1_BUS = {
    .state = &I2C_NPMC_IIC1_BUS_STATE,
    .hal = &I2C_NPMC_IIC1_BUS_HAL,
    .scl_gpio =
        {
            .gpio = NRF5_GPIO_RESOURCE_EXISTS,
            .gpio_pin = NRF_GPIO_PIN_MAP(0, 14),
        },
    .sda_gpio =
        {
            .gpio = NRF5_GPIO_RESOURCE_EXISTS,
            .gpio_pin = NRF_GPIO_PIN_MAP(0, 15),
        },
    .name = "I2C_NPMC_IIC1",
};
IRQ_MAP_NRFX(SPI1_SPIM1_SPIS1_TWI1_TWIM1_TWIS1, nrfx_twim_1_irq_handler);
/* PERIPHERAL ID 9 */

static const I2CSlavePort I2C_SLAVE_NPM1300 = {
    .bus = &I2C_NPMC_IIC1_BUS,
    .address = 0x6B << 1,
};

I2CSlavePort *const I2C_NPM1300 = &I2C_SLAVE_NPM1300;

/* peripheral I2C bus */
static I2CBusState I2C_IIC2_BUS_STATE = {};

static const I2CBusHal I2C_IIC2_BUS_HAL = {
    .twim = NRFX_TWIM_INSTANCE(0),
    .frequency = NRF_TWIM_FREQ_400K,
};

static const I2CBus I2C_IIC2_BUS = {
    .state = &I2C_IIC2_BUS_STATE,
    .hal = &I2C_IIC2_BUS_HAL,
    .scl_gpio =
        {
            .gpio = NRF5_GPIO_RESOURCE_EXISTS,
            .gpio_pin = NRF_GPIO_PIN_MAP(0, 25),
        },
    .sda_gpio =
        {
            .gpio = NRF5_GPIO_RESOURCE_EXISTS,
            .gpio_pin = NRF_GPIO_PIN_MAP(0, 11),
        },
    .name = "I2C_IIC2",
};
IRQ_MAP_NRFX(SPI0_SPIM0_SPIS0_TWI0_TWIM0_TWIS0, nrfx_twim_0_irq_handler);

static const I2CSlavePort I2C_SLAVE_DRV2604 = {
    .bus = &I2C_IIC2_BUS,
    .address = 0x5A << 1,
};

I2CSlavePort *const I2C_DRV2604 = &I2C_SLAVE_DRV2604;

static const I2CSlavePort I2C_SLAVE_OPT3001 = {
    .bus = &I2C_IIC2_BUS,
    .address = 0x44 << 1,
};

I2CSlavePort *const I2C_OPT3001 = &I2C_SLAVE_OPT3001;

static const I2CSlavePort I2C_SLAVE_DA7212 = {
    .bus = &I2C_IIC2_BUS,
    .address = 0x1A << 1,
};

I2CSlavePort *const I2C_DA7212 = &I2C_SLAVE_DA7212;

static const I2CSlavePort I2C_SLAVE_MMC5603NJ = {
    .bus = &I2C_IIC2_BUS,
    .address = 0x30 << 1,
};

I2CSlavePort *const I2C_MMC5603NJ = &I2C_SLAVE_MMC5603NJ;

static const I2CSlavePort I2C_SLAVE_BMP390 = {
    .bus = &I2C_IIC2_BUS,
    .address = 0x76 << 1,
};

I2CSlavePort *const I2C_BMP390 = &I2C_SLAVE_BMP390;

static const I2CSlavePort I2C_SLAVE_LSM6D = {
    .bus = &I2C_IIC2_BUS,
    .address = 0x6A << 1,
};

I2CSlavePort *const I2C_LSM6D = &I2C_SLAVE_LSM6D;

IRQ_MAP_NRFX(I2S, nrfx_i2s_0_irq_handler);

IRQ_MAP_NRFX(PDM, NRFX_PDM_INST_HANDLER_GET(0));

/* PERIPHERAL ID 11 */

/* Microphone */
static MicDeviceState s_mic_state_storage;
static MicDevice s_mic_device = {
  .state = &s_mic_state_storage,
  .pdm_instance = NRFX_PDM_INSTANCE(0),
  .clk_pin = NRF_GPIO_PIN_MAP(1, 0),   // P1.00 - PDM CLK
  .data_pin = NRF_GPIO_PIN_MAP(0, 24), // P0.24 - PDM DATA
};
MicDevice * const MIC = &s_mic_device;

/* sensor SPI bus */

/* asterix shares SPI with flash, which we don't support */

PwmState BACKLIGHT_PWM_STATE;
IRQ_MAP_NRFX(PWM0, nrfx_pwm_0_irq_handler);

IRQ_MAP_NRFX(RTC1, rtc_irq_handler);

const Npm1300Config NPM1300_CONFIG = {
  // 128mA = ~1C (rapid charge)
  .chg_current_ma = 128,
  .dischg_limit_ma = 200,
  .term_current_pct = 10,
  .thermistor_beta = 3380,
};

void board_early_init(void) {
  PBL_LOG(LOG_LEVEL_ERROR, "asterix early init");

  NRF_NVMC->ICACHECNF |= NVMC_ICACHECNF_CACHEEN_Msk;

  nrf_clock_lf_src_set(NRF_CLOCK, NRF_CLOCK_LFCLK_XTAL);
  nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED);
  nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_LFCLKSTART);
  /* TODO: Add timeout, report failure if LFCLK does not start. For now,
   * WDT should trigger a reboot. Calibrated RC may be used as a fallback,
   * provided we can adjust BLE SCA settings at runtime.
   */
  while (!nrf_clock_event_check(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED)) {
  }
  nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED);
}

void board_init(void) {
  i2c_init(&I2C_NPMC_IIC1_BUS);
  i2c_init(&I2C_IIC2_BUS);

  uint8_t da7212_powerdown[] = { 0xFD /* SYSTEM_ACTIVE */, 0 };
  i2c_use(I2C_DA7212);
  i2c_write_block(I2C_DA7212, 2, da7212_powerdown);
  i2c_release(I2C_DA7212);
  
  // XXX: FIRM-264: stop mode breaks NimBLE
  stop_mode_disable(InhibitorMain);
}

extern void HACK_pmic_kill_ldo(int ldo);
extern void HACK_pmic_kill_buck(int buck);

void command_systemoff(void) {
  prompt_command_finish();
  
  /* does not seem to change system power consumption */
  uint8_t da7212_powerdown[] = { 0xFD /* SYSTEM_ACTIVE */, 0 };
  i2c_use(I2C_DA7212);
  i2c_write_block(I2C_DA7212, 2, da7212_powerdown);
  i2c_release(I2C_DA7212);
  
  flash_power_down_for_stop_mode();

  // pulls in about 20 uA over above
  HACK_pmic_kill_ldo(1);
  HACK_pmic_kill_ldo(2);
  
  // does not seem to pull in >10 uA, but does seem to reduce noise, so at
  // least something is happening there
  HACK_pmic_kill_buck(2);
  
  // pulls in about 100 uA
  HACK_pmic_kill_buck(1);

  // obviously we never get here if we kill BUCK1.
  __DSB();
  __ISB();
  
  NRF_POWER->SYSTEMOFF = 1;
}

void command_wfi_forever(void) {
  extern void do_wfi();
  
  prompt_command_finish();
  flash_power_down_for_stop_mode();

  HACK_pmic_kill_ldo(1);
  HACK_pmic_kill_ldo(2);
  HACK_pmic_kill_buck(2);

  // Not __disable_irq -- that doesn't actually stop IRQs from waking us. 
  // (See comment in src/fw/freertos_application.c.)
  portENTER_CRITICAL();
  
  NRF_NVMC->ICACHECNF &= ~NVMC_ICACHECNF_CACHEEN_Msk;
  
  
  NRF_UARTE0->ENABLE = 0;
  NRF_UARTE0->INTENCLR = 0xFFFFFFFF;
  NRF_PWM0->ENABLE = 0;
  NRF_PWM1->ENABLE = 0;
  NRF_QSPI->TASKS_DEACTIVATE = 1;
  *(volatile uint32_t *)0x40029010ul = 1ul; // WAR erratum 122
  *(volatile uint32_t *)0x40029054ul = 1ul;
  NRF_QSPI->ENABLE = 0;
  
  NRF_TIMER0->TASKS_STOP = 1;
  NRF_TIMER0->INTENCLR = 0xFFFFFFFF;
  NRF_TIMER1->TASKS_STOP = 1;
  NRF_TIMER1->INTENCLR = 0xFFFFFFFF;
  NRF_TIMER2->TASKS_STOP = 1;
  NRF_TIMER2->INTENCLR = 0xFFFFFFFF;
  NRF_TIMER3->TASKS_STOP = 1;
  NRF_TIMER3->INTENCLR = 0xFFFFFFFF;
  NRF_TIMER4->TASKS_STOP = 1;
  NRF_TIMER4->INTENCLR = 0xFFFFFFFF;

  NRF_RTC0->TASKS_STOP = 1;
  NRF_RTC0->INTENCLR = 0xFFFFFFFF;
  NRF_RTC1->TASKS_STOP = 1;
  NRF_RTC1->INTENCLR = 0xFFFFFFFF;
  NRF_RTC2->TASKS_STOP = 1;
  NRF_RTC2->INTENCLR = 0xFFFFFFFF;
  
  NRF_GPIOTE->CONFIG[0] = 0;
  NRF_GPIOTE->EVENTS_IN[0] = 0;
  NRF_GPIOTE->CONFIG[1] = 0;
  NRF_GPIOTE->CONFIG[2] = 0;
  NRF_GPIOTE->CONFIG[3] = 0;
  NRF_GPIOTE->CONFIG[4] = 0;
  NRF_GPIOTE->CONFIG[5] = 0;
  NRF_GPIOTE->CONFIG[6] = 0;
  NRF_GPIOTE->CONFIG[7] = 0;

  //NRF_P0->DIR = 0;
  //NRF_P1->DIR = 0;

  //for (int i = 0; i < 32; i++) {
  //  NRF_P0->PIN_CNF[i] = 0x00000002;
  //  NRF_P1->PIN_CNF[i] = 0x00000002;
  //}
  
  NRF_RADIO->POWER = 0;

  NRF_GPIOTE->INTENCLR = 0xFFFFFFFF;
  NRF_RNG->TASKS_STOP = 1;
  NRF_PPI->CHEN = 0;
  for (int i = 0; i < 20; i++) {
    NRF_PPI->CH[i].TEP = 0;
    NRF_PPI->CH[i].EEP = 0;
  }
  NRF_SPIM3->INTENCLR = 0xFFFFFFFF;
  
  NRF_TIMER0->TASKS_SHUTDOWN = 1;
  NRF_TIMER1->TASKS_SHUTDOWN = 1;
  NRF_TIMER2->TASKS_SHUTDOWN = 1;
  NRF_TIMER3->TASKS_SHUTDOWN = 1;
  NRF_TIMER4->TASKS_SHUTDOWN = 1;
  
  *(volatile uint32_t *)0x4002F004 = 1; // WAR erratum 195
  
  // *(volatile uint32_t *)0x4007AC84ul = 0x00000002ul; // WAR erratum 246, but this makes it worse

  // WAR erratum 87
  __set_FPSCR(__get_FPSCR() & ~0x9F);
  (void) __get_FPSCR();
  NVIC_ClearPendingIRQ(FPU_IRQn);
  
  // disable FPU
  SCB->CPACR &= ~((3UL << 20ul) | (3UL << 22ul));
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  SysTick->CTRL = 0;
  CoreDebug->DHCSR &= ~CoreDebug_DHCSR_C_DEBUGEN_Msk;
  CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
  
  // disable all nvic enables and pendings
  for (int i = 0; i < 8; i++) {
    NVIC->ICER[i] = 0xFFFFFFFF;
    NVIC->ICPR[i] = 0xFFFFFFFF;
  }
  
  NRF_UARTE0->EVENTS_CTS = 0;
  NRF_UARTE0->EVENTS_TXDRDY = 0;
  NRF_UARTE0->EVENTS_TXSTARTED = 0;
  NRF_UARTE0->EVENTS_TXSTOPPED = 0;
  NRF_UARTE0->SHORTS = 0;
  NRF_UARTE0->PSEL.TXD = 0xFFFFFFFF;
  NRF_UARTE0->PSEL.RXD = 0xFFFFFFFF;
  
  NRF_SPIM0->EVENTS_ENDRX = 0;
  NRF_SPIM0->EVENTS_ENDTX = 0;
  NRF_SPIM0->EVENTS_STARTED = 0;
  NRF_SPIM0->PSEL.SCK = 0xFFFFFFFF;
  NRF_SPIM0->PSEL.MOSI = 0xFFFFFFFF;
  NRF_TWIM0->EVENTS_TXSTARTED = 0;
  
  NRF_TIMER0->BITMODE = 0;
  NRF_TIMER1->SHORTS = 0;
  NRF_TIMER1->BITMODE = 0;
  NRF_TIMER1->PRESCALER = 4;
  NRF_TIMER1->CC[0] = 0;
  NRF_TIMER2->MODE = 0;
  NRF_TIMER2->BITMODE = 0;
  NRF_TIMER2->CC[0] = 0;
  NRF_RNG->CONFIG = 0;
  NRF_ECB->TASKS_STOPECB = 1;
  NRF_AAR->TASKS_STOP = 1;
  NRF_CCM->SHORTS = 0;
  NRF_CCM->MODE = 1;
  NRF_CCM->CNFPTR = 0;
  NRF_PWM0->EVENTS_SEQSTARTED[1] = 0;
  NRF_PWM0->EVENTS_SEQEND[1] = 0;
  NRF_PWM0->EVENTS_PWMPERIODEND = 0;
  NRF_PWM0->EVENTS_LOOPSDONE = 0;
  NRF_PWM0->PSEL.OUT[0] = 0xFFFFFFFF;
  NRF_PWM1->EVENTS_SEQSTARTED[1] = 0;
  NRF_PWM1->EVENTS_SEQEND[1] = 0;
  NRF_PWM1->EVENTS_PWMPERIODEND = 0;
  NRF_PWM1->EVENTS_LOOPSDONE = 0;
  NRF_PWM1->PSEL.OUT[0] = 0xFFFFFFFF;
  NRF_QSPI->IFCONFIG0 = 0;
  NRF_QSPI->IFCONFIG1 = 0;
  NRF_QSPI->PSEL.SCK = 0xffffffff;
  NRF_QSPI->PSEL.CSN = 0xffffffff;
  NRF_QSPI->PSEL.IO0 = 0xffffffff;
  NRF_QSPI->PSEL.IO1 = 0xffffffff;
  NRF_QSPI->PSEL.IO2 = 0xffffffff;
  NRF_QSPI->PSEL.IO3 = 0xffffffff;
  
  NRF_SPIM3->EVENTS_ENDRX = 0;
  NRF_SPIM3->EVENTS_ENDTX = 0;
  NRF_SPIM3->EVENTS_STARTED = 0;
  NRF_SPIM3->ORC = 0;
  NRF_SPIM3->PSEL.SCK = 0xFFFFFFFF;
  NRF_SPIM3->PSEL.MOSI = 0xFFFFFFFF;
  
  while (1) {
    __DSB();
    __ISB();
    do_wfi();
  }
}
