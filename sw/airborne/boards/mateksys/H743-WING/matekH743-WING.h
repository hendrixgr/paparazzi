#ifndef CONFIG_MATEK_H743_WING_H
#define CONFIG_MATEK_H743_WING_H

#define BOARD_MATEK_H743_WING

/**
 * ChibiOS board file
 */
#include "board.h"

/**
 * PPRZ definitions
 */

/*
 * AHB_CLK
 */
#define AHB_CLK STM32_HCLK


/*
 * Concat macro
 */
#define _CONCAT_BOARD_PARAM(_s1, _s2) _s1 ## _s2
#define CONCAT_BOARD_PARAM(_s1, _s2) _CONCAT_BOARD_PARAM(_s1, _s2)

/*
 * LEDs
 */
/* blue, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO PAL_PORT(LINE_LED1)
#define LED_1_GPIO_PIN PAL_PAD(LINE_LED1)
#define LED_1_GPIO_ON gpio_set
#define LED_1_GPIO_OFF gpio_clear

/* greeg, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO PAL_PORT(LINE_LED2)
#define LED_2_GPIO_PIN PAL_PAD(LINE_LED2)
#define LED_2_GPIO_ON gpio_set
#define LED_2_GPIO_OFF gpio_clear

/*
 * ADCs
 */
// AIRSPEED
#if USE_ADC_1
#define AD1_1_CHANNEL CONCAT_BOARD_PARAM(ADC_CHANNEL_IN, AIRSPEED_ADC_IN)
#define ADC_1 AD1_1
#define ADC_1_GPIO_PORT PAL_PORT(LINE_AIRSPEED)
#define ADC_1_GPIO_PIN PAL_PAD(LINE_AIRSPEED)
#endif

// RSSI
#if USE_ADC_2
#define AD1_2_CHANNEL CONCAT_BOARD_PARAM(ADC_CHANNEL_IN, RSSI_ADC_IN)
#define ADC_2 AD1_2
#define ADC_2_GPIO_PORT PAL_PORT(LINE_RSSI)
#define ADC_2_GPIO_PIN PAL_PAD(LINE_RSSI)
#endif

// VBAT enabled by default
#ifndef USE_ADC_3
#define USE_ADC_3 1
#endif
#if USE_ADC_3
#define AD1_3_CHANNEL CONCAT_BOARD_PARAM(ADC_CHANNEL_IN, VBAT_MEAS_ADC_IN)
#define ADC_3 AD1_3
#define ADC_3_GPIO_PORT PAL_PORT(LINE_VBAT_MEAS)
#define ADC_3_GPIO_PIN PAL_PAD(LINE_VBAT_MEAS)
#endif

// CURRENT
#if USE_ADC_4
#define AD1_4_CHANNEL CONCAT_BOARD_PARAM(ADC_CHANNEL_IN, CURRENT_MEAS_ADC_IN)
#define ADC_4 AD1_4
#define ADC_4_GPIO_PORT PAL_PORT(LINE_CURRENT_MEAS)
#define ADC_4_GPIO_PIN PAL_PAD(LINE_CURRENT_MEAS)
#endif

/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY ADC_3
#endif

/*
 * R1 = 1k
 * R2 = 10k
 * adc * (3.3 / 2^12) * ((R1 + R2) / R1)
 */
#define VBAT_R1 1000.0f
#define VBAT_R2 10000.0f
#define DefaultVoltageOfAdc(adc) ((3.3f/4096.0f)*((VBAT_R1+VBAT_R2)/VBAT_R1)*adc)

/*
 * current sensor: 132A, 3.3V 12bits ADC -> 40 A/V -> 40000 * 3.3/2^12 mA/ADC
 */
#define DefaultMilliAmpereOfAdc(adc) ((40000.f*3.3f/4096.f)*adc)

/*
 * PWM defines
 */

#ifndef USE_PWM1
#define USE_PWM1 1
#endif
#if USE_PWM1
#define PWM_SERVO_1 1
#define PWM_SERVO_1_GPIO PAL_PORT(LINE_S1)
#define PWM_SERVO_1_PIN PAL_PAD(LINE_S1)
#define PWM_SERVO_1_AF AF_S1
#define PWM_SERVO_1_DRIVER CONCAT_BOARD_PARAM(PWMD, S1_TIM)
#define PWM_SERVO_1_CHANNEL (S1_TIM_CH)
#define PWM_SERVO_1_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_1_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM2
#define USE_PWM2 1
#endif
#if USE_PWM2
#define PWM_SERVO_2 2
#define PWM_SERVO_2_GPIO PAL_PORT(LINE_S2)
#define PWM_SERVO_2_PIN PAL_PAD(LINE_S2)
#define PWM_SERVO_2_AF AF_S2
#define PWM_SERVO_2_DRIVER CONCAT_BOARD_PARAM(PWMD, S2_TIM)
#define PWM_SERVO_2_CHANNEL (S2_TIM_CH)
#define PWM_SERVO_2_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_2_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM3
#define USE_PWM3 1
#endif
#if USE_PWM3
#define PWM_SERVO_3 3
#define PWM_SERVO_3_GPIO PAL_PORT(LINE_S3)
#define PWM_SERVO_3_PIN PAL_PAD(LINE_S3)
#define PWM_SERVO_3_AF AF_S3
#define PWM_SERVO_3_DRIVER CONCAT_BOARD_PARAM(PWMD, S3_TIM)
#define PWM_SERVO_3_CHANNEL (S3_TIM_CH)
#define PWM_SERVO_3_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_3_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM4
#define USE_PWM4 1
#endif
#if USE_PWM4
#define PWM_SERVO_4 4
#define PWM_SERVO_4_GPIO PAL_PORT(LINE_S4)
#define PWM_SERVO_4_PIN PAL_PAD(LINE_S4)
#define PWM_SERVO_4_AF AF_S4
#define PWM_SERVO_4_DRIVER CONCAT_BOARD_PARAM(PWMD, S4_TIM)
#define PWM_SERVO_4_CHANNEL (S4_TIM_CH)
#define PWM_SERVO_4_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_4_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM5
#define USE_PWM5 1
#endif
#if USE_PWM5
#define PWM_SERVO_5 5
#define PWM_SERVO_5_GPIO PAL_PORT(LINE_S5)
#define PWM_SERVO_5_PIN PAL_PAD(LINE_S5)
#define PWM_SERVO_5_AF AF_S5
#define PWM_SERVO_5_DRIVER CONCAT_BOARD_PARAM(PWMD, S5_TIM)
#define PWM_SERVO_5_CHANNEL (S5_TIM_CH)
#define PWM_SERVO_5_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_5_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM6
#define USE_PWM6 1
#endif
#if USE_PWM6
#define PWM_SERVO_6 6
#define PWM_SERVO_6_GPIO PAL_PORT(LINE_S6)
#define PWM_SERVO_6_PIN PAL_PAD(LINE_S6)
#define PWM_SERVO_6_AF AF_S6
#define PWM_SERVO_6_DRIVER CONCAT_BOARD_PARAM(PWMD, S6_TIM)
#define PWM_SERVO_6_CHANNEL (S6_TIM_CH)
#define PWM_SERVO_6_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_6_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM7
#define USE_PWM7 1
#endif
#if USE_PWM7
#define PWM_SERVO_7 7
#define PWM_SERVO_7_GPIO PAL_PORT(LINE_S7)
#define PWM_SERVO_7_PIN PAL_PAD(LINE_S7)
#define PWM_SERVO_7_AF AF_S7
#define PWM_SERVO_7_DRIVER CONCAT_BOARD_PARAM(PWMD, S7_TIM)
#define PWM_SERVO_7_CHANNEL (S7_TIM_CH)
#define PWM_SERVO_7_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_7_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM8
#define USE_PWM8 1
#endif
#if USE_PWM8
#define PWM_SERVO_8 8
#define PWM_SERVO_8_GPIO PAL_PORT(LINE_S8)
#define PWM_SERVO_8_PIN PAL_PAD(LINE_S8)
#define PWM_SERVO_8_AF AF_S8
#define PWM_SERVO_8_DRIVER CONCAT_BOARD_PARAM(PWMD, S8_TIM)
#define PWM_SERVO_8_CHANNEL (S8_TIM_CH)
#define PWM_SERVO_8_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_8_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM9
#define USE_PWM9 1
#endif
#if USE_PWM9
#define PWM_SERVO_9 9
#define PWM_SERVO_9_GPIO PAL_PORT(LINE_S9)
#define PWM_SERVO_9_PIN PAL_PAD(LINE_S9)
#define PWM_SERVO_9_AF AF_S9
#define PWM_SERVO_9_DRIVER CONCAT_BOARD_PARAM(PWMD, S9_TIM)
#define PWM_SERVO_9_CHANNEL (S9_TIM_CH)
#define PWM_SERVO_9_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_9_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM10
#define USE_PWM10 1
#endif
#if USE_PWM10
#define PWM_SERVO_10 10
#define PWM_SERVO_10_GPIO PAL_PORT(LINE_S10)
#define PWM_SERVO_10_PIN PAL_PAD(LINE_S10)
#define PWM_SERVO_10_AF AF_S10
#define PWM_SERVO_10_DRIVER CONCAT_BOARD_PARAM(PWMD, S10_TIM)
#define PWM_SERVO_10_CHANNEL (S10_TIM_CH)
#define PWM_SERVO_10_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_10_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM11
#define USE_PWM11 1
#endif
#if USE_PWM11
#define PWM_SERVO_11 11
#define PWM_SERVO_11_GPIO PAL_PORT(LINE_S11)
#define PWM_SERVO_11_PIN PAL_PAD(LINE_S11)
#define PWM_SERVO_11_AF AF_S11
#define PWM_SERVO_11_DRIVER CONCAT_BOARD_PARAM(PWMD, S11_TIM)
#define PWM_SERVO_11_CHANNEL (S11_TIM_CH)
#define PWM_SERVO_11_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_11_ACTIVE PWM_OUTPUT_DISABLED
#endif

#ifndef USE_PWM12
#define USE_PWM12 1
#endif
#if USE_PWM12
#define PWM_SERVO_12 12
#define PWM_SERVO_12_GPIO PAL_PORT(LINE_S12)
#define PWM_SERVO_12_PIN PAL_PAD(LINE_S12)
#define PWM_SERVO_12_AF AF_S12
#define PWM_SERVO_12_DRIVER CONCAT_BOARD_PARAM(PWMD, S12_TIM)
#define PWM_SERVO_12_CHANNEL (S12_TIM_CH)
#define PWM_SERVO_12_ACTIVE PWM_OUTPUT_ACTIVE_HIGH
#else
#define PWM_SERVO_12_ACTIVE PWM_OUTPUT_DISABLED
#endif

// servo index starting at 1 + regular servos + aux servos
// so NB = 1+10+2
#define ACTUATORS_PWM_NB 13


#ifdef STM32_PWM_USE_TIM2
#define PWM_CONF_TIM2 STM32_PWM_USE_TIM2
#else
#define PWM_CONF_TIM2 1
#endif
#define PWM_CONF2_DEF { \
    PWM_FREQUENCY, \
    PWM_FREQUENCY/TIM2_SERVO_HZ, \
    NULL, \
    { \
      { PWM_SERVO_1_ACTIVE, NULL }, \
      { PWM_SERVO_2_ACTIVE, NULL }, \
      { PWM_OUTPUT_DISABLED, NULL }, \
      { PWM_OUTPUT_DISABLED, NULL }, \
    }, \
    0, \
    0 \
  }

#ifdef STM32_PWM_USE_TIM4
#define PWM_CONF_TIM4 STM32_PWM_USE_TIM4
#else
#define PWM_CONF_TIM4 1
#endif
#define PWM_CONF4_DEF { \
    PWM_FREQUENCY, \
    PWM_FREQUENCY/TIM4_SERVO_HZ, \
    NULL, \
    { \
      { PWM_SERVO_7_ACTIVE, NULL }, \
      { PWM_SERVO_8_ACTIVE, NULL }, \
      { PWM_SERVO_9_ACTIVE, NULL }, \
      { PWM_SERVO_10_ACTIVE, NULL }, \
    }, \
    0, \
    0 \
  }

#ifdef STM32_PWM_USE_TIM5
#define PWM_CONF_TIM5 STM32_PWM_USE_TIM5
#else
#define PWM_CONF_TIM5 1
#endif
#define PWM_CONF5_DEF { \
    PWM_FREQUENCY, \
    PWM_FREQUENCY/TIM5_SERVO_HZ, \
    NULL, \
    { \
      { PWM_OUTPUT_DISABLED, NULL }, \
      { PWM_OUTPUT_DISABLED, NULL }, \
      { PWM_SERVO_3_ACTIVE, NULL }, \
      { PWM_SERVO_4_ACTIVE, NULL }, \
    }, \
    0, \
    0 \
  }

#ifdef STM32_PWM_USE_TIM8
#define PWM_CONF_TIM8 STM32_PWM_USE_TIM8
#else
#define PWM_CONF_TIM8 1
#endif
#define PWM_CONF8_DEF { \
    PWM_FREQUENCY, \
    PWM_FREQUENCY/TIM8_SERVO_HZ, \
    NULL, \
    { \
      { PWM_OUTPUT_DISABLED, NULL }, \
      { PWM_SERVO_5_ACTIVE, NULL }, \
      { PWM_SERVO_6_ACTIVE, NULL }, \
      { PWM_OUTPUT_DISABLED, NULL }, \
    }, \
    0, \
    0 \
  }

#ifdef STM32_PWM_USE_TIM9
#define PWM_CONF_TIM9 STM32_PWM_USE_TIM9
#else
#define PWM_CONF_TIM9 1
#endif
#define PWM_CONF9_DEF { \
    PWM_FREQUENCY, \
    PWM_FREQUENCY/TIM9_SERVO_HZ, \
    NULL, \
    { \
      { PWM_SERVO_11_ACTIVE, NULL }, \
      { PWM_SERVO_12_ACTIVE, NULL }, \
      { PWM_OUTPUT_DISABLED, NULL }, \
      { PWM_OUTPUT_DISABLED, NULL }, \
    }, \
    0, \
    0 \
  }

#ifdef STM32_PWM_USE_TIM15
#define PWM_CONF_TIM15 STM32_PWM_USE_TIM15
#else
#define PWM_CONF_TIM15 1
#endif
#define PWM_CONF15_DEF { \
    PWM_FREQUENCY, \
    PWM_FREQUENCY/TIM15_SERVO_HZ, \
    NULL, \
    { \
      { PWM_SERVO_11_ACTIVE, NULL }, \
      { PWM_SERVO_12_ACTIVE, NULL }, \
      { PWM_OUTPUT_DISABLED, NULL }, \
      { PWM_OUTPUT_DISABLED, NULL }, \
    }, \
    0, \
    0 \
  }

/**
 * DSHOT
 */
#ifndef DSHOT_TELEMETRY_DEV
#define DSHOT_TELEMETRY_DEV NULL
#endif

#ifndef USE_DSHOT_TIM4
#define USE_DSHOT_TIM4 1
#endif

#if USE_DSHOT_TIM4 // Servo 7, 8, 9, 10 on TIM4

// Servo B1, B2, B3, B4 on TM4 are primary DSHOT connector
#define DSHOT_SERVO_7 1
#define DSHOT_SERVO_7_GPIO PAL_PORT(LINE_S7)
#define DSHOT_SERVO_7_PIN PAL_PAD(LINE_S7)
#define DSHOT_SERVO_7_AF AF_S7
#define DSHOT_SERVO_7_DRIVER CONCAT_BOARD_PARAM(DSHOTD, S7_TIM)
#define DSHOT_SERVO_7_CHANNEL S7_TIM_CH

#define DSHOT_SERVO_8 2
#define DSHOT_SERVO_8_GPIO PAL_PORT(LINE_S8)
#define DSHOT_SERVO_8_PIN PAL_PAD(LINE_S8)
#define DSHOT_SERVO_8_AF AF_S8
#define DSHOT_SERVO_8_DRIVER CONCAT_BOARD_PARAM(DSHOTD, S8_TIM)
#define DSHOT_SERVO_8_CHANNEL S8_TIM_CH

#define DSHOT_SERVO_9 3
#define DSHOT_SERVO_9_GPIO PAL_PORT(LINE_S9)
#define DSHOT_SERVO_9_PIN PAL_PAD(LINE_S9)
#define DSHOT_SERVO_9_AF AF_S9
#define DSHOT_SERVO_9_DRIVER CONCAT_BOARD_PARAM(DSHOTD, S9_TIM)
#define DSHOT_SERVO_9_CHANNEL S9_TIM_CH

#define DSHOT_SERVO_10 4
#define DSHOT_SERVO_10_GPIO PAL_PORT(LINE_S10)
#define DSHOT_SERVO_10_PIN PAL_PAD(LINE_S10)
#define DSHOT_SERVO_10_AF AF_S10
#define DSHOT_SERVO_10_DRIVER CONCAT_BOARD_PARAM(DSHOTD, S10_TIM)
#define DSHOT_SERVO_10_CHANNEL S10_TIM_CH

#define DSHOT_CONF_TIM4 1
#define DSHOT_CONF4_DEF { \
    .dma_stream = STM32_PWM4_UP_DMA_STREAM,   \
                  .dma_channel = STM32_PWM4_UP_DMA_CHANNEL, \
                                 .pwmp = &PWMD4,                           \
                                         .tlm_sd = DSHOT_TELEMETRY_DEV,            \
                                             .dma_buf = &dshot4DmaBuffer,              \
                                                 .dcache_memory_in_use = false             \
  }

#endif

/**
 * UART7 (Modem with optional flow control disabled by default)
 */
#define UART7_GPIO_PORT_TX  PAL_PORT(LINE_UART7_TX)
#define UART7_GPIO_TX       PAL_PAD(LINE_UART7_TX)
#define UART7_GPIO_PORT_RX  PAL_PORT(LINE_UART7_RX)
#define UART7_GPIO_RX       PAL_PAD(LINE_UART7_RX)
#define UART7_GPIO_AF       AF_UART7_TX
#ifndef UART7_HW_FLOW_CONTROL
#define UART7_HW_FLOW_CONTROL FALSE
#endif

/**
 * UART2 (GPS) and UART3 (Companion)
 * are configured as UART from ChibiOS board file by default
 */

#define UART2_GPIO_PORT_TX  PAL_PORT(LINE_UART2_TX)
#define UART2_GPIO_TX       PAL_PAD(LINE_UART2_TX)
#define UART2_GPIO_PORT_RX  PAL_PORT(LINE_UART2_RX)
#define UART2_GPIO_RX       PAL_PAD(LINE_UART2_RX)
#define UART2_GPIO_AF       AF_UART2_TX

#define UART3_GPIO_PORT_TX  PAL_PORT(LINE_UART3_TX)
#define UART3_GPIO_TX       PAL_PAD(LINE_UART3_TX)
#define UART3_GPIO_PORT_RX  PAL_PORT(LINE_UART3_RX)
#define UART3_GPIO_RX       PAL_PAD(LINE_UART3_RX)
#define UART3_GPIO_AF       AF_UART3_TX

/**
 * UART8
 */

#define UART8_GPIO_PORT_TX  PAL_PORT(LINE_UART8_TX)
#define UART8_GPIO_TX       PAL_PAD(LINE_UART8_TX)
#define UART8_GPIO_PORT_RX  PAL_PORT(LINE_UART8_RX)
#define UART8_GPIO_RX       PAL_PAD(LINE_UART8_RX)
#define UART8_GPIO_AF       AF_UART8_TX

/**
 * SBUS / Spektrum port
 */

#define USE_UART6_RX TRUE
#define USE_UART6_TX FALSE
#define UART6_GPIO_PORT_RX  PAL_PORT(LINE_RC1)
#define UART6_GPIO_RX       PAL_PAD(LINE_RC1)
#define UART6_GPIO_AF       RC1_USART_AF

/* The line that is pulled low at power up to initiate the bind process
 * PB1: AUXb4
 */
#define SPEKTRUM_BIND_PIN       PAL_PORT(LINE_XXX)
#define SPEKTRUM_BIND_PIN_PORT  PAL_PAD(LINE_XXX)

// no wait with chibios as the RTC oscillator takes longer to stabilize
#define SPEKTRUM_BIND_WAIT 30000

/**
 * PPM radio defines
 *
 * available on RC1
 */
#define RC_PPM_TICKS_PER_USEC 6
#define PPM_TIMER_FREQUENCY 6000000
#define PPM_CHANNEL CONCAT_BOARD_PARAM(ICU_CHANNEL_, RC1_TIM_CH)
#define PPM_TIMER CONCAT_BOARD_PARAM(ICUD, RC1_TIM)

/*
 * PWM input
 */
// PWM_INPUT 1
#define PWM_INPUT1_ICU            ICUD1
#define PWM_INPUT1_CHANNEL        ICU_CHANNEL_1
#define PWM_INPUT1_GPIO_PORT      PAL_PORT(LINE_LED_WS2812)
#define PWM_INPUT1_GPIO_PIN       PAL_PAD(LINE_LED_WS2812)
#define PWM_INPUT1_GPIO_AF        AF_LED_WS2812


/**
 * I2C defines
 */
// Digital noise filter: 0 disabled, [0x1 - 0xF] enable up to n t_I2CCLK
#define STM32_CR1_DNF(n)          ((n & 0x0f) << 8)
// Timing register
#define I2C_FAST_400KHZ_DNF0_100NS_PCLK54MHZ_TIMINGR  (STM32_TIMINGR_PRESC(0U) | \
    STM32_TIMINGR_SCLDEL(10U) | STM32_TIMINGR_SDADEL(0U) | \
    STM32_TIMINGR_SCLH(34U)  | STM32_TIMINGR_SCLL(86U))
#define I2C_STD_100KHZ_DNF0_100NS_PCLK54MHZ_TIMINGR  (STM32_TIMINGR_PRESC(1U) | \
    STM32_TIMINGR_SCLDEL(9U) | STM32_TIMINGR_SDADEL(0U) | \
    STM32_TIMINGR_SCLH(105U)  | STM32_TIMINGR_SCLL(153U))


#ifndef I2C1_CLOCK_SPEED
#define I2C1_CLOCK_SPEED 400000
#endif

#if I2C1_CLOCK_SPEED == 400000
#define I2C1_CFG_DEF { \
    .timingr = I2C_FAST_400KHZ_DNF0_100NS_PCLK54MHZ_TIMINGR, \
               .cr1 = STM32_CR1_DNF(0), \
                      .cr2 = 0 \
  }
#elif I2C1_CLOCK_SPEED == 100000
#define I2C1_CFG_DEF { \
    .timingr = I2C_STD_100KHZ_DNF0_100NS_PCLK54MHZ_TIMINGR, \
               .cr1 = STM32_CR1_DNF(0), \
                      .cr2 = 0 \
  }
#else
#error "Unknown I2C1 clock speed"
#endif

#ifndef I2C2_CLOCK_SPEED
#define I2C2_CLOCK_SPEED 400000
#endif

#if I2C2_CLOCK_SPEED == 400000
#define I2C2_CFG_DEF { \
    .timingr = I2C_FAST_400KHZ_DNF0_100NS_PCLK54MHZ_TIMINGR, \
               .cr1 = STM32_CR1_DNF(0), \
                      .cr2 = 0 \
  }
#elif I2C2_CLOCK_SPEED == 100000
#define I2C2_CFG_DEF { \
    .timingr = I2C_STD_100KHZ_DNF0_100NS_PCLK54MHZ_TIMINGR, \
               .cr1 = STM32_CR1_DNF(0), \
                      .cr2 = 0 \
  }
#else
#error "Unknown I2C2 clock speed"
#endif

/**
 * SPI Config
 */


// Internal SPI1 (IMU1)
#define SPI1_GPIO_AF          AF_SPI1_CLK
#define SPI1_GPIO_PORT_MISO   PAL_PORT(LINE_SPI1_MISO)
#define SPI1_GPIO_MISO        PAL_PAD(LINE_SPI1_MISO)
#define SPI1_GPIO_PORT_MOSI   PAL_PORT(LINE_SPI1_MOSI)
#define SPI1_GPIO_MOSI        PAL_PAD(LINE_SPI1_MOSI)
#define SPI1_GPIO_PORT_SCK    PAL_PORT(LINE_SPI1_CLK)
#define SPI1_GPIO_SCK         PAL_PAD(LINE_SPI1_CLK)
// SLAVE1 on SPI1 (IMU1)
#define SPI_SELECT_SLAVE1_PORT  PAL_PORT(LINE_SPI1_CS)
#define SPI_SELECT_SLAVE1_PIN   PAL_PAD(LINE_SPI1_CS)
// SLAVE0 on SPI1 (IMU1)
//#define SPI_SELECT_SLAVE0_PORT  PAL_PORT(LINE_SPI1_CS)
//#define SPI_SELECT_SLAVE0_PIN   PAL_PAD(LINE_SPI1_CS)


// Internal SPI (OSD)
#define SPI2_GPIO_AF          AF_SPI2_CLK
#define SPI2_GPIO_PORT_MISO   PAL_PORT(LINE_SPI2_MISO)
#define SPI2_GPIO_MISO        PAL_PAD(LINE_SPI2_MISO)
#define SPI2_GPIO_PORT_MOSI   PAL_PORT(LINE_SPI2_MOSI)
#define SPI2_GPIO_MOSI        PAL_PAD(LINE_SPI2_MOSI)
#define SPI2_GPIO_PORT_SCK    PAL_PORT(LINE_SPI2_CLK)
#define SPI2_GPIO_SCK         PAL_PAD(LINE_SPI2_CLK)
// SLAVE2 on SPI2 (OSD)
#define SPI_SELECT_SLAVE2_PORT  PAL_PORT(LINE_SPI2_CS)
#define SPI_SELECT_SLAVE2_PIN   PAL_PAD(LINE_SPI2_CS)


// EXTERNAL SPI3 
#define SPI3_GPIO_AF          AF_SPI3_CLK
#define SPI3_GPIO_PORT_MISO   PAL_PORT(LINE_SPI3_MISO)
#define SPI3_GPIO_MISO        PAL_PAD(LINE_SPI3_MISO)
#define SPI3_GPIO_PORT_MOSI   PAL_PORT(LINE_SPI3_MOSI)
#define SPI3_GPIO_MOSI        PAL_PAD(LINE_SPI3_MOSI)
#define SPI3_GPIO_PORT_SCK    PAL_PORT(LINE_SPI3_CLK)
#define SPI3_GPIO_SCK         PAL_PAD(LINE_SPI3_CLK)
// SLAVE3 on SPI3 
#define SPI_SELECT_SLAVE3_PORT  PAL_PORT(LINE_SPI3_CS)
#define SPI_SELECT_SLAVE3_PIN   PAL_PAD(LINE_SPI3_CS)

// internal SPI4 (IMU2)
#define SPI4_GPIO_AF          AF_SPI4_CLK 
#define SPI4_GPIO_PORT_MISO   PAL_PORT(LINE_SPI4_MISO)
#define SPI4_GPIO_MISO        PAL_PAD(LINE_SPI4_MISO)
#define SPI4_GPIO_PORT_MOSI   PAL_PORT(LINE_SPI4_MOSI)
#define SPI4_GPIO_MOSI        PAL_PAD(LINE_SPI4_MOSI)
#define SPI4_GPIO_PORT_SCK    PAL_PORT(LINE_SPI4_CLK)
#define SPI4_GPIO_SCK         PAL_PAD(LINE_SPI4_CLK)
// SLAVE4 on SPI4 (IMU2)
#define SPI_SELECT_SLAVE4_PORT  PAL_PORT(LINE_SPI4_CS)
#define SPI_SELECT_SLAVE4_PIN   PAL_PAD(LINE_SPI4_CS)


/**
 * Baro
 *
 * Apparently needed for backwards compatibility
 * with the ancient onboard baro boards
 */
#ifndef USE_BARO_BOARD
#define USE_BARO_BOARD 0
#endif

/**
 * SDIO
 */
#define SDIO_D0_PORT  PAL_PORT(LINE_SDMMC1_D0)
#define SDIO_D0_PIN   PAL_PAD(LINE_SDMMC1_D0)
#define SDIO_D1_PORT  PAL_PORT(LINE_SDMMC1_D1)
#define SDIO_D1_PIN   PAL_PAD(LINE_SDMMC1_D1)
#define SDIO_D2_PORT  PAL_PORT(LINE_SDMMC1_D2)
#define SDIO_D2_PIN   PAL_PAD(LINE_SDMMC1_D2)
#define SDIO_D3_PORT  PAL_PORT(LINE_SDMMC1_D3)
#define SDIO_D3_PIN   PAL_PAD(LINE_SDMMC1_D3)
#define SDIO_CK_PORT  PAL_PORT(LINE_SDMMC1_CK)
#define SDIO_CK_PIN   PAL_PAD(LINE_SDMMC1_CK)
#define SDIO_CMD_PORT PAL_PORT(LINE_SDMMC1_CMD)
#define SDIO_CMD_PIN  PAL_PAD(LINE_SDMMC1_CMD)
#define SDIO_AF       AF_SDMMC1_CK
// bat monitoring for file closing
#define SDLOG_BAT_ADC CONCAT_BOARD_PARAM(ADCD, VBAT_MEAS_ADC)
#define SDLOG_BAT_CHAN CONCAT_BOARD_PARAM(ADC_CHANNEL_IN, VBAT_MEAS_ADC_IN)
// usb led status
#define SDLOG_USB_LED 2
//#define SDLOG_USB_VBUS_PORT PAL_PORT(LINE_USB_VBUS) // FIXME
//#define SDLOG_USB_VBUS_PIN  PAL_PAD(LINE_USB_VBUS)


/*
 * Actuators for fixedwing
 */
/* Default actuators driver */
#define DEFAULT_ACTUATORS "subsystems/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()

/**
 * For WS2812
 */
#define WS2812D1_GPIO PAL_PORT(LINE_LED_WS2812)
#define WS2812D1_PIN  PAL_PAD(LINE_LED_WS2812)
#define WS2812D1_AF AF_LED_WS2812
#define WS2812D1_CFG_DEF { \
    .dma_stream = STM32_PWM1_UP_DMA_STREAM, \
                  .dma_channel = STM32_PWM1_UP_DMA_CHANNEL, \
                                 .dma_priority = STM32_PWM1_UP_DMA_PRIORITY, \
                                     .pwm_channel = 0, \
                                         .pwmp = &PWMD1 \
  }

#endif /* CONFIG_MATEK_F765_WING_H */

