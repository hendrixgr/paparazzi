/*
 *
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */


#ifndef CONFIG_OMNIBUS_F4_AIO_V2_H
#define CONFIG_OMNIBUS_F4_AIO_V2_H

#define BOARD_OMNIBUS_F4_AIO

/* OpenPilot Revo has a 8MHz external clock and 168MHz internal. */
#define EXT_CLK 8000000
#define AHB_CLK 168000000

#define PPM_IN 1
#define PPM_IN_ALT 2
#define RSSI_PPM_IN 3


//****************************************************************************************
// Onboard LEDs

#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOB
#define LED_1_GPIO_PIN GPIO5
#define LED_1_GPIO_ON gpio_clear
#define LED_1_GPIO_OFF gpio_set
#define LED_1_AFIO_REMAP ((void)0)

//****************************************************************************************

/* Default actuators driver */
#define DEFAULT_ACTUATORS "subsystems/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()


//****************************************************************************************
// UART
//CAN BE USED AS GPS SERIAL PORT, DSM,DSMX,IBUS rx port.
#define UART1_GPIO_AF GPIO_AF7
#define UART1_GPIO_PORT_TX GPIOA
#define UART1_GPIO_TX GPIO9
#define UART1_GPIO_PORT_RX GPIOA
#define UART1_GPIO_RX GPIO10

// SPARE IT CAN BE CONFIGURED AS I2C2
#if !defined(USE_I2C2)
#define UART3_GPIO_AF GPIO_AF7
#define UART3_GPIO_PORT_TX GPIOB
#define UART3_GPIO_TX GPIO10
#define UART3_GPIO_PORT_RX GPIOB
#define UART3_GPIO_RX GPIO11
#endif

// CAN BE USED AS MODEM PORT, SBUS INVERTER IS CONNECTED ON RX PIN AND IT IS CONTROLLED WITH PC8 IO PIN
#define UART6_GPIO_AF GPIO_AF8
#define UART6_GPIO_PORT_TX GPIOC
#define UART6_GPIO_TX GPIO6
#define UART6_GPIO_PORT_RX GPIOC
#define UART6_GPIO_RX GPIO7


//****************************************************************************************
/* SPI */
/* MPU6000 */
#if !defined(USE_SP1)
#define USE_SPI1 1
#endif

#define SPI1_GPIO_AF GPIO_AF5
#define SPI1_GPIO_PORT_SCK GPIOA
#define SPI1_GPIO_SCK GPIO5
#define SPI1_GPIO_PORT_MISO GPIOA
#define SPI1_GPIO_MISO GPIO6
#define SPI1_GPIO_PORT_MOSI GPIOA
#define SPI1_GPIO_MOSI GPIO7
// MPU6000 CHIP SELECT PIN
#define SPI1_GPIO_PORT_NSS GPIOA
#define SPI1_GPIO_NSS GPIO4
// MPU6000 ALTERNATE SELECT DEFINITION
#define SPI_SELECT_SLAVE0_PORT SPI1_GPIO_PORT_NSS
#define SPI_SELECT_SLAVE0_PIN SPI1_GPIO_NSS


#if !defined(USE_SP2)
//#define USE_SPI2 1
#endif
// flash chip
#define SPI2_GPIO_AF GPIO_AF5
#define SPI2_GPIO_PORT_SCK GPIOB
#define SPI2_GPIO_SCK GPIO13
#define SPI2_GPIO_PORT_MISO GPIOB
#define SPI2_GPIO_MISO GPIO14
#define SPI2_GPIO_PORT_MOSI GPIOB
#define SPI2_GPIO_MOSI GPIO15
// flash chip select 
#define SPI2_GPIO_PORT_NSS GPIOB
#define SPI2_GPIO_NSS GPIO12
// FLASH CHIP ALTERNATE SELECT DEFINITION
#define SPI_SELECT_SLAVE1_PORT SPI2_GPIO_PORT_NSS
#define SPI_SELECT_SLAVE1_PIN SPI2_GPIO_NSS
#define SPI_SELECT_SLAVE3_PORT GPIOA
#define SPI_SELECT_SLAVE3_PIN GPIO0



// OSD 7456
#if !defined(USE_SPI3)
#define USE_SPI3 1
#endif

#define SPI3_GPIO_AF GPIO_AF6
#define SPI3_GPIO_PORT_SCK GPIOC
#define SPI3_GPIO_SCK GPIO10
#define SPI3_GPIO_PORT_MISO GPIOC
#define SPI3_GPIO_MISO GPIO11
#define SPI3_GPIO_PORT_MOSI GPIOC
#define SPI3_GPIO_MOSI GPIO12
// OSD CHIP SELECT PIN
#define SPI3_GPIO_PORT_NSS GPIOA
#define SPI3_GPIO_NSS GPIO15
// OSD7456 ALTERNATE SELECT DEFINITION
#define SPI_SELECT_SLAVE2_PORT SPI3_GPIO_PORT_NSS
#define SPI_SELECT_SLAVE2_PIN SPI3_GPIO_NSS

//****************************************************************************************
//  I2C
// CANNOT BE USED IF UART3 IS ACTIVE!
//SDA ON UART3 TX PIN, SCL ON UART3 RX PIN
// Be sure to fit pull up resistors because the board does not have any!
#if !defined(USE_UART3)
#define I2C2_GPIO_PORT GPIOB
#define I2C2_GPIO_SCL  GPIO10
#define I2C2_GPIO_SDA  GPIO11
// ALL I2C PINS ARE ON AF4 SO THE BELOW DEFINITION IS NOT NEEDED ACTUALLY...
//#define I2C2_GPIO_AF GPIO_AF4
#endif

//SCL ON PWM6 and SDA ON BUZZER + OR - NEEDS CHECKING!
// Be sure to fit pull up resistors because the board does not have any!
#if !defined(USE_PWM6) || USE_PWM6 == 0
#define I2C3_GPIO_PORT_SCL GPIOA
#define I2C3_GPIO_SCL GPIO8
#define I2C3_GPIO_PORT_SDA GPIOC
#define I2C3_GPIO_SDA GPIO9
// ALL I2C PINS ARE ON AF4 SO THE BELOW DEFINITION IS NOT NEEDED ACTUALLY...
//#define I2C3_GPIO_AF GPIO_AF4
#endif

// Onboard ADCs */
// provide defines that can be used to access the ADC_x in the code or airframe file
// these directly map to the index number of the 4 adc channels defined above
// 4th (index 3) is used for bat monitoring by default
//

#define USE_AD_TIM9 1

#ifndef USE_ADC_1
#define USE_ADC_1 1
#endif

#ifndef USE_ADC_2
#define USE_ADC_2 1
#endif

// used for ONBOARD input voltage measurement, RATIO 11 TO 1 (10k->1k)
#if USE_ADC_1
#define AD1_1_CHANNEL 12
#define ADC_1 AD1_1
#define ADC_1_GPIO_PORT GPIOC
#define ADC_1_GPIO_PIN GPIO2
/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY ADC_1
#endif
#endif

// used for ONBOARD current measurement.
#if USE_ADC_2
#define AD1_2_CHANNEL 11
#define ADC_2 AD1_2
#define ADC_2_GPIO_PORT GPIOC
#define ADC_2_GPIO_PIN GPIO1
#ifndef CURRENT_ADC_IN
#define CURRENT_ADC_IN ADC_2
#endif
#endif

// SPARE ADC ON RSSI
#if USE_ADC_3
#if RADIO_CONTROL_PPM_PIN != RSSI_IN
#define AD1_3_CHANNEL 0
#define ADC_3 AD1_3
#define ADC_3_GPIO_PORT GPIOA
#define ADC_3_GPIO_PIN GPIO0
#else
#error "ADC 3 IS USED AS PPM INPUT"
#endif
#endif

#define DefaultVoltageOfAdc(adc) (0.008830925*adc)
#define DefaultMilliAmpereOfAdc(adc) (25*adc)


#if RADIO_CONTROL_PPM_PIN == PPM_IN
// PPM input MARKED AS PPM
#define USE_PPM_TIM4 1
#define PPM_CHANNEL         TIM_IC3
#define PPM_TIMER_INPUT     TIM_IC_IN_TI3
#define PPM_IRQ             NVIC_TIM4_IRQ // NVIC_TIM1_UP_TIM10_IRQ for TIM10
#define PPM_IRQ2            NVIC_TIM4_IRQ // NVIC_TIM1_UP_TIM10_IRQ for TIM10
// Capture/Compare InteruptEnable and InterruptFlag
#define PPM_CC_IE           TIM_DIER_CC3IE
#define PPM_CC_IF           TIM_SR_CC3IF
#define PPM_GPIO_PORT       GPIOB
#define PPM_GPIO_PIN        GPIO8
#define PPM_GPIO_AF         GPIO_AF2 //AF2 AND AF3 AVALIALBE USING TIM4 ch3 OR  TIM10 ch1 respectively!

#elif RADIO_CONTROL_PPM_PIN == PPM_IN_ALT

// PPM input MARKED AS PPM, used with TIM10 which let TIM4 free and can be used for PWM.
#define USE_PPM_TIM10 1
#define PPM_CHANNEL         TIM_IC1
#define PPM_TIMER_INPUT     TIM_IC_IN_TI1
#define PPM_IRQ             NVIC_TIM1_UP_TIM10_IRQ
#define PPM_IRQ2            NVIC_TIM1_UP_TIM10_IRQ
// Capture/Compare InteruptEnable and InterruptFlag
#define PPM_CC_IE           TIM_DIER_CC1IE
#define PPM_CC_IF           TIM_SR_CC1IF
#define PPM_GPIO_PORT       GPIOB
#define PPM_GPIO_PIN        GPIO8
#define PPM_GPIO_AF         GPIO_AF2 //AF3 AND AF2 AVALIALBE USING TIM10 ch1 OR TIM4 ch3 respectively!

#elif RADIO_CONTROL_PPM_PIN == RSSI_PPM_IN

// PPM input ON RSSI PAD
#define USE_PPM_TIM5 1
#define PPM_CHANNEL         TIM_IC1
#define PPM_TIMER_INPUT     TIM_IC_IN_TI1
#define PPM_IRQ             NVIC_TIM5_IRQ
#define PPM_IRQ2            NVIC_TIM5_IRQ
// Capture/Compare InteruptEnable and InterruptFlag
#define PPM_CC_IE           TIM_DIER_CC1IE
#define PPM_CC_IF           TIM_SR_CC1IF
#define PPM_GPIO_PORT       GPIOA
#define PPM_GPIO_PIN        GPIO0
#define PPM_GPIO_AF         GPIO_AF2 

#else
#error "PPM INPUT PIN NOT SELECTED"

#endif

// SERVO DEFINITIONS
#define PWM_USE_TIM3  1
#define PWM_USE_TIM4  0
#define PWM_USE_TIM5  1

// PWM SERVO
#define USE_PWM1 1
#define USE_PWM2 1
#define USE_PWM3 1
#define USE_PWM4 1
#define USE_PWM5 1


// PWM_SERVO_x is the index of the servo in the actuators_pwm_values array
#if USE_PWM1
#define PWM_SERVO_1 1
#define PWM_SERVO_1_TIMER TIM3
#define PWM_SERVO_1_GPIO GPIOB
#define PWM_SERVO_1_PIN GPIO0
#define PWM_SERVO_1_AF GPIO_AF2
#define PWM_SERVO_1_OC TIM_OC3
#define PWM_SERVO_1_OC_BIT (1<<2)
#else
#define PWM_SERVO_1_OC_BIT 0
#endif

#if USE_PWM2
#define PWM_SERVO_2 1
#define PWM_SERVO_2_TIMER TIM3
#define PWM_SERVO_2_GPIO GPIOB
#define PWM_SERVO_2_PIN GPIO1
#define PWM_SERVO_2_AF GPIO_AF2
#define PWM_SERVO_2_OC TIM_OC4
#define PWM_SERVO_2_OC_BIT (1<<3)
#else
#define PWM_SERVO_2_OC_BIT 0
#endif

#if USE_PWM3
#define PWM_SERVO_3 1
#define PWM_SERVO_3_TIMER TIM5
#define PWM_SERVO_3_GPIO GPIOA
#define PWM_SERVO_3_PIN GPIO3
#define PWM_SERVO_3_AF GPIO_AF2
#define PWM_SERVO_3_OC TIM_OC4
#define PWM_SERVO_3_OC_BIT (1<<3)
#else
#define PWM_SERVO_3_OC_BIT 0
#endif

#if USE_PWM4
#define PWM_SERVO_4 1
#define PWM_SERVO_4_TIMER TIM5
#define PWM_SERVO_4_GPIO GPIOA
#define PWM_SERVO_4_PIN GPIO2
#define PWM_SERVO_4_AF GPIO_AF2
#define PWM_SERVO_4_OC TIM_OC3
#define PWM_SERVO_4_OC_BIT (1<<2)
#else
#define PWM_SERVO_4_OC_BIT 0
#endif

#if USE_PWM5 // FIXME MARKED AS BUZZER+ OR BUZZER-???? 
#define PWM_SERVO_5 1
#define PWM_SERVO_5_TIMER TIM3
#define PWM_SERVO_5_GPIO GPIOB
#define PWM_SERVO_5_PIN GPIO4
#define PWM_SERVO_5_AF GPIO_AF2
#define PWM_SERVO_5_OC TIM_OC1
#define PWM_SERVO_5_OC_BIT (1<<0)
#else
#define PWM_SERVO_5_OC_BIT 0
#endif

#if USE_PWM6 // MARKED AS LED
#define PWM_SERVO_6 1
#define PWM_SERVO_6_TIMER TIM4
#define PWM_SERVO_6_GPIO GPIOB
#define PWM_SERVO_6_PIN GPIO6
#define PWM_SERVO_6_AF GPIO_AF2
#define PWM_SERVO_6_OC TIM_OC1
#define PWM_SERVO_6_OC_BIT (1<<0)
#else
#define PWM_SERVO_6_OC_BIT 0
#endif

#define PWM_TIM3_CHAN_MASK (PWM_SERVO_1_OC_BIT | PWM_SERVO_2_OC_BIT | PWM_SERVO_5_OC_BIT)
#define PWM_TIM4_CHAN_MASK (PWM_SERVO_6_OC_BIT)
#define PWM_TIM5_CHAN_MASK (PWM_SERVO_3_OC_BIT | PWM_SERVO_4_OC_BIT)


#endif // CONFIG_OMNIBUS_V6_AIO_V1_H 
