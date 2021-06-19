/*
 * Copyright (C) 2011-2013 The Paparazzi Team
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

/**
 * @file modules/rfm_rc_modem/rfm_rc_modem.c
 * V1.00 21 of May 2021
 */

//###################################################################################################
//                          I N C L U D E D   H E A D E R   F I L E S
//###################################################################################################
#include "rfm_regs.h"
#include "rfm.h"
#include "math/pprz_algebra_int.h"
#include "mcu_periph/i2c.h"
#include "mcu_periph/sys_time.h"

#include "math/pprz_isa.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/abi.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"
#if DOWNLINK
#include "subsystems/datalink/telemetry.h"
#endif
#include "flash_functions.h"
/*
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/usart.h>
*/
#include "mcu_periph/uart.h"
#include "mcu_periph/gpio.h"


#include "std.h"

#include "subsystems/radio_control.h"
#include "subsystems/radio_control/ppm.h"

#include "ch.h"
#include "hal.h"



//###################################################################################################
//          P R E P R O C E S S O R   D I R E C T I V E S   A N D   D E F I N I T I O N S
//###################################################################################################
// BIG ENDIAN DEFINITIONS
//#define BUF2INT(_buf,_idx) ((int16_t)((_buf[_idx]<<8) | _buf[_idx+1]))
//#define INT2BUF(_int,_buf,_idx) { _buf[_idx] = (_int>>8); _buf[_idx+1] = _int; }



#define USE_DUMMY_MODEM_UART  1

#ifndef RFM_SLAVE_IDX
#define RFM_SLAVE_IDX SPI_SLAVE0
#endif

#ifndef RFM_SPI_DEV
#define RFM_SPI_DEV spi1
#endif

#define AVAILABLE    0
#define TRANSMIT    1
#define TRANSMITTED  2
#define RECEIVE    3
#define RECEIVED  4

// Version number in single uint16 [8bit major][4bit][4bit]
// a.b.c == 0xaabc
#define OPENLRSNG_VERSION 0x0390

#define RX_FLYTRON8CH 0x01
#define RX_OLRSNG4CH  0x02
#define RX_OLRSNG12CH 0x03
#define RX_DTFUHF10CH 0x04
#define RX_PTOWER     0x05
#define RX_MICRO      0x06
#define RX_FLYTRONM3  0x07
#define RX_BRORX      0x08

#define PINMAP_PPM    0x20
#define PINMAP_RSSI   0x21
#define PINMAP_SDA    0x22
#define PINMAP_SCL    0x23
#define PINMAP_RXD    0x24
#define PINMAP_TXD    0x25
#define PINMAP_ANALOG 0x26
#define PINMAP_LBEEP  0x27 // packetloss beeper
#define PINMAP_SPKTRM 0x28 // spektrum satellite output
#define PINMAP_SBUS   0x29 // SBUS output
#define PINMAP_SUMD   0x2a // SUMD output
#define PINMAP_LLIND  0x2b // LinkLoss indication (digital output)

// OpenLRSng binding

// Factory setting values, modify via the CLI

//####### RADIOLINK RF POWER (beacon is always 100/13/1.3mW) #######
// 7 == 100mW (or 1000mW with M3)
// 6 == 50mW (use this when using booster amp), (800mW with M3)
// 5 == 25mW
// 4 == 13mW
// 3 == 6mW
// 2 == 3mW
// 1 == 1.6mW
// 0 == 1.3mW
#define DEFAULT_RF_POWER 7

#define DEFAULT_CHANNEL_SPACING 5 // 50kHz
#define DEFAULT_HOPLIST 22,10,19,34,49,41
#define DEFAULT_RF_MAGIC 0x021221963

//  0 -- 4800bps, best range
//  1 -- 9600bps, medium range
//  2 -- 19200bps, medium range
#define DEFAULT_DATARATE 2

#define DEFAULT_BAUDRATE 115200

// RX_CONFIG flag masks
#define PPM_MAX_8CH         0x01
#define ALWAYS_BIND         0x02
#define SLAVE_MODE          0x04
#define IMMEDIATE_OUTPUT    0x08
#define STATIC_BEACON       0x10
#define INVERTED_PPMOUT     0x40
#define WATCHDOG_USED       0x80 // read only flag, only sent to configurator

// BIND_DATA flag masks
#define TELEMETRY_OFF       0x00
#define TELEMETRY_PASSTHRU  0x08
#define TELEMETRY_FRSKY     0x10 // covers smartport if used with &
#define TELEMETRY_SMARTPORT 0x18
#define TELEMETRY_MASK      0x18
#define CHANNELS_4_4        0x01
#define CHANNELS_8          0x02
#define CHANNELS_8_4        0x03
#define CHANNELS_12         0x04
#define CHANNELS_12_4       0x05
#define CHANNELS_16         0x06
#define DIVERSITY_ENABLED   0x80
#define DEFAULT_FLAGS       (CHANNELS_8 | TELEMETRY_PASSTHRU)

#define MULTI_OPERATION_TIMEOUT_MS 5000

// helper macro for European PMR channels
#define EU_PMR_CH(x) (445993750L + 12500L * (x)) // valid for ch1-ch16 (Jan 2016  ECC update)

// helper macro for US FRS channels 1-7
#define US_FRS_CH(x) (462537500L + 25000L * (x)) // valid for ch1-ch7

#define DEFAULT_BEACON_FREQUENCY 0 // disable beacon
#define DEFAULT_BEACON_DEADTIME 30 // time to wait until go into beacon mode (30s)
#define DEFAULT_BEACON_INTERVAL 10 // interval between beacon transmits (10s)

#define MIN_DEADTIME 0
#define MAX_DEADTIME 255

#define MIN_INTERVAL 1
#define MAX_INTERVAL 255

#define BINDING_POWER     0x06 // not lowest since may result fail with RFM23BP

#define TELEMETRY_PACKETSIZE 9
#define MAX_PACKETSIZE 21

#define BIND_MAGIC (0xDEC1BE15 + (OPENLRSNG_VERSION & 0xfff0))
#define BINDING_VERSION ((OPENLRSNG_VERSION & 0x0ff0)>>4)

// HW frequency limits
#if (RFMTYPE == 868)
#  define MIN_RFM_FREQUENCY 848000000
#  define MAX_RFM_FREQUENCY 888000000
#  define DEFAULT_CARRIER_FREQUENCY 868000000  // Hz  (ch 0)
#  define BINDING_FREQUENCY 868000000 // Hz
#elif (RFMTYPE == 915)
#  define MIN_RFM_FREQUENCY 895000000
#  define MAX_RFM_FREQUENCY 935000000
#  define DEFAULT_CARRIER_FREQUENCY 915000000  // Hz  (ch 0)
#  define BINDING_FREQUENCY 915000000 // Hz
#else
#  define MIN_RFM_FREQUENCY 413000000
#  define MAX_RFM_FREQUENCY 463000000
#  define DEFAULT_CARRIER_FREQUENCY 435000000  // Hz  (ch 0)
#  define BINDING_FREQUENCY 435000000 // Hz
#endif

#define MAXHOPS      24
#define PPM_CHANNELS 16
#define SERIAL_BUFSIZE 32



//###################################################################################################
//                               G L O B A L   V A R I A B L E S
//###################################################################################################

enum{
       RFM_STATUS_UNINIT,
       RFM_STATUS_INITIALIZED,
       RFM_STATUS_WAITING_BYTE,
       RFM_STATUS_FINISHED
    };

volatile bool isr_packet_tranmitted = false;
volatile bool isr_packet_received = false;
bool willhop = 0;

uint8_t rfm_status = RFM_STATUS_UNINIT;
uint8_t RF_channel = 0;
uint8_t  RSSI_count = 0;
uint8_t  lastRSSIvalue = 0;
uint8_t  smoothRSSI = 0;
uint8_t  compositeRSSI = 0;
uint8_t linkAcquired = 0;
uint8_t numberOfLostPackets = 0;
uint8_t hopcount;
uint8_t  linkQ;
volatile uint8_t disablePPM = 0;
volatile uint8_t RF_Mode = 0;

uint8_t serial_head;
uint8_t serial_tail;
uint8_t serial_buffer[SERIAL_BUFSIZE];
uint8_t default_hop_list[] = {DEFAULT_HOPLIST};
char rfm_spi_tx_buffer[8];
char rfm_spi_rx_buffer[8];
const uint8_t pktsizes[8] = { 0, 7, 11, 12, 16, 17, 21, 0 };

uint8_t rx_buf[21]; // RX buffer (uplink)
// First byte of RX buffer is metadata
// MSB..LSB [1bit uplink seqno.] [1bit downlink seqno] [6bits type]
// type 0x38..0x3f uplinked serial data
// type 0x00 normal servo, 0x01 failsafe set

uint8_t tx_buf[9]; // TX buffer (downlink)(type plus 8 x data)
// First byte of TX buffer is metadata
// MSB..LSB [1 bit uplink seq] [1bit downlink seqno] [6b telemtype]
// 0x00 link info [RSSI] [AFCC]*2 etc...
// type 0x38-0x3f downlink serial data 1-8 bytes

uint16_t lastAFCCvalue = 0;
uint16_t RSSI_sum = 0;
uint16_t linkQuality = 0;
uint16_t version = OPENLRSNG_VERSION;
uint16_t CRC16_value;
volatile uint16_t PPM[PPM_CHANNELS] = { 512, 512, 512, 512, 512, 512, 512, 512 , 512, 512, 512, 512, 512, 512, 512, 512 };
const uint16_t switchThresholds[3] = { 178, 500, 844 };

uint32_t lastPacketTimeUs = 0;
uint32_t lastRSSITimeUs = 0;
uint32_t linkLossTimeMs;
uint32_t hopInterval = 0;
uint32_t hopTimeout = 0;
uint32_t hopTimeoutSlow = 0;
uint32_t RSSI_timeout = 0;
uint32_t pktTimeDelta = 0;
uint32_t timeUs = 0;
uint32_t timeMs= 0;
uint32_t tx_start = 0;
volatile uint32_t lastReceived = 0;

/*
struct rxSpecialPinMap rxSpecialPins[] = {
  { 0, PINMAP_PPM},
  { 1, PINMAP_SDA},
  { 1, PINMAP_ANALOG}, // AIN0
  { 2, PINMAP_RSSI},
  { 2, PINMAP_LBEEP},
  { 3, PINMAP_SCL},
  { 3, PINMAP_ANALOG}, // AIN1
  { 4, PINMAP_ANALOG},
  { 5, PINMAP_ANALOG},
  { 5, PINMAP_LLIND},
  { 6, PINMAP_RXD},
  { 7, PINMAP_TXD},
  { 7, PINMAP_SPKTRM},
  { 7, PINMAP_SBUS},
  { 7, PINMAP_SUMD},
};
*/
// The Rx configuration structure is defined but it is not used used since the configuration for use with paparazzi is specific.
// 27 bytes, we use packing because if we allow padding the structure will have different size in 32bit cpus from that of AVRs
struct __attribute__((__packed__)) RX_config {
  uint8_t  rx_type; // RX type fillled in by RX, do not change
  uint8_t  pinMapping[13];
  uint8_t  flags;
  uint8_t  RSSIpwm; //0-15 inject composite, 16-31 inject quality, 32-47 inject RSSI, 48-63 inject quality & RSSI on two separate channels
  uint32_t beacon_frequency;
  uint8_t  beacon_deadtime;
  uint8_t  beacon_interval;
  uint16_t minsync;
  uint8_t  failsafeDelay;
  uint8_t  ppmStopDelay;
  uint8_t  pwmStopDelay;
} rx_config;

// 18 bytes, we use packing because if we allow padding the structure will have different size in 32bit cpus from that of AVRs
struct __attribute__((__packed__)) Bind_Data {
  uint8_t version;
  uint32_t serial_baudrate;
  uint32_t rf_frequency;
  uint32_t rf_magic;
  uint8_t rf_power;
  uint8_t rf_channel_spacing;
  uint8_t hopchannel[MAXHOPS];
  uint8_t modem_datarate;
  uint8_t flags;
} bind_data;


struct rfm_modem_regs {
  uint32_t bps;
  uint8_t  r_1c, r_1d, r_1e, r_20, r_21, r_22, r_23, r_24, r_25, r_2a, r_6e, r_6f, r_70, r_71, r_72;
} modem_params[] = {
  { 4800, 0x1a, 0x40, 0x0a, 0xa1, 0x20, 0x4e, 0xa5, 0x00, 0x1b, 0x1e, 0x27, 0x52, 0x2c, 0x23, 0x30 }, // 50000 0x00
  { 9600, 0x05, 0x40, 0x0a, 0xa1, 0x20, 0x4e, 0xa5, 0x00, 0x20, 0x24, 0x4e, 0xa5, 0x2c, 0x23, 0x30 }, // 25000 0x00
  { 19200, 0x06, 0x40, 0x0a, 0xd0, 0x00, 0x9d, 0x49, 0x00, 0x7b, 0x28, 0x9d, 0x49, 0x2c, 0x23, 0x30 }, // 25000 0x01
  { 57600, 0x05, 0x40, 0x0a, 0x45, 0x01, 0xd7, 0xdc, 0x03, 0xb8, 0x1e, 0x0e, 0xbf, 0x00, 0x23, 0x2e },
  { 125000, 0x8a, 0x40, 0x0a, 0x60, 0x01, 0x55, 0x55, 0x02, 0xad, 0x1e, 0x20, 0x00, 0x00, 0x23, 0xc8 },
};

#define DATARATE_COUNT (sizeof(modem_params) / sizeof(modem_params[0]))

struct rfm_modem_regs binding_modem_datarate =
{ 9600, 0x05, 0x40, 0x0a, 0xa1, 0x20, 0x4e, 0xa5, 0x00, 0x20, 0x24, 0x4e, 0xa5, 0x2c, 0x23, 0x30 };


uint32_t flash_data[64];

struct spi_transaction rfm_trans;




//###################################################################################################
//                         P R I V A T E   F U N C T I O N   D E F I N I T I O N S
//###################################################################################################

void init_rfm(uint8_t isbind);
uint8_t rfmGetGPIO1(void);
void updateHopChannel(void);
void setHopChannel(uint8_t ch);
void check_module(void);
void rfmSetHeader(uint8_t iHdr, uint8_t bHdr);
void rfmSetChannel(uint8_t ch);
void CRC16_add(uint8_t c);
void rfmSetCarrierFrequency(uint32_t f);
void rfmSetModemRegs(struct rfm_modem_regs* r);
void checkLinkState(void);
void checkRSSI(void);
void tx_packet(uint8_t* pkt, uint8_t size);
void tx_packet_async(uint8_t* pkt, uint8_t size);
void rx_reset(void);
void tx_reset(void);
void unpackChannels(uint8_t config, volatile uint16_t PPM_VALUES[], uint8_t *p);
void handlePacketRX(void);
void rfmClearInterrupts(void);
void rfmGetPacket(uint8_t *buf, uint8_t size);
void rfmSetTX(void);
void rfmClearIntStatus(void);

bool spiReadRegister(uint8_t rfm_reg);
bool spiWriteRegister(uint8_t rfm_reg, uint8_t rfm_reg_val);

uint8_t countSetBits(uint16_t x);
uint8_t getPacketSize(struct Bind_Data *ps);
uint8_t bindReceive(uint32_t timeout);
uint8_t rfmGetRSSI(void);
uint8_t rfmGetPacketLength(void);
uint8_t tx_done(void);

uint16_t rfmGetAFCC(void);

uint32_t millis(void);
uint32_t micros(void);
uint32_t getInterval(struct Bind_Data *ps);

void exti9_5_isr(void);
#if defined(STM32F7) || defined(STM32H7)
static void chibios_ext_isr_cb(void *arg);
#endif

//###################################################################################################
//                        P R I V A T E   F U N C T I O N   P R O T O T Y P E S
//###################################################################################################

void init_rfm(uint8_t isbind)
{
  rx_config.rx_type = RX_OLRSNG4CH;

  //rfmSetReadyMode(); // turn on the XTAL and give it time to settle
  spiWriteRegister(RFM22B_OPMODE1, RFM22B_OPMODE_READY);

  //delayMicroseconds(600);

  rfmClearIntStatus();

  //rfmInit(bind_data.flags&DIVERSITY_ENABLED);
  bool diversity = false; // we will not use diversity in this application.
  spiWriteRegister(RFM22B_INTEN2, 0x00);    // disable interrupts
  spiWriteRegister(RFM22B_INTEN1, 0x00);    // disable interrupts
  spiWriteRegister(RFM22B_XTALCAP, 0x7F);   // XTAL cap = 12.5pF
  spiWriteRegister(RFM22B_MCUCLK, 0x05);    // 2MHz clock

  spiWriteRegister(RFM22B_GPIOCFG2, (diversity ? 0x17 : 0xFD) ); // gpio 2 ant. sw, 1 if diversity on else VDD
  spiWriteRegister(RFM22B_PREAMLEN, (diversity ? 0x14 : 0x0A) );    // 40 bit preamble, 80 with diversity
  spiWriteRegister(RFM22B_IOPRTCFG, 0x00);    // gpio 0,1,2 NO OTHER FUNCTION.

  #ifdef SWAP_GPIOS
  spiWriteRegister(RFM22B_GPIOCFG0, 0x15);    // gpio0 RX State
  spiWriteRegister(RFM22B_GPIOCFG1, 0x12);    // gpio1 TX State
  #else
  spiWriteRegister(RFM22B_GPIOCFG0, 0x12);    // gpio0 TX State
  spiWriteRegister(RFM22B_GPIOCFG1, 0x15);    // gpio1 RX State
  #endif

  // Packet settings
  spiWriteRegister(RFM22B_DACTL, 0x8C);    // enable packet handler, msb first, enable crc,
  spiWriteRegister(RFM22B_HDRCTL1, 0x0F);    // no broadcast, check header bytes 3,2,1,0
  spiWriteRegister(RFM22B_HDRCTL2, 0x42);    // 4 byte header, 2 byte sync, variable packet size
  spiWriteRegister(RFM22B_PREATH, 0x2A);    // preamble detect = 5 (20bits), rssioff = 2
  spiWriteRegister(RFM22B_SYNC3, 0x2D);    // sync word 3
  spiWriteRegister(RFM22B_SYNC2, 0xD4);    // sync word 2
  spiWriteRegister(RFM22B_SYNC1, 0x00);    // sync word 1 (not used)
  spiWriteRegister(RFM22B_SYNC0, 0x00);    // sync word 0 (not used)
  spiWriteRegister(RFM22B_HDREN3, 0xFF);    // must set all bits
  spiWriteRegister(RFM22B_HDREN2, 0xFF);    // must set all bits
  spiWriteRegister(RFM22B_HDREN1, 0xFF);    // must set all bits
  spiWriteRegister(RFM22B_HDREN0, 0xFF);    // must set all bits

  spiWriteRegister(RFM22B_FREQOFF1, 0x00);    // no offset
  spiWriteRegister(RFM22B_FREQOFF2, 0x00);    // no offset
  spiWriteRegister(RFM22B_FHCH,     0x00);   // set to hop channel 0


  //rfmSetStepSize(bind_data.rf_channel_spacing);
  spiWriteRegister(RFM22B_FHS, bind_data.rf_channel_spacing);
  uint32_t magic = isbind ? BIND_MAGIC : bind_data.rf_magic;
  for (uint8_t i = 0; i < 4; i++) {
    //rfmSetHeader(i, (magic >> 24) & 0xff);
    spiWriteRegister(RFM22B_TXHDR3+i, ((magic >> 24) & 0xff));
    spiWriteRegister(RFM22B_CHKHDR3+i, ((magic >> 24) & 0xff));
    magic = magic << 8; // advance to next byte
  }

  if (isbind) {
    rfmSetModemRegs(&binding_modem_datarate);
    //rfmSetPower(BINDING_POWER);
    spiWriteRegister(RFM22B_TXPOWER, BINDING_POWER);
    //delayMicroseconds(25); // PA ramp up/down time
    rfmSetCarrierFrequency(BINDING_FREQUENCY);
  } else {
    rfmSetModemRegs(&modem_params[bind_data.modem_datarate]);
    //rfmSetPower(bind_data.rf_power);
    spiWriteRegister(RFM22B_TXPOWER, bind_data.rf_power);
    //delayMicroseconds(25); // PA ramp up/down time
    rfmSetCarrierFrequency(bind_data.rf_frequency);
  }

return;
}


uint8_t rfmGetGPIO1(void)
{
  return spiReadRegister(RFM22B_GPIOCFG1);
}


void rfmSetTX(void)
{
  spiWriteRegister(RFM22B_OPMODE1, (RFM22B_OPMODE_TX | RFM22B_OPMODE_READY));
  //delayMicroseconds(200); // allow for PLL & PA ramp-up, ~200us
}


uint16_t rfmGetAFCC(void)
{
  return (((uint16_t) spiReadRegister(RFM22B_AFC0) << 2) | ((uint16_t) spiReadRegister(RFM22B_AFC1) >> 6));
}

void setHopChannel(uint8_t ch)
{
  uint8_t magicLSB = (bind_data.rf_magic & 0xFF) ^ ch;
  rfmSetHeader(3, magicLSB);
  rfmSetChannel(bind_data.hopchannel[ch]);
}


void updateHopChannel(void)
{
  if (willhop == 1) {
    RF_channel++;
    if ((RF_channel == MAXHOPS) || (bind_data.hopchannel[RF_channel] == 0)) {
      RF_channel = 0;
    }
    //handleBeacon();
    setHopChannel(RF_channel);
    willhop = 0;
  }
}


void check_module(void)
{
  if (rfmGetGPIO1() == 0) {
    // detect the locked module and reboot
    //Serial.println("RFM Module Locked");
    //Red_LED_ON;
    init_rfm(0);
    rx_reset();
    //Red_LED_OFF;
  }
}


void rfmClearIntStatus(void)
{
  spiReadRegister(RFM22B_INTSTAT1);
  spiReadRegister(RFM22B_INTSTAT2);
}

void tx_reset(void)
{
  tx_start = micros();
  RF_Mode = TRANSMIT;
  rfmSetTX();
}

uint32_t millis(void)
{

return(msec_of_sys_time_ticks(sys_time.nb_tick));
}


uint32_t micros(void)
{

return(usec_of_sys_time_ticks(sys_time.nb_tick));
}


inline void CRC16_reset(void)
{
  CRC16_value = 0;

return;
}


uint8_t countSetBits(uint16_t x)
{
  x  = x - ((x >> 1) & 0x5555);
  x  = (x & 0x3333) + ((x >> 2) & 0x3333);
  x  = x + (x >> 4);
  x &= 0x0F0F;
  return (x * 0x0101) >> 8;
}



void CRC16_add(uint8_t c) // CCITT polynome
{
  uint8_t i;
  CRC16_value ^= (uint16_t)c << 8;
  for (i = 0; i < 8; i++) {
    if (CRC16_value & 0x8000) {
      CRC16_value = (CRC16_value << 1) ^ 0x1021;
    } else {
      CRC16_value = (CRC16_value << 1);
    }
  }

return;
}


bool spiReadRegister(uint8_t rfm_reg)
{
  //uint8_t reg_val = 0;

  //rfm_trans.after_cb      = spi_cb;
  rfm_trans.output_length = 1;
  rfm_trans.input_length  = 2;
  rfm_trans.output_buf[0] = rfm_reg;
  //spi_blocking_transceive(&(RFM_SPI_DEV), &rfm_trans);
  if ( !spi_submit(&(RFM_SPI_DEV), &rfm_trans) ) {
    return false;
  }
  // Wait for transaction to complete
  uint32_t timeout = msec_of_sys_time_ticks(sys_time.nb_tick) + ((1000/SYS_TIME_FREQUENCY)+1);
  while ((rfm_trans.status == SPITransPending) || (rfm_trans.status == SPITransRunning)) {
    if (msec_of_sys_time_ticks(sys_time.nb_tick) > timeout) {
      break;
    }
  }

return(rfm_trans.input_buf[1]);
}


bool spiWriteRegister(uint8_t rfm_reg, uint8_t rfm_reg_val)
{
  rfm_trans.output_length = 2;
  rfm_trans.input_length  = 0;
  rfm_trans.output_buf[0] = rfm_reg;
  rfm_trans.output_buf[1] = (uint8_t)(rfm_reg_val);
  //spi_blocking_transceive(&(RFM_SPI_DEV), &rfm_trans);
  if ( !spi_submit(&(RFM_SPI_DEV), &rfm_trans) ) {
    return false;
  }
  // Wait for transaction to complete
  uint32_t timeout = msec_of_sys_time_ticks(sys_time.nb_tick) + ((1000/SYS_TIME_FREQUENCY)+1);
  while (rfm_trans.status == SPITransPending || rfm_trans.status == SPITransRunning) {
    if (msec_of_sys_time_ticks(sys_time.nb_tick) > timeout) {
      break;
    }
  }

return(true);
}


/*
// Blocking read/write functions

static uint8_t readRegister_blocking(struct spi_trans *rfm, uint8_t addr) {

  rfm->trans.output_buf[0] = addr & 0x7F;  // MSB 0 => read
  rfm->trans.output_length = 1;
  rfm->trans.input_length = 0;
  rfm->trans.select = SPISelect;
  spi_blocking_transceive(rfm->periph, &rfm->trans);
  sys_time_usleep(35);  // See ref firmware and datasheet
  rfm->trans.output_length = 0;
  rfm->trans.input_length = 1;
  rfm->trans.select = SPIUnselect;
  spi_blocking_transceive(rfm->periph, &rfm->trans);
  rfm->trans.select = SPISelectUnselect;

return(rfm->trans.input_buf[0])
}
*/


void rfmSetChannel(uint8_t ch)
{
  spiWriteRegister(RFM22B_FHCH, ch);
}


void rfmSetHeader(uint8_t iHdr, uint8_t bHdr)
{
  spiWriteRegister(RFM22B_TXHDR3+iHdr, bHdr);
  spiWriteRegister(RFM22B_CHKHDR3+iHdr, bHdr);

return;
}


void rfmSetCarrierFrequency(uint32_t f)
{
  uint16_t fb, fc, hbsel;
  if (f < 480000000) {
    hbsel = 0;
    fb = f / 10000000 - 24;
    fc = (f - (fb + 24) * 10000000) * 4 / 625;
  } else {
    hbsel = 1;
    fb = f / 20000000 - 24;
    fc = (f - (fb + 24) * 20000000) * 2 / 625;
  }
  spiWriteRegister(RFM22B_BANDSEL, 0x40 + (hbsel ? 0x20 : 0) + (fb & 0x1f));
  spiWriteRegister(RFM22B_CARRFREQ1, (fc >> 8));
  spiWriteRegister(RFM22B_CARRFREQ0, (fc & 0xff));
  //delayMicroseconds(200); // VCO / PLL calibration delay

return;
}


void rfmSetModemRegs(struct rfm_modem_regs* r)
{
  spiWriteRegister(RFM22B_IFBW,      r->r_1c);
  spiWriteRegister(RFM22B_AFCLPGR,   r->r_1d);
  spiWriteRegister(RFM22B_AFCTIMG,   r->r_1e);
  spiWriteRegister(RFM22B_RXOSR,     r->r_20);
  spiWriteRegister(RFM22B_NCOFF2,    r->r_21);
  spiWriteRegister(RFM22B_NCOFF1,    r->r_22);
  spiWriteRegister(RFM22B_NCOFF0,    r->r_23);
  spiWriteRegister(RFM22B_CRGAIN1,   r->r_24);
  spiWriteRegister(RFM22B_CRGAIN0,   r->r_25);
  spiWriteRegister(RFM22B_AFCLIM,    r->r_2a);
  spiWriteRegister(RFM22B_TXDR1,     r->r_6e);
  spiWriteRegister(RFM22B_TXDR0,     r->r_6f);
  spiWriteRegister(RFM22B_MODCTL1,   r->r_70);
  spiWriteRegister(RFM22B_MODCTL2,   r->r_71);
  spiWriteRegister(RFM22B_FREQDEV,   r->r_72);
}


void rfmClearInterrupts(void)
{
  spiWriteRegister(RFM22B_INTEN1, 0x00);
  spiWriteRegister(RFM22B_INTEN2, 0x00);

return;
}


uint8_t rfmGetPacketLength(void)
{
  return spiReadRegister(RFM22B_RXPLEN);
}


void rfmGetPacket(uint8_t *buf, uint8_t size)
{
  // Send the package read command
  //spiSendAddress(RFM22B_FIFO);
  for (uint8_t i = 0; i < size; i++) {
   buf[i] = 0; //spiReadData();
  }

return;
}


uint8_t tx_done()
{
  if (RF_Mode == TRANSMITTED) {
    RF_Mode = AVAILABLE;
    return 1; // success
  } else if ((RF_Mode == TRANSMIT) && ((micros() - tx_start) > 100000)) {
    RF_Mode = AVAILABLE;
    return 2; // timeout
  }

return 0;
}


uint8_t getPacketSize(struct Bind_Data *ps)
{
  return pktsizes[(ps->flags & 0x07)];
}


uint32_t getInterval(struct Bind_Data *ps)
{
  uint32_t ret;
  // Sending an 'x' byte packet at 'y' bps takes approx. (emperical):
  // usec = (x + 15 {20 w/ diversity}) * 8200000 / bps
#define BYTES_AT_BAUD_TO_USEC(bytes, bps, div) ((uint32_t)((bytes) + (div?20:15)) * 8200000L / (uint32_t)(bps))

  ret = (BYTES_AT_BAUD_TO_USEC(getPacketSize(ps), modem_params[ps->modem_datarate].bps, ps->flags & DIVERSITY_ENABLED) + 2000);

  if (ps->flags & TELEMETRY_MASK) {
    ret += (BYTES_AT_BAUD_TO_USEC(TELEMETRY_PACKETSIZE, modem_params[ps->modem_datarate].bps, ps->flags & DIVERSITY_ENABLED) + 1000);
  }

  // round up to ms
  ret = ((ret + 999) / 1000) * 1000;

  // enable following to limit packet rate to 50Hz at most
#ifdef LIMIT_RATE_TO_50HZ
  if (ret < 20000) {
    ret = 20000;
  }
#endif

return ret;
}


void unpackChannels(uint8_t config, volatile uint16_t PPM_VALUES[], uint8_t *p)
{
  uint8_t i;
  for (i=0; i<=(config/2); i++) { // 4ch packed in 5 bytes
    // the 5th byte holds the 2 MSBits of each servo channel, 
    // the first 2 LSBits are the 9th and 10th bit of the first servo value in microseconds.
    PPM_VALUES[0] = (((uint16_t)p[4] & 0x03) << 8) + p[0];
    PPM_VALUES[1] = (((uint16_t)p[4] & 0x0c) << 6) + p[1];
    PPM_VALUES[2] = (((uint16_t)p[4] & 0x30) << 4) + p[2];
    PPM_VALUES[3] = (((uint16_t)p[4] & 0xc0) << 2) + p[3];
    p+=5;
    PPM_VALUES+=4;
  }
  if (config & 1) { // 4ch packed in 1 byte;
    PPM_VALUES[0] = (((uint16_t)p[0] >> 6) & 3) * 333 + 12;
    PPM_VALUES[1] = (((uint16_t)p[0] >> 4) & 3) * 333 + 12;
    PPM_VALUES[2] = (((uint16_t)p[0] >> 2) & 3) * 333 + 12;
    PPM_VALUES[3] = (((uint16_t)p[0] >> 0) & 3) * 333 + 12;
  }
  for (i=0; i < RADIO_CONTROL_NB_CHANNEL; i++) {
    ppm_pulses[i] = PPM[i] * RC_PPM_TICKS_PER_USEC; // 6 ppm pulses are 1 microsecond.
  }
  // signal that we have a new rc frame, it auto reset once the frame is read.
  // if we loose the link then this variable won't be set againn triggering a RC_REALLY_LOST message
  // after about 1 second, relevant files are ppm.c and radio_control.c
  ppm_frame_available = true;
  //radio_control.status = RC_OK;
 

return;
}


void rx_reset(void)
{
  //rfmClearFIFO(bind_data.flags & DIVERSITY_ENABLED);
  //clear FIFO, disable multi-packet, enable diversity if needed
  //requires two write ops, set & clear
  spiWriteRegister(RFM22B_OPMODE2, 0x03);
  spiWriteRegister(RFM22B_OPMODE2, 0x00);

  rfmClearIntStatus();


  RF_Mode = RECEIVE;
  //rfmSetRX();
  spiWriteRegister(RFM22B_INTEN1, RFM22B_RX_PACKET_RECEIVED_INTERRUPT);
  spiWriteRegister(RFM22B_OPMODE1, (RFM22B_OPMODE_RX | RFM22B_OPMODE_READY));
  //delayMicroseconds(200);  // allow for PLL ramp-up, ~200us, NOT NEEDED IN THIS APPLICATION

return;
}


void tx_packet_async(uint8_t* pkt, uint8_t size)
{
  //rfmSendPacket(pkt, size);
  spiWriteRegister(RFM22B_PKTLEN, size);   // total tx size
  for (uint8_t i = 0; i < size; i++) {
    spiWriteRegister(RFM22B_FIFO, pkt[i]);
  }
  spiWriteRegister(RFM22B_INTEN1, RFM22B_PACKET_SENT_INTERRUPT);

  rfmClearIntStatus();
  
  tx_reset();

return;
}


void tx_packet(uint8_t* pkt, uint8_t size)
{
  tx_packet_async(pkt, size);
  while ((RF_Mode == TRANSMIT) && ((micros() - tx_start) < 100000));

return;
}


void checkRSSI(void)
{
  // sample RSSI when packet is in the 'air'
  if ((numberOfLostPackets < 2) && (lastRSSITimeUs != lastPacketTimeUs) && (pktTimeDelta > RSSI_timeout)) {
    lastRSSITimeUs = lastPacketTimeUs;
    lastRSSIvalue = rfmGetRSSI(); // Read the RSSI value
    RSSI_sum += lastRSSIvalue;    // tally up for average
    RSSI_count++;

    if (RSSI_count > 8) {
      RSSI_sum /= RSSI_count;
      smoothRSSI = (((uint16_t)smoothRSSI * 3 + (uint16_t)RSSI_sum * 1) / 4);
      //set_RSSI_output();
      RSSI_sum = 0;
      RSSI_count = 0;
    }
  }
}

void checkLinkState(void)
{
  if ((numberOfLostPackets < hopcount) && (pktTimeDelta > hopTimeout)) {
    // we lost a packet, so hop to next channel
    linkQuality <<= 1;
    willhop = 1;
    if (numberOfLostPackets == 0) {
      linkLossTimeMs = timeMs;
    }
    numberOfLostPackets++;
    lastPacketTimeUs += hopInterval;
    willhop = 1;
    //Red_LED_ON;
    //updateLBeep(true);
  } else if ((numberOfLostPackets == hopcount) && (pktTimeDelta > hopTimeoutSlow)) {
    // hop slowly to allow re-sync with TX
    linkQuality = 0;
    willhop = 1;
    smoothRSSI = 0;
    lastPacketTimeUs = timeUs;
  }

  if (numberOfLostPackets) {
    //handleFailsafe();
    uint32_t timeMsDelta = (timeMs - linkLossTimeMs);
    if ( timeMsDelta > (rx_config.ppmStopDelay * 100) ) {
      disablePPM = 1;
      //radio_control.status = RC_REALLY_LOST;
    }
  }

return;
}




uint8_t bindReceive(uint32_t timeout)
{
  uint32_t start = millis();
  uint8_t rxc_buf[33];
  uint8_t len;
  
  init_rfm(1);
  rx_reset();
  //Serial.println("Waiting bind\n");
  while ((!timeout) || ((millis() - start) < timeout)) {
    if (RF_Mode == RECEIVED) {
      //Serial.println("Got pkt");
      len=rfmGetPacketLength();
      rfmGetPacket(rxc_buf, len);
      switch((char) rxc_buf[0]) {

        case 'b': { // GET bind_data
          memcpy(&bind_data, (rxc_buf + 1), sizeof(bind_data));
          if (bind_data.version == BINDING_VERSION) {
            //Serial.println("data good");
            rxc_buf[0] = 'B';
            tx_packet(rxc_buf, 1); // ACK that we got bound
            //Green_LED_ON; //signal we got bound on LED:s
            return 1;
          }
        }
        break;

        default : break;
      }
      rx_reset();
    }
  }

return 0;
}


uint8_t rfmGetRSSI(void)
{
  return spiReadRegister(RFM22B_RSSI);
}


void handlePacketRX(void)
{

  uint32_t timeTemp = usec_of_sys_time_ticks(sys_time.nb_tick);
  // force the datalink to add to the buffer instead of putting the first byte to the UART register in order to start the interrupts
  DOWNLINK_DEVICE.tx_running = true;

  if (RF_Mode == RECEIVED) {

      rfmGetPacket(rx_buf, getPacketSize(&bind_data));
      lastAFCCvalue = rfmGetAFCC();
      //Green_LED_ON;
      lastPacketTimeUs = timeTemp;
      numberOfLostPackets = 0;
      linkQuality <<= 1;
      linkQuality |= 1;
      //updateLBeep(false);

      if ((rx_buf[0] & 0x3e) == 0x00) {  // this packet has the servo values.
        unpackChannels(bind_data.flags & 7, PPM, rx_buf + 1);

      } else if ((rx_buf[0] & 0x38) == 0x38) { // this packet has the serial uplink telemetry.
        //processPacketData();
        // process serial / data up-link packet 
        if ((rx_buf[0] ^ tx_buf[0]) & 0x80) {
          // We got new data... (not retransmission)
          uint8_t i;
          tx_buf[0] ^= 0x80; // signal that we got it
          //relay the uplink telemetry to the serial port's input buffer.
          for (i = 0; i <= (rx_buf[0] & 7);) {  // up to 8 bytes maximum.
            i++;
            uint16_t temp = (DOWNLINK_DEVICE.rx_insert_idx + 1) % UART_RX_BUFFER_SIZE;;
            DOWNLINK_DEVICE.rx_buf[DOWNLINK_DEVICE.rx_insert_idx] = rx_buf[i];
            // check for more room in queue.
            // FIXME i need to address what happens if there isn't any more room in the telemetry rx buffer
            if (temp != DOWNLINK_DEVICE.rx_extract_idx) {
              DOWNLINK_DEVICE.rx_insert_idx = temp;  // update insert index
            }
          }
        }
      }
      linkAcquired = 1;
      disablePPM = 0;
      // if we have enabled the transparentserial link (default in our case)
      //handlePacketTelem();
      if ((tx_buf[0] ^ rx_buf[0]) & 0x40) {
        // resend last message
        // does this currently do anything? -csurf
      } else {
        tx_buf[0] &= 0xc0;
        tx_buf[0] ^= 0x40; // swap sequence as we have new data
        // if we have room in the output buffer add the data TO THE TELEMETRY DOWNLINK .
        if (DOWNLINK_DEVICE.tx_insert_idx != DOWNLINK_DEVICE.tx_extract_idx) {
          uint8_t bytes = 0;
          // Add the serial data bytes to the output buffer, 8 bytes maximum.
          while ((bytes < 8) && (DOWNLINK_DEVICE.tx_insert_idx != DOWNLINK_DEVICE.tx_extract_idx)) {
            bytes++;
            tx_buf[bytes] =  DOWNLINK_DEVICE.tx_buf[DOWNLINK_DEVICE.tx_extract_idx];
            DOWNLINK_DEVICE.tx_extract_idx++;
            DOWNLINK_DEVICE.tx_extract_idx %= UART_TX_BUFFER_SIZE;
          }
          tx_buf[0] |= (0x37 + bytes);
        } else {
          // tx_buf[0] lowest 6 bits left at 0
          tx_buf[1] = lastRSSIvalue;
          tx_buf[2] = 0;
          tx_buf[3] = 0;
          tx_buf[4] = (lastAFCCvalue >> 8);
          tx_buf[5] = lastAFCCvalue & 0xff;
          tx_buf[6] = countSetBits(linkQuality & 0x7fff);
        }
     }

     tx_packet_async(tx_buf, 9);

     rfmClearIntStatus();
     tx_reset();
     rfmSetTX();
     // wait until the packet is sent with timeout.
     while(!tx_done()) {
       //checkSerial();
     }

     willhop = 1;
     rx_reset();
  }

return;
}

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//                        P U B L I C   F U N C T I O N   D E F I N I T I O N S
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

//111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111



//222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222
void rfm_init(void) {

  rfm_trans.slave_idx     = RFM_SLAVE_IDX;
  rfm_trans.select        = SPISelectUnselect;
  rfm_trans.cpol          = SPICpolIdleLow;
  rfm_trans.cpha          = SPICphaEdge1;
  rfm_trans.dss           = SPIDss8bit;
  rfm_trans.bitorder      = SPIMSBFirst;
  rfm_trans.cdiv          = SPIDiv64;
  rfm_trans.output_length = sizeof(rfm_spi_tx_buffer);
  rfm_trans.output_buf    = (uint8_t *) rfm_spi_tx_buffer;
  rfm_trans.input_length  = sizeof(rfm_spi_rx_buffer);;
  rfm_trans.input_buf     = (uint8_t *)rfm_spi_rx_buffer;
  rfm_trans.before_cb     = NULL;
  rfm_trans.after_cb      = NULL;

  //rxReadEeprom();
  //failsafeLoad();

  //bindInitDefaults();
  bind_data.version = BINDING_VERSION;
  bind_data.serial_baudrate = DEFAULT_BAUDRATE;
  bind_data.rf_power = DEFAULT_RF_POWER;
  bind_data.rf_frequency = DEFAULT_CARRIER_FREQUENCY;
  bind_data.rf_channel_spacing = DEFAULT_CHANNEL_SPACING;
  bind_data.rf_magic = DEFAULT_RF_MAGIC;
  memset(bind_data.hopchannel, 0, sizeof(default_hop_list));
  memcpy(bind_data.hopchannel, default_hop_list, sizeof(default_hop_list));

  bind_data.modem_datarate = DEFAULT_DATARATE;
  bind_data.flags = DEFAULT_FLAGS;
 
  if (bindReceive(500)) { //we always bind at start!
    //bindWriteEeprom();
    // save data to flash
    write_to_flash_8bit(11, (char*)&bind_data, sizeof(bind_data));  
/*
      // FIXME in case i need to change strategy on saving the bind data to flash i will use separate 32bit variables.
      uint8_t idx = 0;
      for(idx = 0; idx < MAXHOPS; idx++) {
        flash_data[idx] = bind_data.hopchannel[idx];
      }
      flash_data[idx++] = bind_data.version;
      flash_data[idx++] = bind_data.serial_baudrate;
      flash_data[idx++] = bind_data.rf_frequency;
      flash_data[idx++] = bind_data.rf_magic;
      flash_data[idx++] = bind_data.rf_power;
      flash_data[idx++] = bind_data.rf_channel_spacing;
      flash_data[idx++] = bind_data.modem_datarate;
      flash_data[idx++] = bind_data.flags;
      write_to_flash_32bit(11, flash_data, sizeof(flash_data)); 
*/
  }
  // NOT ALL SETTINGS ARE USED!
  //rxInitDefaults();
  rx_config.flags = ALWAYS_BIND;
  rx_config.RSSIpwm = 255; // off, IT DOES NOT MATTER IN THIS APPLICATION
  rx_config.failsafeDelay = 10; //1s, IT DOES NOT MATTER IN THIS APPLICATION
  rx_config.ppmStopDelay = 3; //300ms, IT DOES NOT MATTER IN THIS APPLICATION
  rx_config.pwmStopDelay = 3; //300ms
  rx_config.beacon_frequency = DEFAULT_BEACON_FREQUENCY; // IT DOES NOT MATTER IN THIS APPLICATION
  rx_config.beacon_deadtime = DEFAULT_BEACON_DEADTIME;   // IT DOES NOT MATTER IN THIS APPLICATION
  rx_config.beacon_interval = DEFAULT_BEACON_INTERVAL;   // IT DOES NOT MATTER IN THIS APPLICATION
  rx_config.minsync = 3000;


  rx_config.rx_type = RX_MICRO;
  rx_config.pinMapping[0] = PINMAP_PPM;
  rx_config.pinMapping[1] = PINMAP_ANALOG;
  rx_config.pinMapping[2] = PINMAP_RSSI;
  rx_config.pinMapping[3] = PINMAP_ANALOG;
  rx_config.pinMapping[4] = PINMAP_RXD;
  rx_config.pinMapping[5] = PINMAP_TXD;
/*
  rx_config.rx_type = RX_OLRSNG4CH;  
  rx_config.pinMapping[0] = PINMAP_PPM;     // IT DOES NOT MATTER IN THIS APPLICATION
  rx_config.pinMapping[1] = PINMAP_ANALOG;  // IT DOES NOT MATTER IN THIS APPLICATION
  rx_config.pinMapping[2] = PINMAP_RSSI;    // IT DOES NOT MATTER IN THIS APPLICATION
  rx_config.pinMapping[3] = PINMAP_ANALOG;  // IT DOES NOT MATTER IN THIS APPLICATION
  rx_config.pinMapping[4] = 4;              // IT DOES NOT MATTER IN THIS APPLICATION
  rx_config.pinMapping[5] = 5;              // IT DOES NOT MATTER IN THIS APPLICATION
  rx_config.pinMapping[6] = PINMAP_RXD;     // IT DOES NOT MATTER IN THIS APPLICATION
  rx_config.pinMapping[7] = PINMAP_TXD;     // IT DOES NOT MATTER IN THIS APPLICATION
*/
  // READ SETTINGS FROM STM32F405 FLASH AND LOAD THEM TO THE RAM. 
  flash_Read(flashSectorBegin(11), (char*)&bind_data, sizeof(bind_data) );
/*
  // FIXME in case i need to change strategy on saving the bind data to flash i will use separate 32bit variables.
  for(idx = 0; idx < MAXHOPS; idx++) {
    bind_data.hopchannel[idx] = flash_data[idx];
  }
  bind_data.version = flash_data[idx++];
  bind_data.serial_baudrate = flash_data[idx++];
  bind_data.rf_frequency = flash_data[idx++];
  bind_data.rf_magic = flash_data[idx++];
  bind_data.rf_power = flash_data[idx++];
  bind_data.rf_channel_spacing = flash_data[idx++];
  bind_data.modem_datarate = flash_data[idx++];
  bind_data.flags = flash_data[idx++];
  //write_to_flash(11, flash_data, sizeof(flash_data); 
*/       

#if defined(STM32F4)
  //setupRfmInterrupt();
  // SETUP THE EXTERNAL PIN INTERRUPT
  rcc_periph_clock_enable(RCC_SYSCFG);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOB);

  gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO5);
  //gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO6);

  nvic_enable_irq(NVIC_EXTI9_5_IRQ);
  exti_select_source(EXTI5, GPIOB);
  //exti_select_source(EXTI6, GPIOC);
  exti_set_trigger(EXTI5, EXTI_TRIGGER_FALLING);
  //exti_set_trigger(EXTI6, EXTI_TRIGGER_RISING);
  exti_enable_request(EXTI5);
  //exti_enable_request(EXTI6);
  nvic_set_priority(NVIC_EXTI9_5_IRQ, 0x0F);

#elif defined(STM32F7) || defined(STM32H7)
#pragma message "USING CHIBIOS HAL"

  // ChibiOS/HAL and ChibiOS/RT initialization. 
  //halInit();
  //chSysInit();
/*
  palSetPadMode(GPIOA, 0,PAL_MODE_INPUT_PULLUP | PAL_STM32_OSPEED_HIGHEST);
  // Enabling events on both edges of PA0 signal.
  //palEnablePadEvent(GPIOA, 0U, PAL_EVENT_MODE_BOTH_EDGES);
  palEnablePadEvent(GPIOA, 0U, PAL_EVENT_MODE_FALLING_EDGE);
  palSetPadCallback(GPIOA, 0U, chibios_ext_isr_cb, NULL);
*/
  palSetLineMode(LINE_RFM_EXTI, PAL_MODE_INPUT_PULLUP | PAL_STM32_OSPEED_HIGHEST);
  palEnableLineEvent(LINE_RFM_EXTI, PAL_EVENT_MODE_FALLING_EDGE);
  palSetLineCallback(LINE_RFM_EXTI, chibios_ext_isr_cb, NULL);

#endif

  // Count hopchannels as we need it later
  hopcount=0;
  while ((hopcount < MAXHOPS) && (bind_data.hopchannel[hopcount] != 0)) {
    hopcount++;
  }

  //################### RX SYNC AT STARTUP #################
  init_rfm(0);   // Configure the RFM22B's registers for normal operation
  RF_channel = 0;
  setHopChannel(RF_channel);
  rx_reset();
  // force the datalink to add to the buffer instead of putting the first byte to the UART register in order to start the interrupts
  DOWNLINK_DEVICE.tx_running = true;   
  serial_head = 0;
  serial_tail = 0;
  linkAcquired = 0;
  hopInterval = getInterval(&bind_data);
  hopTimeout = hopInterval + 1000;
  hopTimeoutSlow = hopInterval * hopcount;
  RSSI_timeout = hopInterval - 1500;
  lastPacketTimeUs = usec_of_sys_time_ticks(sys_time.nb_tick);

return;
}


//333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333
void rfm_periodic(void){

  // watchdogReset(); // checked ok, not used
  check_module();  // checked ok, code follows

  //checkSerial();  // NOT USED HERE, the serial buffer is updayed elsewhere.

  handlePacketRX();

  timeUs = micros();
  timeMs = millis();
  pktTimeDelta = (timeUs - lastPacketTimeUs);  

  checkRSSI();
  
  if (linkAcquired) {
    // check RC link status after initial 'lock'
    checkLinkState();
  } else if (pktTimeDelta > hopTimeoutSlow) {
    // Still waiting for first packet, so hop slowly
    lastPacketTimeUs = timeUs;
    willhop = 1;
  }

  //updateSerialRC(); // update serial rc protocols like sbus or spectrum, NOT USED HERE
  updateHopChannel();

  palWaitPadTimeout(GPIOA, 0U, TIME_INFINITE);
  // The action depends on the button state.
    if (palReadPad(GPIOA, 0U) == PAL_HIGH) {
    // Button is pressed: turning the LED on. 
      palSetPad(GPIOE, 8U);
    }
    else {
      // Button is released: turning the LED off. 
      palClearPad(GPIOE, 8U);
    }


return;
}


//444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444
void rfm_event( void ) 
{

  if (rfm_trans.status == SPITransSuccess) {
    rfm_trans.status = SPITransDone;

    switch (rfm_status) {

      case (RFM_STATUS_WAITING_BYTE):
        rfm_status = RFM_STATUS_FINISHED;
        break;

      default: break;
    }
  }
 
return;
}

//###################################################################################################
//                                  INTERRUPT SERVICE ROUTINES
//###################################################################################################

#if defined(STM32F4)
void exti9_5_isr(void)
{
  // clear EXTI

  if (EXTI_PR & EXTI6) {
    exti_reset_request(EXTI6);
    exti_6_triggered = true;
  }

  if (EXTI_PR & EXTI5) {  // check the interrupt flag
    exti_reset_request(EXTI5);
    if (RF_Mode == TRANSMIT) {
      RF_Mode = TRANSMITTED;
      isr_packet_tranmitted = true;
    } else if (RF_Mode == RECEIVE) {
      RF_Mode = RECEIVED;
      isr_packet_received = true;
      lastReceived = millis();
    }
  }

return;
}
#elif defined(STM32F7) || defined(STM32H7)

static void chibios_ext_isr_cb(void *arg)
{
    (void)arg;
    if (RF_Mode == TRANSMIT) {
      RF_Mode = TRANSMITTED;
      isr_packet_tranmitted = true;
    } else if (RF_Mode == RECEIVE) {
      RF_Mode = RECEIVED;
      isr_packet_received = true;
      lastReceived = millis();
    }

return;
}
#endif


/************************************************************************************************/
//                                       T H E   E N D
/************************************************************************************************/

