#ifndef OPENLRSNG_H
#define OPENLRSNG_H 

#define RSSI_AVAILABLE  1


#include <inttypes.h>

extern void rfm_init(void);
extern void rfm_periodic(void);
extern void rfm_event(void);

extern uint8_t rx_RSSI;

#endif

