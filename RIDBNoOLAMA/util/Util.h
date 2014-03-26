#ifndef UTIL_H
#define UTIL_H

typedef struct {
	am_addr_t sender;
	am_addr_t receiver;
#ifdef HETER_TX_POWER
	uint8_t tx_power_level;
#endif
} link_t;
#endif
