#ifndef UTIL_H
#define UTIL_H

typedef struct {
	am_addr_t sender;
	am_addr_t receiver;
#ifdef HETER_TX_POWER
	uint8_t tx_power_level;
#endif
} link_t;

#ifdef SET_PDR_REQ_BY_LINK
enum {
	// for fair comparison
#warning PRKS_PDR_OVERSHOOT 10	
	PRKS_PDR_OVERSHOOT = 0, // 5,
};
#endif
#endif

