/*
 * @ author: Xiaohui Liu (whulxh@gmail.com)
 * @ created: 12/28/2011
 */
 
#ifndef SIGNAL_MAP_H
#define SIGNAL_MAP_H

#include "IMAC.h"

enum {
	INVALID_GAIN = 0x7FFF,
	// 1 dB
	MIN_GAIN_GAP = (1 << SCALE_L_SHIFT_BIT),
	OFFSET = 1,
	EWMA_R_SHIFT_BIT = 4,	// right shift to scale down

//#ifdef RIDB
	MAX_NOISE = -90,
//#endif
};

//signal map header
typedef nx_struct {
	nx_uint8_t power_level;
	// number of entries in the footer
	nx_uint8_t footer_entry_cnts;
	// TODO: remove after debug
	//nx_uint16_t seqno;
	// tx prob.
	nx_uint8_t data_tx_slot_ratio;
} sm_header_t;

//signal map footer containing neighbor and inbound gain, used to compute outbound gain
typedef nx_struct {
	nx_am_addr_t nb;
	// my signal map
	nx_int16_t inbound_gain;
	nx_int16_t outbound_gain;	
} sm_footer_t;

//signal map entry
typedef struct {
	am_addr_t nb;
	bool valid;
	uint8_t data_tx_slot_ratio;
	// scaled
	// to compute ER
	int16_t inbound_gain;
	// to determine if I'm in the ER of the neighbor
	int16_t outbound_gain;
} sm_entry_t;


typedef struct {
	am_addr_t nb;
	int16_t inbound_gain;
	int16_t outbound_gain;
} signal_map_entry_t;

// further optimize since only inbound_gain of other nodes to my_receiver is necessary when only rx ER is considered? NO, receiver also has to know sender's outbound gain to the receiver of another link
// neighbor's signal map, from/to whom there is a link
typedef struct {
	am_addr_t nb;
	bool valid;
	signal_map_entry_t signal_map[SM_SIZE];
} nb_signal_map_entry_t;


#endif
