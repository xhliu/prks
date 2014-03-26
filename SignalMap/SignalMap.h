/*
 * @ author: Xiaohui Liu (whulxh@gmail.com)
 * 
 * @ created: 12/28/2011
 */
 
#ifndef SIGNAL_MAP_H
#define SIGNAL_MAP_H

enum {
	INVALID_GAIN = 0xFFFF,
	EWMA_R_SHIFT_BIT = 4,	// right shift to scale down
	// scale gain for precision
	SCALE_L_SHIFT_BIT = 7,	//left shift to scale up
};

// default signal map size
#ifndef SM_SIZE
#define SM_SIZE 20
#endif

//signal map header containing tx power level
typedef nx_struct {
	nx_uint8_t power_level;
	// number of entries in the footer
	nx_uint8_t footer_entry_cnts;
	// TODO
	nx_uint16_t seqno;
} sm_header_t;

//signal map footer containing neighbor and inbound gain, used to compute outbound gain
typedef nx_struct {
	nx_am_addr_t nb;
	//power gain to a neighbor
	nx_uint16_t inbound_gain;
} sm_footer_t;

//signal map entry
typedef struct {
	am_addr_t nb;
	bool valid;
	uint16_t inbound_gain;
	uint16_t outbound_gain;
} sm_entry_t;

#endif
