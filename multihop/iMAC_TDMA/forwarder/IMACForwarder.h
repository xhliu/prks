/*
 * @ author: Xiaohui Liu (whulxh@gmail.com)
 * 
 * @ updated: 06/27/2012 
 */
 
#ifndef IMAC_FORWARDER_H
#define IMAC_FORWARDER_H

enum {
	//#warning all duration in uis
	SLOT_LEN = 32 * 1024UL, //(16384UL << 1),
//	SLOT_LEN_SHIFT = 15,	// 2 ^ 14 = 16384
//	SLOT_HEX_MODULAR = 0x00007FFF, // x % SLOT_LEN == x & SLOT_HEX_MODULAR
	
	// contention window size
	MIN_CW = 2048, //2000,
	// CW = 2048
	CW_HEX_MODULAR = 0x7FF,	// x % CW == x & CW_HEX_MODULAR
	
	// tx ftsp beacon instead of ctrl pkt 1 out of CTRL_SLOT_FTSP_CHANCE when ctrl channel available
	INIT_CTRL_SLOT_FTSP_CHANCE_MASK = 0x1,
	// tunable
	CTRL_SLOT_FTSP_CHANCE_MASK = 0x1,
	
//	#warning change if link set changes
//	// least SLOT_MASK = (2^n - 1) such that 2^n > active_link_size
//	SLOT_MASK = 0x3F, //0x7F,
//	// max # of slot search forward; to prevent inf loop
//	MAX_SLOT_FORWARD = SLOT_MASK + 1,

	// control channel; see Guoliang RTSS09 Section 3B
	CC2420_CONTROL_CHANNEL = 19,
};

typedef nx_struct {
	// TODO
	nx_uint8_t seqno;
	// # of ER entries piggybacked in DATA
	nx_uint8_t link_er_cnt;
	nx_uint32_t next_slot_by_tx;
} imac_header_t;


#endif
