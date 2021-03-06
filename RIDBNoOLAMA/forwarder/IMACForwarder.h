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
	CTRL_SLOT_FTSP_CHANCE_MASK = 0, //0x1,
	
	#warning SLOT_MASK
	// change if link set changes, least SLOT_MASK = (2^n - 1), such that 2^n > active_link_size (not >= to accomodate dedicated ftsp slots)
	SLOT_MASK = 0x7F, //0x7F,
	// max # of slot search forward; to prevent inf loop
	MAX_SLOT_FORWARD = SLOT_MASK + 1,

#ifdef SCREAM
#warning ACTIVE_LINK_SIZE
	ACTIVE_LINK_SIZE = 100,	//100
//#warning INTERFERENCE_DIAMETER
	// w/ highest power level
	INTERFERENCE_DIAMETER = 3,
	// plus one data transaction and one concurrency decision making
	ROUND_LEN = 1 + INTERFERENCE_DIAMETER + 1,
	FRAME_LEN = ROUND_LEN * ACTIVE_LINK_SIZE,
	
	// control ftsp slot frequency; 1 occasionally insufficient
	FTSP_SLOT_RATIO_BASE = 3,
	
	// conservative; half of packet tx time to not miss any SCREAM; in us
	LISTEN_INTERVAL = 2048,
	// conservative; divide 2 to not exceed a slot
	LISTEN_CNT = SLOT_LEN / LISTEN_INTERVAL / 2,
#endif
//#warning fake control channel
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
