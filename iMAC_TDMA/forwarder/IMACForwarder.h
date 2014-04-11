/*
 * @ author: Xiaohui Liu (whulxh@gmail.com)
 * 
 * @ updated: 06/27/2012 
 */
 
#ifndef IMAC_FORWARDER_H
#define IMAC_FORWARDER_H

enum {
	// all duration in uis unless otherwise noted
	
	// SLOT_LEN must be a factor of 2^32 for implicit time wraparound
#if !defined(SCREAM) && !defined(OLAMA_DISABLED)
//#warning 2 s slot
	SLOT_LEN = ((uint32_t)512 << 10), // 512
#else
	SLOT_LEN = ((uint32_t)32 << 10),
#endif
//	SLOT_LEN_SHIFT = 15,	// 2 ^ 14 = 16384
//	SLOT_HEX_MODULAR = 0x00007FFF, // x % SLOT_LEN == x & SLOT_HEX_MODULAR
	
	// contention window size
	MIN_CW = 2048,
	// CW = 2048
	CW_HEX_MODULAR = 0x7FF,	// x % CW == x & CW_HEX_MODULAR

	// tx ftsp beacon instead of ctrl pkt 1 out of CTRL_SLOT_FTSP_CHANCE when ctrl channel available
	//INIT_CTRL_SLOT_FTSP_CHANCE_MASK = 0x1,
	// tunable
	//CTRL_SLOT_FTSP_CHANCE_MASK = 0, //0x1,
	// sample tx probability every TX_PROB_SAMPLE_WINDOW slots
	TX_PROB_SAMPLE_WINDOW = 100,
	
// subslot related:	DATA || CTRL/FTSP || COMPUTATION
//#warning DATA_SUBSLOT_LEN_MILLI
	DATA_SUBSLOT_LEN_MILLI = 24, //12,
	DATA_SUBSLOT_LEN = ((uint32_t)DATA_SUBSLOT_LEN_MILLI << 10),
	// be conservative not to exceed slot boundary: > 160ms
	//COMPUTATION_SUBSLOT_LEN_MILLI = 176,
	//COMPUTATION_SUBSLOT_LEN = ((uint32_t)COMPUTATION_SUBSLOT_LEN_MILLI << 10),
	// CAUTION: cannot exceed or equal 64 ms; 16 bit micro alarm
//#warning COMM_SUBSLOT_BEACON_CNT
	// TODO: can be increased since at most 1 tx in a slot
//#warning 1 s slot
	COMM_SUBSLOT_FTSP_BEACON_CNT = 4, // 16, // 8, // 4,
	COMM_SUBSLOT_FTSP_BEACON_PERIOD_MILLI = 3,		// on basis of measurement
	COMM_SUBSLOT_FTSP_BEACON_PERIOD = ((uint32_t)COMM_SUBSLOT_FTSP_BEACON_PERIOD_MILLI << 10),
//#warning 1 s slot
	COMM_SUBSLOT_CTRL_BEACON_CNT = 10, // 65, // 26, // 10,
	COMM_SUBSLOT_CTRL_BEACON_PERIOD_MILLI = 24,		// on basis of measurement
	COMM_SUBSLOT_CTRL_BEACON_PERIOD = ((uint32_t)COMM_SUBSLOT_CTRL_BEACON_PERIOD_MILLI << 10),
//#warning 1 s slot
	// CONTENTION_INTERVAL * COMM_SUBSLOT_CTRL_BEACON_CNT >= total # of nodes
	CONTENTION_INTERVAL = 13, // 2, // 5, //13,
	// total # of beacons
	COMM_SUBSLOT_BEACON_CNT = COMM_SUBSLOT_FTSP_BEACON_CNT + COMM_SUBSLOT_CTRL_BEACON_CNT,
//#warning 1 s slot
	// > DATA_SUBSLOT_LEN_MILLI + COMM_SUBSLOT_FTSP_BEACON_CNT * COMM_SUBSLOT_FTSP_BEACON_PERIOD + COMM_SUBSLOT_CTRL_BEACON_CNT * COMM_SUBSLOT_CTRL_BEACON_PERIOD
	COMM_SUBSLOT_LEN_MILLI = 324, // 824 + (39 + 2) * (24 + 1) + 32 + 10,  // 324 + 16 + 400 + 84, // 324
	// # of ctrl/ftsp beacons to tx in a dedicated comm subslot; 1 is low utilization and may cause time out of sync after some time, e.g., 40 min
	COMM_SUBSLOT_LEN = ((uint32_t)COMM_SUBSLOT_LEN_MILLI << 10),
	// max beacon interval; smaller than, not equal to, (COMM_SUBSLOT_LEN_MILLI / COMM_SUBSLOT_BEACON_CNT) bcoz of firing delay
	//COMM_SUBSLOT_BEACON_PERIOD_MILLI = 8,
	//COMM_SUBSLOT_BEACON_PERIOD = ((uint32_t)COMM_SUBSLOT_BEACON_PERIOD_MILLI << 10),
	// contention window
	//COMM_SUBSLOT_BEACON_CW = (COMM_SUBSLOT_BEACON_PERIOD >> 1),
	//COMM_SUBSLOT_BEACON_CW_MASK = COMM_SUBSLOT_BEACON_CW - 1,
	// do not contend again till after CONTENTION_INTERVAL slots upon winning to mimic round robin
	// COMM_SUBSLOT_BEACON_CNT * CONTENTION_INTERVAL = ACTIVE_LINK_SIZE
	// CAUTION: CONTENTION_INTERVAL + (MAX_FORWARD_SLOT - ACTIVE_LINK_SIZE) < SUCCESSFUL_SIGNALLING_INTERVAL must hold
	//CONTENTION_INTERVAL = 13, //7 //8,
	
//	#warning SLOT_MASK
//	// change if link set changes, least SLOT_MASK = (2^n - 1), such that 2^n > active_link_size (not >= to accomodate dedicated ftsp slots)
//	SLOT_MASK = 0x7F, //0x7F,
//	// max # of slot search forward; to prevent inf loop
//	MAX_SLOT_FORWARD = SLOT_MASK + 1,

#ifdef VARY_PDR_REQ
	// how often to change pdr req, in terms of wraparounds
	WRAPAROUND_CNT = 2,
#endif
		
#ifdef SCREAM
#warning ACTIVE_LINK_SIZE should be equal to activeLinks size
	ACTIVE_LINK_SIZE = 45, //100,
//#warning INTERFERENCE_DIAMETER
	// w/ highest power level
	// based on false postive/negative analysis
	INTERFERENCE_DIAMETER = 3, //8, //3, 
	// plus 1st slot data transaction and last slot concurrency decision making and ftsp
	ROUND_LEN = 1 + INTERFERENCE_DIAMETER + 1,
	FRAME_LEN = ROUND_LEN * ACTIVE_LINK_SIZE,
	
//#warning FTSP_SLOT_RATIO_BASE
	// control ftsp slot frequency; 3 insufficient
	FTSP_SLOT_RATIO_BASE = 28, //28,
	
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
