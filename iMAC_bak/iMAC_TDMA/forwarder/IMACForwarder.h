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
	//#warning SLOT_LEN 512
	SLOT_LEN = ((uint32_t)512 << 10), //256, //32 + 160
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
	
// subslot related	
	// be conservative not to exceed slot boundary: > 160ms
	//COMPUTATION_SUBSLOT_LEN_MILLI = 176,
	//COMPUTATION_SUBSLOT_LEN = ((uint32_t)COMPUTATION_SUBSLOT_LEN_MILLI << 10),
	// CAUTION: cannot exceed or equal 64 ms; 16 bit micro alarm
	COMM_SUBSLOT_LEN_MILLI = 300,
	// # of ctrl/ftsp beacons to tx in a dedicated comm subslot; 1 is low utilization and may cause time out of sync after some time, e.g., 40 min
	COMM_SUBSLOT_LEN = ((uint32_t)COMM_SUBSLOT_LEN_MILLI << 10),
//#warning COMM_SUBSLOT_BEACON_CNT
	// TODO: can be increased since at most 1 tx in a slot
	COMM_SUBSLOT_FTSP_BEACON_CNT = 8,
	COMM_SUBSLOT_FTSP_BEACON_PERIOD_MILLI = 3,		// on basis of measurement
	COMM_SUBSLOT_FTSP_BEACON_PERIOD = ((uint32_t)COMM_SUBSLOT_FTSP_BEACON_PERIOD_MILLI << 10),
	COMM_SUBSLOT_CTRL_BEACON_CNT = 16,
	COMM_SUBSLOT_CTRL_BEACON_PERIOD_MILLI = 15,		// on basis of measurement
	COMM_SUBSLOT_CTRL_BEACON_PERIOD = ((uint32_t)COMM_SUBSLOT_CTRL_BEACON_PERIOD_MILLI << 10),
	// total # of beacons
	COMM_SUBSLOT_BEACON_CNT = COMM_SUBSLOT_FTSP_BEACON_CNT + COMM_SUBSLOT_CTRL_BEACON_CNT,
	// max beacon interval; smaller than, not equal to, (COMM_SUBSLOT_LEN_MILLI / COMM_SUBSLOT_BEACON_CNT) bcoz of firing delay
	//COMM_SUBSLOT_BEACON_PERIOD_MILLI = 8,
	//COMM_SUBSLOT_BEACON_PERIOD = ((uint32_t)COMM_SUBSLOT_BEACON_PERIOD_MILLI << 10),
	// contention window
	//COMM_SUBSLOT_BEACON_CW = (COMM_SUBSLOT_BEACON_PERIOD >> 1),
	//COMM_SUBSLOT_BEACON_CW_MASK = COMM_SUBSLOT_BEACON_CW - 1,
	// do not contend again till after CONTENTION_INTERVAL slots upon winning to mimic round robin
	// COMM_SUBSLOT_BEACON_CNT * CONTENTION_INTERVAL = ACTIVE_LINK_SIZE
	// CAUTION: CONTENTION_INTERVAL + (MAX_FORWARD_SLOT - ACTIVE_LINK_SIZE) < SUCCESSFUL_SIGNALLING_INTERVAL must hold
#warning CONTENTION_INTERVAL
	CONTENTION_INTERVAL = 8, //2, //8,
	
//	#warning SLOT_MASK
//	// change if link set changes, least SLOT_MASK = (2^n - 1), such that 2^n > active_link_size (not >= to accomodate dedicated ftsp slots)
//	SLOT_MASK = 0x7F, //0x7F,
//	// max # of slot search forward; to prevent inf loop
//	MAX_SLOT_FORWARD = SLOT_MASK + 1,

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
