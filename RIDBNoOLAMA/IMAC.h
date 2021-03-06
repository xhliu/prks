#ifndef IMAC_H
#define IMAC_H

#ifdef SCREAM
#include "IMACForwarder.h"
#endif

enum {
	AM_IMAC_LE = 6,
	AM_IMAC_SM = 7,
	
	// EWMA, w/ a denominator of 10
	ALPHA = 9,
	// 128
	SCALE_L_SHIFT_BIT = 7,
	
#warning pdr req; SINR_THRESHOLD MUST be consistent w/ pdr req
	// PDR requirement
	REFERENCE_DATA_PDR = 90,
	// SINR threshold to initialize ER based on pairwise interference model
	//scaled: <30%, 1.6 = 205>, <40%, 1.8 = 231>, <50%, 2.2 = 282>, <60%, 2.4 = 308>, <70%, 2.8 * 128 = 359> <80%, 3.2 * 128 = 410> <90%, 4 * 128 = 512>, <95%, 4.5 * 128 = 576>
	SINR_THRESHOLD = 512,

	// RTSCTS needs this
	REFERENCE_ACK_PDR = 90,

//	#warning corresponds to power level 3 -> 11
//	CC2420_DEF_RFPOWER_DBM = -10, //-25,
//	CC2420_DEF_RFPOWER_DBM_SCALED = (CC2420_DEF_RFPOWER_DBM << SCALE_L_SHIFT_BIT),
	CONTROL_POWER_LEVEL = 31,
	
	// SM_SIZE = 128 is insufficient for Kansei; many outbound gains unknown
	// size is int16_t to accommodate network consisting of more than 256 nodes
	SM_SIZE = 128,
	//SM_SIZE_MODULAR = 0x7F, //0x7F,	// x % SM_SIZE == x & SM_SIZE_MODULAR

	// can be less than max # of active links in the ntw; keep LINK_ER_TABLE_SIZE of them
	MAX_ACTIVE_LINK_SIZE = 100,
	// max # of links a node can be incident w/
	MAX_INCIDENT_LINK_SIZE = 4,
	NB_SIGNAL_MAP_SIZE = MAX_INCIDENT_LINK_SIZE,
	LOCAL_LINK_ER_TABLE_SIZE = MAX_INCIDENT_LINK_SIZE,
	NEIGHBOR_TABLE_SIZE = MAX_INCIDENT_LINK_SIZE,
	LINK_ER_TABLE_SIZE = MAX_ACTIVE_LINK_SIZE,
	//NB_ER_CI_TABLE_SIZE = MAX_INCIDENT_LINK_SIZE,
	
	// invalid global time
	INVALID_TIME = 0xFFFFFFFF,
	// use max, otherwise next_slot_by_rx cannot be updated since it's the earliest
	INVALID_SLOT = INVALID_TIME,
	INVALID_ADDR = 0xFFFF,
	
	// period to send signal map beacons at the beginning
	// cannot exceed 64 ms bcoz IMACForwarderP$SlotTime is uint16_t
#ifndef SCREAM
	SM_BEACON_PERIOD_MILLI = 50UL,
#warning sm_beacon_cnt
	SM_BEACON_CNT = 5000UL,	// 5000 seems insufficient for 130 nodes, takes about 10 mins

#else
	SM_BEACON_PERIOD_MILLI = 32UL,
#warning SM_BEACON_CNT
	// must be a multiple of FRAME_LEN so that 1st link's frame is complete
	SM_BEACON_CNT = FRAME_LEN * 4UL,	// 10
#endif
	// when to initialize ER w/ tx range afte SM is valid
	INITIAL_ER_TIME = (uint32_t)SM_BEACON_PERIOD_MILLI * SM_BEACON_CNT,
	FTSP_BEACON_CNT = (SM_BEACON_CNT << 1),
	// when to start data forwarding after ftsp converges
	GLOBAL_TDMA_START_TIME = ((uint32_t)SM_BEACON_PERIOD_MILLI << 10) * FTSP_BEACON_CNT,
};

#define assert(param) call UartLog.logEntry(DBG_FLAG, DBG_ERR_FLAG, __LINE__, param)

#endif
