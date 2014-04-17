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
	
//#warning pdr req; SINR_THRESHOLD MUST be consistent w/ pdr req
	// PDR requirement
	REFERENCE_DATA_PDR = 90,
	// SINR threshold to initialize ER based on pairwise interference model
	//scaled: <30%, 1.6 = 205>, <40%, 1.8 = 231>, <50%, 2.2 = 282>, <60%, 2.4 = 308>, <70%, 2.8 * 128 = 359> <80%, 3.2 * 128 = 410> <90%, 4 * 128 = 512>, <95%, 4.5 * 128 = 576>, <99%, 16 * 128 = 2048>
#ifndef CMAC
	// in $CMAC/forwarder/IMACForwarder.h
	SINR_THRESHOLD = 512,
#endif
	// RTSCTS needs this
	REFERENCE_ACK_PDR = 90,

//	#warning corresponds to power level 3 -> 11
//	CC2420_DEF_RFPOWER_DBM = -10, //-25,
//	CC2420_DEF_RFPOWER_DBM_SCALED = (CC2420_DEF_RFPOWER_DBM << SCALE_L_SHIFT_BIT),
	CONTROL_POWER_LEVEL = 31,
	
#ifndef MULTIHOP
	// SM_SIZE = 128 is insufficient for Kansei; many outbound gains unknown
	// size is int16_t to accommodate network consisting of more than 256 nodes
	SM_SIZE = 128,
	//SM_SIZE_MODULAR = 0x7F, //0x7F,	// x % SM_SIZE == x & SM_SIZE_MODULAR

	#ifndef LINK_SET_PDR_99
	// can be less than max # of active links in the ntw; keep LINK_ER_TABLE_SIZE of them
	MAX_ACTIVE_LINK_SIZE = 100,
//#warning MAX_INCIDENT_LINK_SIZE
	// max # of links a node can be incident w/
	MAX_INCIDENT_LINK_SIZE = 2, //2, //4,
	#else
	MAX_ACTIVE_LINK_SIZE = 63,
	MAX_INCIDENT_LINK_SIZE = 3,
	#endif
#else
	SM_SIZE = 46,
	MAX_ACTIVE_LINK_SIZE = 45,
	MAX_INCIDENT_LINK_SIZE = 4,
#endif
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
	
#warning SLOT_MASK
#if !defined(LINK_SET_PDR_99) && !defined(MULTIHOP)
	// change if link set changes, least SLOT_MASK = (2^n - 1), such that 2^n > active_link_size (not >= to accomodate dedicated ftsp slots)
	// not anymore in OLAMA: 2^n >= active_link_size equal is enough since no dedicated ftsp slot
	SLOT_MASK = 0x7F, //0x7F,
#else
	SLOT_MASK = 0x3F,
#endif
	// max # of slot search forward to ensure link consistency; to prevent inf loop
	MAX_SLOT_FORWARD = SLOT_MASK + 1,
	
//  O(ptimal)-LAMA related
	// overlay conflict graph, not the actual network
	CONFLICT_GRAPH_DIAMETER = 7, // 2
	// # of slots for one round in O-LAMA; 1 round includes 1 ntw-wide message exchange, takes about # of nodes bcoz of ctrl signalling CSMA in neteye
	// interval between two successful signalling
	SUCCESSFUL_SIGNALLING_INTERVAL = 16, // 56, 112UL, //128UL,
	// # of slots LAMA takes to converge, at least w/ high probability
	OLAMA_CONVERGENCE_TIME = (uint16_t)CONFLICT_GRAPH_DIAMETER * SUCCESSFUL_SIGNALLING_INTERVAL,
	// OLAMA_CONVERGENCE_TIME is chosen such that both GROUP_SIZE & ROUND_SIZE are 2^n
	GROUP_SIZE = 16,
	GROUP_SIZE_SHIFT = 4,
	GROUP_SIZE_MASK = GROUP_SIZE - 1,
	// # of groups in a round to group conflict graphs: 8 as represented by uint8_t conflict_flags
	ROUND_SIZE = OLAMA_CONVERGENCE_TIME / GROUP_SIZE + 1,
	ROUND_SIZE_MASK = ROUND_SIZE - 1,
	// # of slots to start OLAMA in advance, i.e., OLAMA starts at (t - OLAMA_PRECOMPUTE_TIME) for activation decision in slot t
	// OLAMA_CONVERGENCE_TIME is for OLAMA convergence, MAX_SLOT_FORWARD is to ensure link consistency
	//OLAMA_PRECOMPUTE_TIME = OLAMA_CONVERGENCE_TIME + MAX_SLOT_FORWARD,
	
	// period to send signal map beacons at the beginning
	// cannot exceed 64 ms bcoz IMACForwarderP$SlotTime is uint16_t
#ifndef SCREAM
	SM_BEACON_PERIOD_MILLI = 50UL,
#warning sm_beacon_cnt 2000
	SM_BEACON_CNT = 5000UL, //5000UL,	// 5000 seems insufficient for 130 nodes, takes about 10 mins

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
