/*
 * @ author: Xiaohui Liu (whulxh@gmail.com)
 * 
 * @ updated: 04/05/2012 08:56:00 PM 
 */

#ifndef TEST_IMAC_H
#define TEST_IMAC_H

//#warning basic CSMA
//#define DEFAULT_MAC
//#warning RTSCTS
//#define RTSCTS
//#warning CMAC
//#define CMAC

// TDMA
//#warning SCREAM enabled
//#define SCREAM
// basically PRKS but differ w/ const initial ER, no adaptation; OLAMA_DISABLED has to be enabled for original RIDB w/o ONAMA
//#warning RIDB enabled
//#define RIDB

// by default, PRKS is used w/ initialized ER as in RIDB

// multihop? ; prior to IMAC.h, which uses it
#define MULTIHOP
#warning MULTIHOP

#include "IMAC.h"

#if defined(CMAC)
#include "IMACBeacon.h"
#endif

// links w/ pdr >= 99%
//#define LINK_SET_PDR_99
//#warning LINK_SET_PDR_99

// randomize packet interval, uniform [t/2, 3t/2]
//#define RANDOM_PKT_INTERVAL
//#warning RANDOM_PKT_INTERVAL
// changing traffic/period
//#define VARY_PERIOD
//#warning VARY_PERIOD


#if defined(DEFAULT_MAC) || defined(RTSCTS) || defined(CMAC)
	#warning change Makefile & make telosb !!!
#else
// PRKS specific
	// run basic LAMA only: PRKS or RIDB
//	#define OLAMA_DISABLED
//	#warning OLAMA_DISABLED
	
	// PRKS-L: do not use refined a(t) and use SINR-vs-PDR gradient directly even it's away from target
//	#define PRKSL
//	#warning PRKSL
	// PID controller w/ filter
//	#define FILTERED_PID
//	#warning FILTERED_PID
	
	// change of pdr req online
//	#define VARY_PDR_REQ
//	#warning VARY_PDR_REQ
	// heterogeneous pdr req
//	#define HETER_PDR_REQ
//	#warning HETER_PDR_REQ
	// heterogeneous per node tx power
//	#define HETER_TX_POWER
//	#warning HETER_TX_POWER
	// set pdr requirement by link
//	#define SET_PDR_REQ_BY_LINK
//	#warning SET_PDR_REQ_BY_LINK

	
	// disable SPI resource arbitration
	#define DUMMY_SPI

	//#warning disable fix packet-level time sync: sender-side
	#define TIMESYNC_FIX

	#define TEST_FTSP

#endif
// use FastCC2420TransceiverC to replace ActiveMessage; no iMAC neither
//#define RAW_FAST_MAC


// choose controller
//#define MIN_VAR_CONTROLLER

// enable tx ER
//#define TX_ER

// ensure slot integrity, see IMACForwarderP for more info
//#define SLOT_INTEGRITY

// enable signal map
#define SIGNAL_MAP

// only works assuming the smallest id node does not change, which holds for static testbeds
#define FTSP_FIX
//#define FAST_SMCLK

enum {
// fit in one pkt after format change
#if defined(DEFAULT_MAC)
//#warning
	// not using 110 to be safe
	PLACE_HOLDER_LEN = 106
#elif defined(RTSCTS)
	// maxPayloadLength 99, to accomodate linkestimator footer
	PLACE_HOLDER_LEN = 91
#elif defined(CMAC)
	// 4 bytes less than RTS-CTS
	PLACE_HOLDER_LEN = 86
#elif defined(SCREAM)
	//22 + sizeof(link_er_footer_t) * MAX_ITEM_CNT
	PLACE_HOLDER_LEN = 94
#else
	// 86
	PLACE_HOLDER_LEN = 22
#endif
#ifdef MULTIHOP
	// - 4 to account for network_header_t
	- 4
#endif
,
	TX_FAIL_FLAG = 0,
	TX_SUCCESS_FLAG = 1,
	TX_DONE_FLAG = 2,
	RX_FLAG = 3,
	TX_DONE_FAIL_FLAG = 4,
	
	DBG_FLAG = 255,
	DBG_LOSS_FLAG = 0,
	DBG_TX_FLAG = DBG_LOSS_FLAG + 1,
	DBG_RX_FLAG = DBG_TX_FLAG + 1,
	DBG_BACKOFF_FLAG = DBG_RX_FLAG + 1,
	DBG_ER_FLAG = DBG_BACKOFF_FLAG + 1,
	DBG_SM_FLAG = DBG_ER_FLAG + 1,				//5
	DBG_TX_FAIL_FLAG = DBG_SM_FLAG + 1,
	DBG_TIMEOUT_FLAG = DBG_TX_FAIL_FLAG + 1,
	DBG_BI_ER_FLAG = DBG_TIMEOUT_FLAG + 1,
	DBG_EXEC_TIME_FLAG = DBG_BI_ER_FLAG + 1,
	DBG_DELAY_FLAG = DBG_EXEC_TIME_FLAG + 1,	//10
	DBG_CANCEL_FLAG = DBG_DELAY_FLAG + 1,
	DBG_CONTROLLER_FLAG = DBG_CANCEL_FLAG + 1,
	DBG_COUNTER_NAV_FLAG = DBG_CONTROLLER_FLAG + 1,
	DBG_CALC_FLAG = DBG_COUNTER_NAV_FLAG + 1,
	DBG_HEARTBEAT_FLAG = DBG_CALC_FLAG + 1,		// 15
	DBG_FTSP_FLAG = DBG_HEARTBEAT_FLAG + 1,
	DBG_TDMA_FLAG = DBG_FTSP_FLAG + 1,
	DBG_SPI_FLAG = DBG_TDMA_FLAG + 1,
	DBG_DRIVER_FLAG = DBG_SPI_FLAG + 1,
	DBG_ERR_FLAG = DBG_DRIVER_FLAG + 1,			//20
	DBG_OVERFLOW_FLAG = DBG_ERR_FLAG + 1,

	/**
		initial stage:
		[0 .. INITIAL_ER_TIME] jump start signal map
		[INITIAL_ER_TIME .. INITIAL_FTSP_TIME] exchange link ERs
		[0 .. INITIAL_FTSP_TIME] jump start ftsp
	 */
	//SM_BEACON_CNT = 1000UL, // 2000 seems insufficient
	// when to initialize ER w/ tx range afte SM is valid; SM_BEACON_PERIOD is 50 ms
	//INITIAL_ER_TIME = (uint32_t)SM_BEACON_PERIOD_MILLI * SM_BEACON_CNT,
	//FTSP_BEACON_CNT = (SM_BEACON_CNT << 1),
	// when to start data forwarding after ftsp converges
	//INITIAL_FTSP_TIME = (uint32_t)SM_BEACON_PERIOD_MILLI * FTSP_BEACON_CNT,
#if defined(DEFAULT_MAC) || defined(RTSCTS)
	START_DATA_TIME = 30000,
#elif defined(CMAC)
	#warning START_DATA_TIME
	// refer to IMACBeacon.h,  + 100000U is to ensure beacon has completed
	START_DATA_TIME = (uint32_t)BEACON_PERIOD_SHORT * BEACON_PERIOD_SHORT_CNT + 100000U,
#else
	START_DATA_TIME = INITIAL_ER_TIME,
#endif
#ifdef MULTIHOP
#warning change period
#endif
#warning period not 20
	// CSMA/RTSCTS/CMAC: 1800UL, 	RIDB&SCREAM 22500UL, 	PRKS: 360000UL,
	PERIOD_MILLI = 32,	// 20
	MAX_PKT_CNT = 45000U,
	
	TYPE_SYNC = 12,
//#warning	
	ROOT_NODE_ID = 15,
};

typedef nx_struct radio_count_msg {
	nx_uint16_t src;
	nx_uint16_t seqno;
	// to make pkt len equal for fair comparison
	nx_uint8_t place_holder[PLACE_HOLDER_LEN];
} radio_count_msg_t;


// sync
//enum {
//	// sync	
//	AM_TYPE_SYNC = 12,
//	BUFFER_TIME = 2000,
//	CONVERGE_TIME = 300000U,
//};

typedef nx_struct {
	nx_uint16_t seqno;
	nx_uint32_t globalTime;
} sync_header_t;

#endif
