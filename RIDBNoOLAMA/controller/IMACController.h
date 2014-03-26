/*
 * @ author: Xiaohui Liu (whulxh@gmail.com)
 * 
 * @ updated: 04/05/2012 08:56:00 PM 
 */
#ifndef IMAC_CONTROLLER_H
#define IMAC_CONTROLLER_H

#include "IMAC.h"

enum {
	// scaled by 100
	DELTA_Y = 0,
	// 0.04 by Chuan: scaled by 100
	E0 = 4,
	
	// scale for accuracy; otherwise significant precision loss can occur
	SLOPE_SCALE = 1000,
	PDR_SLOPE_SCALE = 10,
	// cancel out with PDR scale
	//PDR_INV_SCALE = 100,
	
	// empty ER
	EMPTY_ER_IDX = -1,
	
	// max # of ER items to place in a packet
	// save as much space as possible to accommodate signal map entries initially
	INIT_MAX_ITEM_CNT = 0,
	// to make at least 1 sm_footer_t is admitted
	MAX_ITEM_CNT = 9,
	
	// initial prio of other links' ER items: sort of "THL"
	LOCAL_INIT_PRIO = 0,
	NON_LOCAL_INIT_PRIO = 2,

    //sqrt(qtl / (1 - qtl))
    CHEBYSHEV_SCALAR = 3,   //qtl 0.9
    
    BITSHIFT_3 = 3,
    
    //BITMAP_SIZE = (LINK_ER_TABLE_SIZE % 8) ? (LINK_ER_TABLE_SIZE / 8 + 1) : (LINK_ER_TABLE_SIZE / 8),

	// min signed 16 bits
	MIN_DBM = 0x8000,
	INVALID_DBM = 0x7FFF,

//#ifdef RIDB
//	//scaled: <30%, 1.6 = 205>, <40%, 1.8 = 231>, <50%, 2.2 = 282>, <60%, 2.4 = 308>, <70%, 2.8 * 128 = 359> <80%, 3.2 * 128 = 410> <90%, 4 * 128 = 512>, <95%, 4.5 * 128 = 576>
//	SINR_THRESHOLD = 308,
//#endif
};


// represent \delta_I in dBm
// 0 is represented as {0, 0}
typedef struct {
	// can only be 1, 0 or -1
	int8_t sign;
	// scaled absolute value in dBm; thus can be negative
	int16_t abs;
} dbm_t;


/*
 * IMACController packet layout 
 * header | payload | pdr footer | er footer
 */
typedef nx_struct {
	//nx_bool is_data_pending;
	nx_uint8_t local_link_pdr_cnt;
	nx_uint8_t link_er_cnt;
	// TODO
	nx_uint16_t seqno;
} imac_control_header_t;

// footer includes first PDR and then ER
// local link's PDR + next_slot
typedef nx_struct {
	nx_am_addr_t nb;
	// pdr piggybacked in ctrl pkt in case DATA receiver does not act as sender
	nx_uint8_t inquality;
	nx_uint8_t inquality_version;
	// to tx for sender, to rx for receiver
	nx_uint32_t next_slot;
} local_link_pdr_footer_t;


// ER item
typedef nx_struct {
	nx_am_addr_t sender;
	nx_am_addr_t receiver;
	nx_int16_t rx_interference_threshold;
	nx_uint8_t rx_er_version;
} link_er_footer_t;

enum {
	VALID_FLAG = 0x1,
	// am I in this link's ER
	IS_IN_ER_FLAG = 0x2,
	// link's ER changes. Two things are to be updated: is_in_er & contend_flags
	ER_CHANGED_FLAG = 0x4,
	// link's relay rank increment
	RANK_INCREMENT_FLAG = 0x8,
};

// link table entry <link, ER>
typedef struct {
	am_addr_t sender;
	am_addr_t receiver;
	
	uint8_t flags;
	// entry relay priority, not link priority
	// TODO: change to "rank" to avoid confusion
	uint8_t prio;
	// index within active links; used to compute priority
	uint8_t link_idx;
	int16_t rx_interference_threshold;
	uint8_t rx_er_version;
	// TODO: # of incidet links cannot exceed 8
	// i-th bit indicates whether this link contends w/ i-th link in localLinkERTable
	uint8_t contend_flags;
} link_er_table_entry_t;

// local link ER table entry
// ER here (i.e., local link's ER) is also stored in linkERTable to be uniformly sorted and relayed
// store as a seperate table from signal map if memory is ever a concern since only neighbors talking to me need to be maintained, whose # is much less than SM_SIZE
// further reduce since a link is seldomly bidirectional, store seperately if this ever occurs, differentiated by is_sender
typedef struct {
	am_addr_t nb;
	bool valid;
	// index within active links; used to compute priority
	uint8_t link_idx;
	
	bool is_sender;
	// next slot to tx for a sender, or rx for a receiver
	// sender-computed
	uint32_t next_slot_by_tx;
	// computed by receiver; for opportunistic receiver-informs-sender
	// always the earliest one
	uint32_t next_slot_by_rx;
	bool is_rx_pending;
	
	int16_t rx_interference_threshold;	
	uint8_t rx_er_version;
	/* boundary of the current exclusion region for the link to this neighbor (inbound), 
	 * i.e., outmost node index within the ER in SM
	 * ER is the set of neighbors from [0 .. ex_region_border_idx] in signalMap, not linkERTable
	 * EMPTY_ER_IDX (-1) means empty ER
	 */
	// when I receive DATA from the neighbor
	int16_t rx_er_border_idx;
	
	// min variance controller related info
	// notations: capital I means in capsulation, _dB mean in dB vs de facto in dBm
	// EWMA of noise+interference for a link during interval (t, t+1] so far
	dbm_t rx_nI;
	
#ifdef FILTERED_PID	
// PID controller info
	// sum of error for integration control; use 16 bits to alleviate overflow
	int16_t sum_e;
	// last error for derivative control	
	int8_t last_e;
#endif	
} local_link_er_table_entry_t;

#endif
