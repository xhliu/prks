/*
 * @ author: Xiaohui Liu (whulxh@gmail.com)
 * 
 * @ updated: 04/05/2012 08:56:00 PM 
 */
 
#ifndef IMAC_FORWARDER_H
#define IMAC_FORWARDER_H

enum {
	TYPE_RTS 	= 1,
	TYPE_CTS 	= 2,
	TYPE_DATA 	= 3,
	TYPE_INVALID = 4,
	
// 40, 40, 70, 35	
	// max tries for RTS to transmit
	RTS_MAX_RETRIES = 4,	//8
	// max retries for CTS to backoff before dropping it
	CTS_MAX_BACKOFF_RETRIES = 2,
	
#warning C-MAC related setting here
	//#warning low SINR_THRESHOLD_MARGIN
	// 3.5 * 128 = 448
	SINR_THRESHOLD_MARGIN = 0, //448,
	//scaled: <30%, 1.6 = 205>, <40%, 1.8 = 231>, <50%, 2.2 = 282>, <60%, 2.4 = 308>, <70%, 2.8 * 128 = 359> <80%, 3.2 * 128 = 410> <90%, 4 * 128 = 512>, <95%, 4.5 * 128 = 576>, <99%, 16 * 128 = 2048>
	SINR_THRESHOLD = 359 + SINR_THRESHOLD_MARGIN,
	LINK_TABLE_SIZE = 120,
	
	// based on CMAC paper
	//SNOOP_DURATION = 80,
	//#warning BLOCK_SIZE 1
	BLOCK_SIZE = 10,
	// conservative
	PACKET_TIME = 8,

	// not used since NAV is absent in CMAC
	//ideally, from a interferer receives RTS till the ack is received by the sender; approxiate from RTS sendDone
	// gagged node, set as max CTS turnaround time instead of max transaction time from RTS sent to ACK back (300 ms)
	RTS_NAV = 100 + (BLOCK_SIZE - 1) * PACKET_TIME,	//80,
	//ideally, from a interferer receives CTS till the ack is received by the sender; approxiate from CTS reception
	CTS_NAV = 65 + (BLOCK_SIZE - 1) * PACKET_TIME,	//35 (70 percentile), 65 (90 percentile),
	
	// 95 percentile of turnaround time
	CTS_TIMEOUT = 75,
	DATA_TIMEOUT = 75 + (BLOCK_SIZE - 1) * PACKET_TIME,


	//#warning corresponds only to power level 3
	CC2420_DEF_RFPOWER_DBM = -25,
	CC2420_DEF_RFPOWER_DBM_SCALED = (CC2420_DEF_RFPOWER_DBM << SCALE_L_SHIFT_BIT),
};

// state machine of iMAC
enum {
	S_IDLE,
	S_SENDING_RTS,
	S_SENDING_CTS,
	S_SENDING_DATA,
	S_EXPECTING_CTS,
	S_EXPECTING_DATA,
	S_CANCELLING_RTS,
	S_CANCELLING_CTS,
	//S_CANCELLING_DATA
};

// iMAC header: RTS/CTS only contains header, while data packet contains additional upper layer payload
typedef nx_struct {
	// iMAC pkts: RTS/CTS/data
	nx_uint8_t type;
	
	// receiver; only for RTS/CTS, not for data
	nx_am_addr_t dst;
	
	// P(S, R) / K : decides exclusion region
	nx_int16_t interference_threshold;
	// determines maximal ER; others use this information to decide outbound ER and thus tx power
	nx_int16_t min_interference_threshold;
	// network allocation vector for virtual carries sensing
	nx_uint16_t nav;
	
	// TODO
	nx_uint16_t seqno;
	nx_int16_t node_i;
	
	// C-MAC
	// only in data and CTS, to compute SINR at receiver
	nx_int16_t rx_signal;
	nx_int16_t ni;
	nx_uint8_t block_idx;
} imac_header_t;

// early detection of transaction completion
enum {
	LINK_NAV_CACHE_SIZE = 10,
};
typedef struct {
	bool valid;
	// link
	am_addr_t sender;
	am_addr_t receiver;
	
	uint32_t link_nav;
} link_nav_entry_t;


// C-MAC related

// link table entry <link, sinr related>
typedef struct {
	bool valid;
	am_addr_t sender;
	am_addr_t receiver;
	// signal at the sender from the receiver
	int16_t rx_signal;
	// noise + interference at receiver
	int16_t ni;
	// time this info overheard
	//uint32_t rx_timestamp;
	// time this overheard block completes tx
	uint32_t complete_timestamp;
} link_table_entry_t;


#endif
