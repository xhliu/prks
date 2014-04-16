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
	
	// 95 percentile of turnaround time
	CTS_TIMEOUT = 75,
	DATA_TIMEOUT = 75,
	
	//ideally, from a interferer receives RTS till the ack is received by the sender; approxiate from RTS sendDone
	// gagged node, set as max CTS turnaround time instead of max transaction time from RTS sent to ACK back (300 ms)
	RTS_NAV = 100,	//80,
	
	//ideally, from a interferer receives CTS till the ack is received by the sender; approxiate from CTS reception
	CTS_NAV = 65,	//35 (70 percentile), 65 (90 percentile),

// 40, 40, 70, 35	
	// max tries for RTS to transmit
	RTS_MAX_RETRIES = 4,	//8
	// max retries for CTS to backoff before dropping it
	CTS_MAX_BACKOFF_RETRIES = 2,
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

#endif
