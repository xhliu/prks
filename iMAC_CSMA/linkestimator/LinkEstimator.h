/*
 * @ author: Xiaohui Liu (whulxh@gmail.com)
 * @ created: 12/28/2011
 * @ acknowledgement: based on 4bitle
 */
#ifndef LINK_ESITIMATOR_H
#define LINK_ESITIMATOR_H

// Number of entries in the neighbor table
#ifndef NEIGHBOR_TABLE_SIZE
#define NEIGHBOR_TABLE_SIZE 6
#endif

// Masks for the flag field in the link estimation header
enum {
	// use last four bits to keep track of
	// how many footer entries there are
	NUM_ENTRIES_FLAG = 15,
};

// The first byte of each outgoing packet is a control byte
// Bits 4..7 reserved for routing and other protocols
// Bits 0..3 is used by the link estimator to encode the
//   number of linkest entries in the packet

// link estimator header added to
// every message passing through the link estimator
typedef nx_struct linkest_header {
	nx_uint8_t flags;
	nx_uint8_t seq;
} linkest_header_t;

// for outgoing link estimator message
// we put the above neighbor entry in the footer
typedef nx_struct linkest_footer {
	nx_am_addr_t ll_addr;
	nx_uint8_t inquality;
	nx_uint8_t inquality_version;
} linkest_footer_t;


// Flags for the neighbor table entry
enum {
	VALID_ENTRY = 0x1, 
	// A link becomes mature after BLQ_PKT_WINDOW
	// packets are received and an estimate is computed
	MATURE_ENTRY = 0x2,
	// Flag to indicate that this link has received the
	// first sequence number
	INIT_ENTRY = 0x4,
	// eetx becomes mature after initialized
	EETX_MATURE_ENTRY = 0x8,
	// The upper layer has requested that this link be pinned
	// Useful if we don't want to lose the root from the table
	PINNED_ENTRY = 0x10,
};


// neighbor table entry
typedef struct neighbor_table_entry {
	// link layer address of the neighbor
	am_addr_t ll_addr;
	// last beacon sequence number received from this neighbor
	uint8_t lastseq;
	// number of beacons received after last beacon estimator update
	// the update happens every BLQ_PKT_WINDOW beacon packets
	uint8_t rcvcnt;
	// number of beacon packets missed after last beacon estimator update
	uint8_t failcnt;
	// flags to describe the state of this entry
	uint8_t flags;
	// inbound qualities in the range [1..255]
	// 1 bad, 255 good
	uint8_t inquality;
	// inbound PDR version
	uint8_t inquality_version;
	// Y_(S, R)(t) for inbound DATA
	uint8_t inquality_sample;
	// outbound qualities
	uint8_t outquality;
	// last outbound PDR version
	uint8_t last_outquality_version;
	// Y_(S, R)(t) for inbound ACK
	uint8_t ack_quality_sample;
	// EETX for the link to this neighbor. This is the quality returned to
	// the users of the link estimator
	uint16_t eetx;
	// Number of data packets successfully sent (ack'd) to this neighbor
	// since the last data estimator update round. This update happens
	// every DLQ_PKT_WINDOW data packets
	uint8_t data_success;
	// The total number of data packets transmission attempt to this neighbor
	// since the last data estimator update round.
	uint8_t data_total;
} neighbor_table_entry_t;


#endif
