/*
 * @ author: Xiaohui Liu (whulxh@gmail.com)
 * 
 * @ updated: 06/25/2012 
 */

#ifndef TEST_SLOT_LEN_H
#define TEST_SLOT_LEN_H

// fix sync error; needed for global sync to measure w/o concurrency
//#define RX_TIMESTAMP

//#include "IeeeEui64.h"

typedef nx_struct radio_count_msg {
	nx_uint16_t src;
	nx_uint16_t seqno;
	nx_uint8_t place_holders[100];
} radio_count_msg_t;

enum {
	AM_TYPE = 6,
	AM_TYPE_SYNC = 12,
	SINK_NODE_ID = 15,
	PERIOD = 1000,
	DBG_FLAG = 255,
	DBG_FTSP_FLAG = 15,
	DBG_SKEW_FLAG = 16,
	
	CC2420_CONTROL_CHANNEL = CC2420_DEF_CHANNEL,
};

typedef nx_struct {
	nx_uint16_t seqno;
	nx_uint32_t globalTime;
} sync_header_t;

#endif
