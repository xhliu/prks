#ifndef TEST_SM_H
#define TEST_SM_H

#define SIGNAL_MAP

// sync or async reading
//#define SYNC_READ

// fix sync error; needed for global sync to measure w/o concurrency
#define RX_TIMESTAMP

enum {
	TX_FLAG = 0,
	RX_FLAG = 1,
	DBG_FLAG = 2,
	PERIOD = 50,	//100,	//50,
	PKT_CNT = 200,	//65536,	//200,
	BUFFER_TIME = 90000,	//Indirya no sync: 90000		NetEye w/ commander pkt_time sync: 2000
	
	MSG_SIZE = 50,		// make packet long; the longer, the more challenging
	AM_RADIO_COUNT_MSG = 6,
};

typedef nx_struct radio_count_msg {
  nx_uint16_t counter[MSG_SIZE];
} radio_count_msg_t;


// sync
enum {
	AM_TYPE_SYNC = 12,	// must agree w/ commander
	CONVERGE_TIME = 300000,
	
	AM_UART_SYNC = 100,
};
typedef nx_struct {
	nx_uint16_t seqno;
	nx_uint32_t globalTime;
} sync_header_t;
#endif
