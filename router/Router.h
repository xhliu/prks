/* *
 * @ author: Xiaohui Liu (whulxh@gmail.com) 
 * @ updated: 7/23/2012 
 */

#ifndef ROUTER_H
#define ROUTER_H

enum {
//#warning MAX_RETRIES 1
	MAX_RETRIES = 8,
	
//	CLIENT_SIZE = 1,
//#warning POOL_SIZE 1
//	POOL_SIZE = 3, // 14,
//	QUEUE_SIZE = POOL_SIZE + CLIENT_SIZE,
//#warning QUEUE_SIZE 2
	QUEUE_SIZE = 255,
	CACHE_SIZE = 4,
};

typedef nx_struct {
	nx_am_addr_t origin;
	nx_uint16_t originSeqNo;
} router_header_t;

typedef struct {
	// replace msg to save space
	//message_t *msg;
	am_addr_t origin;
	uint16_t originSeqNo;
	
	uint8_t retries;
} fe_queue_entry_t;

#endif
