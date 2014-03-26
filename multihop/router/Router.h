/* *
 * @ author: Xiaohui Liu (whulxh@gmail.com) 
 * @ updated: 7/23/2012 
 */

#ifndef ROUTER_H
#define ROUTER_H

enum {
	#warning MAX_RETRIES 1
	MAX_RETRIES = 30, //8,
	
	CLIENT_SIZE = 1,
	POOL_SIZE = 14,
	QUEUE_SIZE = POOL_SIZE + CLIENT_SIZE,
	CACHE_SIZE = 4,
};

typedef nx_struct {
	nx_am_addr_t origin;
	nx_uint8_t originSeqNo;
} router_header_t;

typedef struct {
	message_t *msg;
	uint8_t retries;
} fe_queue_entry_t;

#endif
