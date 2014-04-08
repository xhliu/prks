/* *
 * @ author: Xiaohui Liu (whulxh@gmail.com) 
 * @ updated: 7/24/2012 
 * @ description: LUR cache based on LruCtpMsgCacheC
 */

#include <message.h>

generic module AsyncCacheP(uint8_t size) {
    provides {
      interface Init;
      interface AsyncCache<message_t*> as Cache;
    }
    uses {
      //interface CtpPacket;
	#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
     	interface AsyncPacket as SubPacket;
    #else
    	interface Packet as SubPacket;
    #endif
    }
}
implementation {
typedef struct {
	am_addr_t origin;
	uint16_t seqno;
} network_packet_sig_t;

router_header_t* getHeader(message_t* m) {
	return (router_header_t*)call SubPacket.getPayload(m, sizeof(router_header_t));
}

network_packet_sig_t cache[size];
uint8_t first;
uint8_t count;

command error_t Init.init() {
	first = 0;
	count = 0;
	return SUCCESS;
} 

void printCache() {
#ifdef TOSSIM
	int i;
	dbg("Cache", "Cache:");
	for (i = 0; i < count; i++) {
		dbg_clear("Cache", " %04x %02x", cache[i].origin, cache[i].seqno);
		if (i == first)
			dbg_clear("Cache","*");
	} 
	dbg_clear("Cache","\n");
#endif
}

/* if key is in cache returns the index (offset by first), otherwise returns count */
uint8_t lookup(message_t* m) {
	uint8_t i;
	uint8_t idx;
atomic {	
	for (i = 0; i < count; i++) {
		idx = (i + first) % size;
		if (getHeader(m)->origin == cache[idx].origin &&
			getHeader(m)->originSeqNo == cache[idx].seqno) {
			break;
		}
	}
}
	return i;
}

/* remove the entry with index i (relative to first) */
void remove(uint8_t i) {
	uint8_t j;
atomic {
	if (i >= count) 
		return;
	if (i == 0) {
		//shift all by moving first
		first = (first + 1) % size;
	} else {
		//shift everyone down
		for (j = i; j < count; j++) {
			memcpy(&cache[(j + first) % size], &cache[(j + first + 1) % size], sizeof(network_packet_sig_t));
		}
	}
	count--;
}
}

#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
	async
#endif
command void Cache.insert(message_t* m) {
	uint8_t i;
atomic {
	if (count == size ) {
		//remove someone. If item not in 
		//cache, remove the first item.
		//otherwise remove the item temporarily for
		//reinsertion. This moves the item up in the
		//LRU stack.
		i = lookup(m);
		remove(i % count);
	}
	//now count < size
	cache[(first + count) % size].origin = getHeader(m)->origin;
	cache[(first + count) % size].seqno  = getHeader(m)->originSeqNo;
	count++;
}
}

#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
	async
#endif
command bool Cache.lookup(message_t* m) {
	atomic return (lookup(m) < count);
}

#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
	async
#endif
command void Cache.flush() {
	call Init.init(); 
}

}
