/*
 * @ author: Xiaohui Liu (whulxh@gmail.com)
 * 
 * @ updated: 07/02/2012 
 */
#include "Util.h"

interface Util {
	// return the active links: first element address and size
	async command link_t *getActiveLinks(uint8_t *size);
	
	// is the link <sender, receiver> active
	async command bool isActiveLink(am_addr_t sender, am_addr_t receiver);
	async command uint8_t findLinkIdx(am_addr_t sender, am_addr_t receiver);

	async command am_addr_t getReceiver();
	
#ifdef HETER_TX_POWER
	async command uint8_t getNodeTxPowerLevel(am_addr_t node);
#endif	
}
