/*
 * @ author: Xiaohui Liu (whulxh@gmail.com)
 * @ updated: 04/05/2012 08:56:00 PM 
 * @ description:
	1) controller	input: current link pdr
 					output: deltaI
 	2) prioritized forwarding of ER items bcoz a signal ctrl packet cannot accommodate all items
 		- relay highest MAX_ITEM_CNT priority items
 		- 0 is lowest priority
 		- local item initial priority LOCAL_INIT_PRIO, non-local item NON_LOCAL_INIT_PRIO
 */
#include "IMACController.h"
#include "IMAC.h"

module IMACControllerP {
	provides {
		interface AsyncAMSend as AMSend;
		interface AsyncReceive as Receive;
		interface AsyncPacket as Packet;
		
		interface AsyncStdControl as StdControl;
		interface IMACController as Controller;
		interface Init;
	};
	
	uses {
		interface AsyncAMSend as SubSend;
		interface AsyncReceive as SubReceive;
		interface AsyncPacket as SubPacket;
		interface AsyncAMPacket as SubAMPacket;
		interface CC2420Packet;
		
		interface ForwarderInfo;
		interface LinkEstimator;
		interface SignalMap;
		
		interface GlobalTime<TMicro>;
		
		interface Util;
		interface UartLog;
		interface LocalTime<TMicro>;
		interface Random;
	};
}
implementation {
// pdr_slope_table[] & pdr_inv_table both contain 101 entries [0..100], i-th entry corresponds to i%
// mapping PDR to (1 / a); scaled by 10
// TODO: scale here
uint16_t pdr_slope_table[] = {8405, 193, 114, 85, 70, 61, 54, 49, 45, 42, 40, 38, 36, 35, 34, 33, 31, 31, 30, 29, 29, 28, 28, 28, 27, 27, 27, 26, 26, 26, 26, 26, 26, 26, 26, 25, 25, 25, 25, 26, 26, 26, 26, 26, 26, 26, 26, 26, 27, 27, 27, 27, 28, 28, 28, 29, 29, 29, 30, 30, 31, 31, 32, 33, 33, 34, 35, 35, 36, 37, 38, 39, 40, 41, 43, 44, 46, 47, 49, 51, 53, 56, 58, 61, 65, 69, 73, 78, 84, 92, 100, 111, 125, 141, 164, 196, 243, 323, 6197, 12332, 34296UL};

// map PDR to inv(PDR), i.e., f^{-1}(PDR); scaled by 100
// change pdr_inv_table[0] from -24 to 0 to be uniform
uint16_t pdr_inv_table[] = {0, 36, 51, 61, 69, 75, 81, 86, 91, 95, 99, 103, 107, 111, 114, 117, 121, 124, 127, 130, 133, 136, 138, 141, 144, 147, 149, 152, 155, 157, 160, 162, 165, 167, 170, 173, 175, 178, 180, 183, 185, 188, 190, 193, 196, 198, 201, 203, 206, 209, 211, 214, 217, 220, 223, 225, 228, 231, 234, 237, 240, 243, 246, 250, 253, 256, 260, 263, 267, 270, 274, 278, 282, 286, 290, 294, 299, 303, 308, 313, 318, 324, 329, 335, 342, 348, 355, 363, 371, 380, 389, 400, 412, 425, 440, 458, 479, 507, 753, 1609, 2871};

uint8_t pdr_req;

int16_t CC2420_DEF_RFPOWER_DBM, CC2420_DEF_RFPOWER_DBM_SCALED;

am_addr_t my_ll_addr;
// index of my outgoing link in localLinkERTable
uint8_t my_local_link_idx;

// max # of ER to carry in a ctrl pkt
uint8_t const_max_item_cnt;
/*
 * localLinkERTable vs linkERTable: seperate these two bcoz no need estimate non-local links, memory-consuming
 * localLinkERTable: only link(s)'s sender ER starting from me and receiver ER to me
 					 also many fields used for ER adapation based on controller
 * linkERTable: all links' ER, including link(s)'s sender ER starting from me and receiver ER to me; duplicate info here bcoz 1) to make link ER storage uniform to facilitate implementation, otherwise distinguish local & non-local links; 2) the storage is there even not duplicate
 */
// local link er table
local_link_er_table_entry_t localLinkERTable[LOCAL_LINK_ER_TABLE_SIZE];

// contain all active links bi ER, including link(s)'s sender ER starting from me and receiver ER to me
// which is stored locally anyway; links are directional, meaning <a, b> and <b, a> are distinct in this table; expect hear link <a, b>'s tx ER from a and rx ER from b
link_er_table_entry_t linkERTable[LINK_ER_TABLE_SIZE];
// valid entry count
uint8_t link_er_table_size;

uint8_t active_link_size;
link_t *activeLinks;

sm_entry_t *signalMap;
neighbor_table_entry_t *neighborTable;

norace uint32_t start_time;

// divide into 2 seperate files for readability
#include "IMACControllerPUtil.nc"

// get the header in the packet
imac_control_header_t *getHeader(message_t* m) {
	return (imac_control_header_t *)call SubPacket.getPayload(m, sizeof(imac_control_header_t));
}
// get the pdr footer in the packet
// @param len: payload length of this layer
void* getFooter(message_t* m, uint8_t len) {
	return (void*)(len + (uint8_t *)call Packet.getPayload(m, len + sizeof(local_link_pdr_footer_t)));
}

command error_t Init.init() {
	activeLinks = call Util.getActiveLinks(&active_link_size);
	signalMap = call SignalMap.getSignalMap();
	neighborTable = call LinkEstimator.getNeighborTable();
	link_er_table_size = 0;
	const_max_item_cnt = INIT_MAX_ITEM_CNT;
	my_ll_addr = call SubAMPacket.address();
	my_local_link_idx = LOCAL_LINK_ER_TABLE_SIZE;

	CC2420_DEF_RFPOWER_DBM = call SignalMap.level2Power(CC2420X_DEF_RFPOWER);
	CC2420_DEF_RFPOWER_DBM_SCALED = (CC2420_DEF_RFPOWER_DBM << SCALE_L_SHIFT_BIT),
	
	pdr_req = REFERENCE_DATA_PDR;
	initLinkERTable();
	initLocalLinkERTable();
	return SUCCESS;
}

//-------------------------------------------------------------------------------------
// Interface StdControl
//-------------------------------------------------------------------------------------
// initialize ER
async command error_t StdControl.start() {
#ifdef HETER_PDR_REQ
#warning HETER_PDR_REQ enabled
	uint8_t pdr_req_idx;
	uint8_t pdr_reqs[] = {70, 80, 90, 95};
#endif

	// to ensure called after signal map boots up to use communication range to initialize ER
	int16_t num, in_num, out_num;
	uint8_t total, num0, num1, num2;
	call SignalMap.getSignalMapSizeDbg(&num, &in_num, &out_num);
	call SignalMap.getNbSignalMapSize(&total, &num0, &num1, &num2);
	call UartLog.logTxRx(DBG_FLAG, DBG_SM_FLAG, num, in_num, out_num, total, num0, num1, num2);
	
#ifdef HETER_PDR_REQ
	// all incoming links sharing same receiver use the same randomized pdr req
	pdr_req_idx = call Random.rand16() % (sizeof(pdr_reqs) / sizeof(pdr_reqs[0]));
	atomic pdr_req = pdr_reqs[pdr_req_idx];
#endif
	
	setLocalLinkERTable();
	atomic const_max_item_cnt = MAX_ITEM_CNT;
	return SUCCESS;
}

async command error_t StdControl.stop() {
	return SUCCESS;
}

//-------------------------------------------------------------------------------------
// Interface AMSend
//-------------------------------------------------------------------------------------
// slap the header and footer before sending the message
// add the header. Call just before sending the packet
uint8_t addLinkEstHeaderAndFooter(message_t *msg, uint8_t len);

async command error_t AMSend.send(am_addr_t addr, message_t* msg, uint8_t len) {
	uint8_t newlen;
	//start_time = call LocalTime.get();
	newlen = addLinkEstHeaderAndFooter(msg, len);
	//call UartLog.logEntry(DBG_FLAG, DBG_DELAY_FLAG, __LINE__, call LocalTime.get() - start_time);
	dbg("LI", "%s packet of length %hhu became %hhu\n", __FUNCTION__, len, newlen);
	if (newlen <= call Packet.maxPayloadLength()) {
		return (call SubSend.send(addr, msg, newlen));
	} else {
		// careful not to exceed max payload length
		// do not use SubSend.maxPayloadLength() bcoz that assumes empty footer at sub layer
		assert(newlen);
		return FAIL;
	}
}

uint16_t seqno = 0;
// load link ER
async command uint8_t Controller.loadLinkER(link_er_footer_t *er_footer) {
  	uint8_t i, k;
	link_er_table_entry_t *le;
	link_er_footer_t *er_footer_p;
  	
  	k = 0;
  	// top MAX_ITEM_CNT priority links whose ER I'm in
  	for (i = 0; i < LINK_ER_TABLE_SIZE && k < const_max_item_cnt; i++) {
  		// indirect sort
  		//idx = sortedIndices[i];
  		//le = &linkERTable[idx];
  		le = &linkERTable[i];
		// stored FIFO, so not continue
  		if (!(le->flags & VALID_FLAG))
  			break;
  		// only relay if I'm in the link's ER; one reason is to prevent flooding
  		if (!(le->flags & IS_IN_ER_FLAG))
  			continue;
  		// load
  		er_footer_p = &er_footer[k];
  		er_footer_p->sender = le->sender;
  		er_footer_p->receiver = le->receiver;
  		er_footer_p->rx_interference_threshold = le->rx_interference_threshold;
  		er_footer_p->rx_er_version = le->rx_er_version;
 		// to be re-sorted
  		le->flags |= RANK_INCREMENT_FLAG;
 		//call UartLog.logTxRx(DBG_FLAG, DBG_ER_FLAG, __LINE__, le->prio, le->sender, le->receiver, le->rx_interference_threshold, le->rx_er_version, seqno);
  		k++;
  	}
  	// re-sort
	sortLinkERTable();
	return k;
}

// packet layout: header | payload | pdr footer | er footer
// add the header in the packet. Call just before sending the packet
// add pdr first and then er
uint8_t addLinkEstHeaderAndFooter(message_t *msg, uint8_t len) {
	int16_t i, j, k;
	uint8_t newlen;
	uint8_t link_pdr, link_pdr_sample;
	uint8_t link_pdr_version = 0;
	
	imac_control_header_t *hdr;
	local_link_pdr_footer_t *pdr_footer, *pdr_footer_p;
	link_er_footer_t *er_footer;

	local_link_er_table_entry_t *se;
	
	hdr = getHeader(msg);
	pdr_footer = (local_link_pdr_footer_t *)getFooter(msg, len);
	
	//// footer
	// PDR only from DATA receiver to sender
	// distinguish redundant PDR entry by inquality of INVALID_INQUALITY (i.e., 255)
	j = 0;
	for (i = 0; i < LOCAL_LINK_ER_TABLE_SIZE; i++) {
		se = &localLinkERTable[i];
		if (!se->valid)
			break;
		pdr_footer_p = &pdr_footer[j++];
		// sender
		if (se->is_sender) {
  			pdr_footer_p->inquality = INVALID_INQUALITY;
  			// leave version alone since it will be ignored upon rx anyway
  			pdr_footer_p->next_slot = se->next_slot_by_tx;
		} else {
			call LinkEstimator.getInDataPdr(se->nb, &link_pdr, &link_pdr_sample, &link_pdr_version);
			// already converted from base 255 to 100
  			pdr_footer_p->inquality = link_pdr;
  			pdr_footer_p->inquality_version = link_pdr_version;
  			pdr_footer_p->next_slot = se->next_slot_by_rx;
		}
	}
  	
  	// ER
  	er_footer = (link_er_footer_t *)((uint8_t *)getFooter(msg, len) + j * sizeof(local_link_pdr_footer_t));
//  	k = 0;
//  	// top MAX_ITEM_CNT priority links whose ER I'm in
//  	for (i = 0; i < LINK_ER_TABLE_SIZE && k < const_max_item_cnt; i++) {
//  		// indirect sort
//  		//idx = sortedIndices[i];
//  		//le = &linkERTable[idx];
//  		le = &linkERTable[i];
//		// stored FIFO, so not continue
//  		if (!le->valid)
//  			break;
//  		// only relay if I'm in the link's ER; one reason is to prevent flooding
//  		if (!(le->flags & IS_IN_ER_FLAG))
//  			continue;
//  		// load
//  		er_footer_p = &er_footer[k];
//  		er_footer_p->sender = le->sender;
//  		er_footer_p->receiver = le->receiver;
//  		er_footer_p->rx_interference_threshold = le->rx_interference_threshold;
//  		er_footer_p->rx_er_version = le->rx_er_version;
// 		// to be re-sorted
//  		le->flags |= RANK_INCREMENT_FLAG;
// 		//call UartLog.logTxRx(DBG_FLAG, DBG_ER_FLAG, __LINE__, le->prio, le->sender, le->receiver, le->rx_interference_threshold, le->rx_er_version, seqno);
//  		k++;
//  	}
//  	// re-sort
//	sortLinkERTable();
	k = call Controller.loadLinkER(er_footer);
	
	
  	//// header
  	//hdr->is_data_pending = call ForwarderInfo.isDataPending();
  	hdr->local_link_pdr_cnt = j;
  	hdr->link_er_cnt = k;
  	hdr->seqno = seqno++;
  	newlen = sizeof(imac_control_header_t) + len + j * sizeof(local_link_pdr_footer_t) + k * sizeof(link_er_footer_t);
//	call UartLog.logTxRx(DBG_FLAG, DBG_ER_FLAG, hdr->local_link_pdr_cnt, hdr->link_er_cnt, newlen, sizeof(link_er_footer_t), sizeof(local_link_pdr_footer_t), __LINE__, hdr->seqno);
	return newlen;
}

// done sending the message that originated by the user of this component
async event void SubSend.sendDone(message_t* msg, error_t error) {
//	imac_control_header_t *hdr = getHeader(msg);
//	call UartLog.logTxRx(DBG_FLAG, DBG_ER_FLAG, __LINE__, error, hdr->local_link_pdr_cnt, hdr->link_er_cnt, call Packet.payloadLength(msg), call SubPacket.payloadLength(msg), hdr->seqno);
	signal AMSend.sendDone(msg, error);
}

// cascade the calls down
async command uint8_t AMSend.cancel(message_t* msg) {
	return call SubSend.cancel(msg);
}

async command uint8_t AMSend.maxPayloadLength() {
	return call Packet.maxPayloadLength();
}

async command void* AMSend.getPayload(message_t* msg, uint8_t len) {
	return call Packet.getPayload(msg, len);
}

//-------------------------------------------------------------------------------------
// Interface Receive
//-------------------------------------------------------------------------------------
// new messages are received here
void processReceivedMessage(message_t *msg, void *payload, uint8_t len);

async event message_t* SubReceive.receive(message_t* msg, void* payload, uint8_t len) {
	// runtime check
	if (call Packet.payloadLength(msg) > call Packet.maxPayloadLength()) {
		// can occur bcoz of packet corruption
		//assert(0);
		//imac_control_header_t *hdr = getHeader(msg);
		//call UartLog.logTxRx(DBG_FLAG, DBG_ERR_FLAG, __LINE__, call SubAMPacket.source(msg), hdr->local_link_pdr_cnt, hdr->link_er_cnt, call Packet.payloadLength(msg), len, hdr->seqno);
		return msg;
	}
	processReceivedMessage(msg, payload, len);
	return signal Receive.receive(msg, call Packet.getPayload(msg, call Packet.payloadLength(msg)), call Packet.payloadLength(msg));
}

// called when signal map generated packet or packets from upper layer that are wired to pass through
// signal map is received
void processReceivedMessage(message_t *msg, void *payload, uint8_t len) {
	am_addr_t from = call SubAMPacket.source(msg);
	imac_control_header_t* hdr = getHeader(msg);
	local_link_pdr_footer_t *pdr_footer = (local_link_pdr_footer_t *)getFooter(msg, call Packet.payloadLength(msg));
	link_er_footer_t *er_footer = (link_er_footer_t *)(hdr->local_link_pdr_cnt * sizeof(local_link_pdr_footer_t) + (uint8_t *)getFooter(msg, call Packet.payloadLength(msg)));
	
	//- 4 - hdr->local_link_pdr_cnt * 4 - hdr->link_er_cnt * 8	
//	call UartLog.logTxRx(DBG_FLAG, DBG_ER_FLAG, __LINE__, from, hdr->local_link_pdr_cnt, hdr->link_er_cnt, call Packet.payloadLength(msg), len, hdr->seqno);
//#warning non-saturate
	// send has no data to tx, do not expect data from it
//	if (!hdr->is_data_pending) {
//		call Controller.clearDataPending(from);
//		call UartLog.logEntry(DBG_FLAG, DBG_ER_FLAG, __LINE__, from);
//	}
	
	updateLinkPdr(from, pdr_footer, hdr->local_link_pdr_cnt);
	call Controller.updateLinkERTable(er_footer, hdr->link_er_cnt, from, hdr->seqno);
}

// update next slot of a link, tx from a sender or receiver
inline async command void Controller.updateNextSlot(am_addr_t from, bool is_from_sender, uint32_t next_slot) {
	uint8_t idx;
	local_link_er_table_entry_t *se;

	// is_from_sender is !is_sender
	idx = findLocalLinkERTableIdx(from, !is_from_sender);
	if (idx >= LOCAL_LINK_ER_TABLE_SIZE)
		return;
	se = &localLinkERTable[idx];
	if (is_from_sender) {
		se->next_slot_by_tx = next_slot;
	} else {
		se->next_slot_by_rx = next_slot;
	}
}

// update outbound
void updateLinkPdr(am_addr_t from, local_link_pdr_footer_t *footer, uint8_t size) {
	uint8_t i;
	bool is_from_sender;
	local_link_pdr_footer_t *lp;
	
	// each entry
	for (i = 0; i < size; i++) {
		lp = &footer[i];
		// MATURE_ENTRY outbound PDR for me
		if (lp->nb == my_ll_addr) {
			is_from_sender = (lp->inquality == INVALID_INQUALITY);
			// valid pdr; from receiver
			if (!is_from_sender)
				call LinkEstimator.updateNeighborOutQuality(from, lp->inquality, lp->inquality_version);
			call Controller.updateNextSlot(from, is_from_sender, lp->next_slot);
			// do not break here for bidirectional links
			//break;
		}
	}
}

// update link ER
// also called from Forwarder$SubReceive$receive & Forwarder$SubSnoop$receive 
async command void Controller.updateLinkERTable(link_er_footer_t *footer, uint8_t size, am_addr_t from, uint16_t seqno_) {
	uint8_t i;
	link_er_footer_t *lp;

	// each entry
	for (i = 0; i < size; i++) {
		lp = &footer[i];
		// ER
		updateLinkERTableEntry(lp->sender, lp->receiver, from, lp->rx_interference_threshold, lp->rx_er_version, FALSE);
		//call UartLog.logTxRx(DBG_FLAG, DBG_ER_FLAG, __LINE__, from, lp->sender, lp->receiver, lp->rx_interference_threshold, lp->rx_er_version, seqno_);
	}
	
	// do not update contention relationship till all ER update committed
	updateContentionTable();
}

//-------------------------------------------------------------------------------------
// Interface Packet
//-------------------------------------------------------------------------------------
async command void Packet.clear(message_t* msg) {
	call SubPacket.clear(msg);
}

// subtract the space occupied by the control header and footer from the incoming payload size
async command uint8_t Packet.payloadLength(message_t* msg) {
	imac_control_header_t *hdr = getHeader(msg);
	
	return (call SubPacket.payloadLength(msg) - sizeof(imac_control_header_t) - hdr->local_link_pdr_cnt * sizeof(local_link_pdr_footer_t) - hdr->link_er_cnt * sizeof(link_er_footer_t));
}

// account for the space used by header and footer while setting the payload length
async command void Packet.setPayloadLength(message_t* msg, uint8_t len) {
	imac_control_header_t *hdr = getHeader(msg);

	call SubPacket.setPayloadLength(msg, len + sizeof(imac_control_header_t) + hdr->local_link_pdr_cnt * sizeof(local_link_pdr_footer_t) + hdr->link_er_cnt * sizeof(link_er_footer_t));
}

async command uint8_t Packet.maxPayloadLength() {
	return (call SubPacket.maxPayloadLength() - sizeof(imac_control_header_t));
}

// application payload pointer is just past the link estimation header
async command void* Packet.getPayload(message_t* msg, uint8_t len) {
	void* payload = call SubPacket.getPayload(msg, len + sizeof(imac_control_header_t));
	if (payload != NULL) {
		payload += sizeof(imac_control_header_t);
	}
	return payload;
}


//-------------------------------------------------------------------------------------------------
// Interface IMACController
//-------------------------------------------------------------------------------------------------
async command local_link_er_table_entry_t *Controller.getLocalLinkERTable() {
	return localLinkERTable;
}

async command link_er_table_entry_t *Controller.getLinkERTable() {
	return linkERTable;
}

// am I a sender in the current slot
// @param is_tx_based: is sender slot computed by sender; only in this case recompute next tx slot
inline async command bool Controller.isTxSlot(uint32_t current_slot) {
	local_link_er_table_entry_t *se;
//	#warning dedicate these slots to ctrl channel, more specifically for ftsp
	if ((current_slot & SLOT_MASK) < (MAX_SLOT_FORWARD - active_link_size))
		return FALSE;
		
	if (my_local_link_idx >= LOCAL_LINK_ER_TABLE_SIZE)
		return FALSE;
	se = &localLinkERTable[my_local_link_idx];
	// either determined by the sender or the receiver
	// use >=, not >, next_slot_by_tx to deal w/ missing slot due to imperfect slotting, including INVALID_TIME
	// not use >= next_slot_by_rx bcoz next_slot_by_rx not updated immediately; anyway it's opportunistic
	//return (current_slot >= se->next_slot_by_tx || current_slot == se->next_slot_by_rx);
	// time wrap around
	// check se->next_slot_by_tx bcoz current_slot is always "larger" than se->next_slot_by_rx of INVALID_SLOT bcoz of time wrap around; not check se->next_slot_by_rx bcoz it is "==" comparison
//	return ((se->next_slot_by_tx != INVALID_SLOT && (int32_t)(current_slot - se->next_slot_by_tx) >= 0) || current_slot == se->next_slot_by_rx);
	// slot * SLOT_LEN to recover global time, which wraps around every 2 ^ 32, not slot itself
	return ((se->next_slot_by_tx != INVALID_SLOT && (int32_t)(current_slot * SLOT_LEN - se->next_slot_by_tx * SLOT_LEN) >= 0) || current_slot == se->next_slot_by_rx); 
}

// am I a receiver in the current slot
// called from Forwarder$scheduleSlot()
// stay in DATA channel iff highest local link is expecting DATA, i.e., do not stay if no sender wins locally and thus impossible to tx
inline async command bool Controller.isRxSlot(uint32_t current_slot) {
	bool is_rx_pending;
	uint8_t i, t, max_prio, prio;
	local_link_er_table_entry_t *se;
	
//	#warning dedicate these slots to ctrl channel, more specifically for ftsp
	if ((current_slot & SLOT_MASK) < (MAX_SLOT_FORWARD - active_link_size))
		return FALSE;

	// nodes not tx/rx DATA stay in CTRL channel
	is_rx_pending = FALSE;
	t = current_slot & SLOT_MASK;
	max_prio = 0;
	for (i = 0; i < LOCAL_LINK_ER_TABLE_SIZE; i++) {
		se = &localLinkERTable[i];
		if (!se->valid)
			continue;
		
		// incoming link only	
		if (!se->is_sender) {
			//call UartLog.logTxRx(DBG_FLAG, DBG_TDMA_FLAG, __LINE__, se->is_rx_pending, (se->next_slot_by_tx != INVALID_SLOT && (int32_t)(current_slot - se->next_slot_by_tx) >= 0) || current_slot == se->next_slot_by_rx, se->next_slot_by_rx, se->next_slot_by_tx >> 16, se->next_slot_by_tx, current_slot);
			// either determined by the sender or the receiver
			// use >=, not >, next_slot_by_tx to deal w/ missing slot due to imperfect slotting
			//if (current_slot >= se->next_slot_by_tx || current_slot == se->next_slot_by_rx) {
			if ((se->next_slot_by_tx != INVALID_SLOT && (int32_t)(current_slot * SLOT_LEN - se->next_slot_by_tx * SLOT_LEN) >= 0) || current_slot == se->next_slot_by_rx) {
				// set pending bcoz will expect DATA after the call
				se->is_rx_pending = TRUE;
			}
		}
		
		// also consider outgoing link, whose is_rx_pending is always FALSE
		prio = se->link_idx ^ t;
		if (max_prio < prio || 0 == max_prio) {
			max_prio = prio;
			is_rx_pending = se->is_rx_pending;
		}
	}
	// highest priority link is expecting DATA
	return is_rx_pending;
}

//inline async command bool Controller.isRxSlot(uint32_t current_slot) {
//	uint8_t i;
//	local_link_er_table_entry_t *se;
//	
//	for (i = 0; i < LOCAL_LINK_ER_TABLE_SIZE; i++) {
//		se = &localLinkERTable[i];
//		if (!se->valid)
//			continue;
//		if (se->is_sender)
//			continue;
//		// either determined by the sender or the receiver
//		// use >=, not >, next_slot_by_tx to deal w/ missing slot due to imperfect slotting
//		if (current_slot >= se->next_slot_by_tx || current_slot == se->next_slot_by_rx) {
//			// set pending bcoz will expect DATA after the call
//			se->is_rx_pending = TRUE;
//			return TRUE;
//		}
//	}
//	return FALSE;
//}

// get next tx slot; only called at sender from Forwarder$scheduleSlot()
// async command uint32_t Controller.getNextTxSlot() {
// 	local_link_er_table_entry_t *se;
// 	
// 	if (my_local_link_idx >= LOCAL_LINK_ER_TABLE_SIZE)
// 		return INVALID_SLOT;
// 	se = &localLinkERTable[my_local_link_idx];
// 	return se->next_slot_by_tx;
// }

// @return: my outgoing link's index in localLinkERTable
async command uint8_t Controller.findMyLinkLocalIdx() {
	uint8_t my_local_link_idx_;
	atomic my_local_link_idx_ = my_local_link_idx;
	return my_local_link_idx_;
}

// indicate an expected DATA packet is received
inline async command void Controller.clearDataPending(am_addr_t from) {
	uint8_t idx;
	local_link_er_table_entry_t *se;

	idx = findLocalLinkERTableIdx(from, FALSE);
	if (idx >= LOCAL_LINK_ER_TABLE_SIZE)
		return;
	se = &localLinkERTable[idx];
	se->is_rx_pending = FALSE;
}

// get the version of latest rx ER being used
inline async command uint8_t Controller.getNbERVer(am_addr_t nb, bool is_sender) {
	uint8_t idx;
	link_er_table_entry_t *le;
	local_link_er_table_entry_t *se;

	if (is_sender) {
		// rx ER is at the receiver
		idx = call Controller.findLinkERTableIdx(my_ll_addr, nb);
		if (idx >= LINK_ER_TABLE_SIZE) {
			// this can occur when receiver is not programmed
			//assert(nb);
			return 0;
		}
		le = &linkERTable[idx];
		return le->rx_er_version;
	} else {
		// search in localLinkERTable to save time, even can also search in linkERTable
		idx = findLocalLinkERTableIdx(nb, FALSE);
		if (idx >= LOCAL_LINK_ER_TABLE_SIZE) {
			assert(0);
			return 0;
		}
		se = &localLinkERTable[idx];
		return se->rx_er_version;		
	}
}

// update NI for a link when a new NI sample arrives, i.e., when DATA received
// one reason to change I(t) estimation from arithmetic mean to moving average is bcoz of ease of computation using dbmWeightedSumS
// called from LinkEstimatorP$updateNeighborInOutQuality()
async command error_t Controller.sampleNI(message_t *msg) {
//	uint8_t idx;
//	int16_t in_gain, out_gain;
//	// scaled
//	int16_t tx_rss, total_rss, ni;
//	am_addr_t nb;
//	dbm_t nI;
//	local_link_er_table_entry_t *se;
//	
//	nb = call SubAMPacket.source(msg);
//	// sample link NI: ni  = total signal - signal by sender (based on signal map)
//	if (call SignalMap.getLocalGain(nb, &in_gain, &out_gain) != SUCCESS)
//		return FAIL;
//	tx_rss = CC2420_DEF_RFPOWER_DBM_SCALED - in_gain;
//	// total signal during rx, including sender signal and ni
//	total_rss = call CC2420Packet.getRssi(msg);
//	// scale up
//	total_rss <<= SCALE_L_SHIFT_BIT;
//	call UartLog.logTxRx(DBG_FLAG, DBG_CONTROLLER_FLAG, __LINE__, nb, in_gain / 128, -tx_rss, -total_rss, -tx_rss / 128, -total_rss / 128);
//	if (total_rss < tx_rss)
//		#warning return FAIL;
//		return 3;
//	ni = call Controller.dbmDiffU(total_rss, tx_rss);
//	
////	// TODO: filter outliers, mostly, if not all, are 83 (255 - 172)
////	if (ni >= 0) {
////		assert(0);
////		return FAIL;
////	}

	uint8_t idx;
	// scaled
	int16_t total_rss, ni;
	am_addr_t nb;
	dbm_t nI;
	local_link_er_table_entry_t *se;
	
	nb = call SubAMPacket.source(msg);
//	// total signal during rx, including sender signal and ni
//	total_rss = call CC2420Packet.getRssi(msg);
	total_rss = call CC2420Packet.getRssiIdle(msg);
	// scale up
	total_rss <<= SCALE_L_SHIFT_BIT;
	ni = total_rss;
	
	nI.sign = 1;
	nI.abs = ni;

	idx = findLocalLinkERTableIdx(nb, FALSE);
	if (idx >= LOCAL_LINK_ER_TABLE_SIZE)
		return FAIL;
	
	se = &localLinkERTable[idx];
	if (se->rx_nI.sign != 0) {
		se->rx_nI = dbmWeightedSumS(se->rx_nI, nI);
	} else {
		se->rx_nI = nI;
	}
	return SUCCESS;
//	// node NI
//	//if (node_I.abs != INVALID_DBM) {
//	if (node_I.sign != 0) {
//		//node_i = node_i - (node_i >> 4) + (ni >> 4);
//		node_I = dbmWeightedSumS(node_I, nI);
//	} else {
//		node_I = nI;
//	}
}


// -------------------------------------------------------------------------------------------------
//	controller
// -------------------------------------------------------------------------------------------------
// compute control output based on input
// scaled by 2^SCALE_L_SHIFT_BIT
int32_t pdr2DeltaIdB(uint8_t link_pdr, uint8_t link_pdr_sample, uint8_t reference_pdr) {
	uint8_t table_size;
	// scale slope for accuracy; otherwise significant precision loss, e.g., 3.6296 vs 3
	int64_t slope, deltaI_dB;
	
	//start_time = call LocalTime.get();
	// just to be cautious and prevent arrayoutofbound
	table_size = sizeof(pdr_slope_table) / sizeof(pdr_slope_table[0]);
	if (link_pdr >= table_size)
		link_pdr = table_size - 1;
//#warning	pdr sample, not EWMA
//	// 1 / a
//	slope = (int32_t)SLOPE_SCALE * pdr_slope_table[link_pdr_sample] / PDR_SLOPE_SCALE;
//	// 1 / a_r: special care when PDR is far away from reference
//	// a_0 = abs((T(S, R) - y(t)) / (f^(-1)(T(S, R)) - f^(-1)(y(t))))
//	// slope = 1 / a_0
//	// pdr_inv_table and link_pdr both scaled by 100
//	if ((link_pdr_sample > reference_pdr) && (link_pdr_sample - reference_pdr) > E0) {
//		slope = (int32_t)SLOPE_SCALE * (pdr_inv_table[link_pdr_sample] - pdr_inv_table[reference_pdr]) / (link_pdr_sample - reference_pdr);
//	}
//	if ((reference_pdr > link_pdr_sample) && (reference_pdr - link_pdr_sample) > E0) {
//		slope = (int32_t)SLOPE_SCALE * (pdr_inv_table[reference_pdr] - pdr_inv_table[link_pdr_sample]) / (reference_pdr - link_pdr_sample);
//	}
	// 1 / a
	slope = (int32_t)SLOPE_SCALE * pdr_slope_table[link_pdr] / PDR_SLOPE_SCALE;
	// 1 / a_r: special care when PDR is far away from reference
	// a_0 = abs((T(S, R) - y(t)) / (f^(-1)(T(S, R)) - f^(-1)(y(t))))
	// slope = 1 / a_0
	// pdr_inv_table and link_pdr both scaled by 100
	if ((link_pdr > reference_pdr) && (link_pdr - reference_pdr) > E0) {
		slope = (int32_t)SLOPE_SCALE * (pdr_inv_table[link_pdr] - pdr_inv_table[reference_pdr]) / (link_pdr - reference_pdr);
	}
	if ((reference_pdr > link_pdr) && (reference_pdr - link_pdr) > E0) {
		slope = (int32_t)SLOPE_SCALE * (pdr_inv_table[reference_pdr] - pdr_inv_table[link_pdr]) / (reference_pdr - link_pdr);
	}
	
	//deltaI_dB = slope * nominator * 128 * 10 / (10 - ALPHA) / SLOPE_SCALE / 100;
	deltaI_dB = (((slope * ALPHA * link_pdr + slope * (10 - ALPHA) * link_pdr_sample) / 10 - slope * (reference_pdr - DELTA_Y)) << SCALE_L_SHIFT_BIT) * 10 / (10 - ALPHA) / SLOPE_SCALE / 100;
	//call UartLog.logTxRx(DBG_FLAG, DBG_CONTROLLER_FLAG, __LINE__, link_pdr, link_pdr_sample, slope, 0, deltaI_dB, call LocalTime.get() - start_time);
	return deltaI_dB;
}

// new inbound data/ack pdr arrives
// ER is updated here, not on per pkt basis
// @param is_sender: ACK reliability updated or data
async event error_t LinkEstimator.inLinkPdrUpdated(am_addr_t nb, bool is_sender) {
	error_t ret = execController(nb, is_sender);
	return ret;
}

async command uint8_t Controller.getLinkPdrReq() {
	return pdr_req;
}
async command void Controller.setLinkPdrReq(uint8_t new_req) {
	pdr_req = new_req;
}

// consider taskize if too much computation
error_t execController(am_addr_t nb, bool is_sender) {
	int16_t idx;
	error_t ret;
	local_link_er_table_entry_t *se;

	uint8_t link_pdr, link_pdr_sample, reference_pdr, link_pdr_version;
	// scaled
	int32_t next_i, delta_i_dB;
	// encapsulation of dbm
	dbm_t current_I, delta_I, next_I;

	int16_t er_border_gain, in_gain, out_gain;
//	int32_t current_i
//	int16_t in_gain, out_gain;
//	// scaled
//	int16_t tx_rss;
	
	idx = findLocalLinkERTableIdx(nb, is_sender);
	if (idx >= LOCAL_LINK_ER_TABLE_SIZE) {
		return FAIL;
	}
	se = &localLinkERTable[idx];
	
	// sender uses inbound ACK pdr while receiver uses inbound data pdr
	if (is_sender) {
		return FAIL;
	} else {
//		// impossible to occur since one PDR sample, one NI sample
//		if (0 == se->rx_nI.sign) {
//			// no sample, wait till next update
//			return FAIL;
//		}
		ret = call LinkEstimator.getInDataPdr(nb, &link_pdr, &link_pdr_sample, &link_pdr_version);
		if (ret != SUCCESS)
			return ret;
		atomic reference_pdr = pdr_req;
		//reference_pdr = (TOS_NODE_ID & 0x1) ? 90 : 70;
		
		current_I = se->rx_nI;
		
//		if (call SignalMap.getLocalGain(nb, &in_gain, &out_gain) != SUCCESS)
//			return FAIL;
//		tx_rss = CC2420_DEF_RFPOWER_DBM_SCALED - in_gain;
//		call UartLog.logTxRx(DBG_FLAG, DBG_CONTROLLER_FLAG, __LINE__, nb, in_gain / 128, -tx_rss, -current_I.abs, -tx_rss / 128, -current_I.abs / 128);
//		if (current_I.abs < tx_rss)
//			return FAIL;
//		// current_I includes sender signal
//		current_i = call Controller.dbmDiffU(current_I.abs, tx_rss);
//		current_I.sign = 1;
//		current_I.abs = current_i;
//		
		// reset for next interval
		se->rx_nI.sign = 0;
	}
	
//	if (link_pdr != 0) {
	// THEOREM 1
	delta_i_dB = pdr2DeltaIdB(link_pdr, link_pdr_sample, reference_pdr);
	// next_i =  current_i + delta_i + mean_delta_i_u(i.e., 0 by assumption)
	next_i = current_I.abs + delta_i_dB;
	next_I.sign = 1;
	next_I.abs = next_i;
	delta_I = dbmDiffS(next_I, current_I);
//	} else {
//		// TODO: this should only happen when pdr is uninitialized in iMAC simple test since no BSL issue here
//		// if we make sure the other end of the active link is in my neighbor table: size larger than network size
//		// fix ER to initialize link PDR
//		delta_I.sign = 0;
//		delta_I.abs = 0;
//	}
	// to compute model parameter K in PRK
	er_border_gain = (se->rx_er_border_idx != EMPTY_ER_IDX) ? signalMap[se->rx_er_border_idx].inbound_gain : INVALID_GAIN;
	if (call SignalMap.getLocalGain(nb, &in_gain, &out_gain) != SUCCESS)
		in_gain = INVALID_GAIN;
	call UartLog.logTxRx(DBG_FLAG, DBG_CONTROLLER_FLAG, __LINE__, nb, link_pdr, link_pdr_sample, se->rx_er_border_idx, er_border_gain/128, in_gain/128);
//	call UartLog.logTxRx(DBG_FLAG, DBG_CONTROLLER_FLAG, __LINE__, nb, link_pdr, link_pdr_sample, se->rx_er_border_idx, er_border_gain, delta_i_dB);
//	#warning fake pdr 95% to prevent abrupt ER shrink
//	if (link_pdr >= 95 && delta_i_dB < 0)
//		return SUCCESS;
	
	ret = adjustER(idx, is_sender, delta_I);
	if (ret != SUCCESS)
		assert(ret);
	return ret;
}

// wrapper
// regard ER change here instead of updateBorder() bcoz even ER border stays, ER threshold can still change due to link gain changes
error_t udpateER(bool is_sender, local_link_er_table_entry_t *se, int16_t gain, int16_t delta) {
	if (INVALID_GAIN == gain) {
		return FAIL;
	} else {
		if (is_sender) {
			return FAIL;
		} else {
			se->rx_interference_threshold = CC2420_DEF_RFPOWER_DBM - (gain >> SCALE_L_SHIFT_BIT) + delta;
			se->rx_er_version++;
			// new rx ER
			updateLinkERTableEntry(se->nb, my_ll_addr, my_ll_addr, se->rx_interference_threshold, se->rx_er_version, TRUE);
			updateContentionTable();
//			call UartLog.logTxRx(DBG_FLAG, DBG_ER_FLAG, __LINE__, sender, receiver, -le->tx_interference_threshold, -le->rx_interference_threshold, le->tx_interference_threshold, le->rx_interference_threshold);
			return SUCCESS;
		}
	}
}

inline void updateBorder(bool is_sender, local_link_er_table_entry_t *se, int16_t i) {
	if (is_sender) {
#ifdef TX_ER
		se->tx_er_border_idx = i;
#endif
	} else {
		se->rx_er_border_idx = i;
	}
}

// based on delta_I, update ER
error_t adjustER(int16_t idx, bool is_sender, dbm_t delta_I) {
	// signed to avoid inf loop due to i >= 0
	int16_t i, sm_size, er_border_idx;
	// point to the entry of the neighbor
	local_link_er_table_entry_t *le;
	sm_entry_t *se;
	int32_t delta_i;
	// scaled; otherwise dBm addition, substraction may cause great precision loss
	int32_t total_i;
	
	// runtime check
	if (idx >= LOCAL_LINK_ER_TABLE_SIZE) {
		assert(0);
		return FAIL;
	}

	le = &localLinkERTable[idx];
	// current ER border
	if (is_sender) {
	#ifdef TX_ER
		er_border_idx = le->tx_er_border_idx;
	#endif
	} else {
		er_border_idx = le->rx_er_border_idx;	
	}
	
	sm_size = call SignalMap.getSignalMapSize();
	
	delta_i = delta_I.abs;
	// enlarge exclusion region to reduce interference
	if (delta_I.sign < 0) {
		// cannot enlarge further; already largest
		if ((er_border_idx + 1) >= sm_size) {
			se = &signalMap[er_border_idx];
			return udpateER(is_sender, le, se->inbound_gain, 0);
		}
		
		total_i = INVALID_DBM;
		for (i = er_border_idx + 1; i < sm_size; i++) {
			se = &signalMap[i];
			if (INVALID_GAIN == se->inbound_gain) {
				return FAIL;
			}
			if (total_i != INVALID_DBM) {
				total_i = dbmSumU(total_i, CC2420_DEF_RFPOWER_DBM_SCALED - se->inbound_gain);
			} else {
				total_i = CC2420_DEF_RFPOWER_DBM_SCALED - se->inbound_gain;
			}
			// maximally enlarge the ER but still total_i >= |delta_i|
			if (total_i >= delta_i)
				break;
		}
//		// to make PDR not above requirement
//		if (total_i > delta_i)
//			i--;
		//call UartLog.logTxRx(DBG_FLAG, DBG_CONTROLLER_FLAG, __LINE__, er_border_idx, i, sm_size, se->inbound_gain, total_i, delta_i);
		// even the entire SM is not sufficient for the exclusion region; try the largest
		if (i >= sm_size)
			i--;
		// update boundary
		updateBorder(is_sender, le, i);
		se = &signalMap[i];
		return udpateER(is_sender, le, se->inbound_gain, 0);
	} else if (delta_I.sign > 0) {
		// shrink exclusion region to increase interference

		// cannot shrink further; already empty
		if (EMPTY_ER_IDX == er_border_idx) {
			se = &signalMap[0];
			// add OFFSET so even "closest" neighbor is excluded
			return udpateER(is_sender, le, se->inbound_gain, OFFSET);
		}
		
		total_i = INVALID_DBM;
		// assert(er_border_idx >= 0)
		// start from er_border_idx, not er_border_idx - 1, bcoz it is within ER
		for (i = er_border_idx; i >= 0; i--) {
			se = &signalMap[i];
			if (INVALID_GAIN == se->inbound_gain) {
				return FAIL;
			}
			if (total_i != INVALID_DBM) {
				total_i = dbmSumU(total_i, CC2420_DEF_RFPOWER_DBM_SCALED - se->inbound_gain);
			} else {
				total_i = CC2420_DEF_RFPOWER_DBM_SCALED - se->inbound_gain;
			}
			// maximally shrink the ER but still total_i <= |delta_i|
			if (total_i > delta_i)
				break;
		}
		//call UartLog.logTxRx(DBG_FLAG, DBG_CONTROLLER_FLAG, __LINE__, er_border_idx, i, sm_size, se->inbound_gain, total_i, delta_i);
		// ER excludes the last evicted negihbor
		// only if it exists
		if (i > EMPTY_ER_IDX)
			i--;
		
//		// to make PDR not below requirement
//		if (total_i > delta_i)
//			i++;
		
		// update boundary
		updateBorder(is_sender, le, i);
			
		if (i > EMPTY_ER_IDX) {
			se = &signalMap[i];
			return udpateER(is_sender, le, se->inbound_gain, 0);
		} else {
			// empty ER
			se = &signalMap[0];
			return udpateER(is_sender, le, se->inbound_gain, OFFSET);
		}
	} else {
		// maintain current exclusion region
		if (er_border_idx > EMPTY_ER_IDX) {
			se = &signalMap[er_border_idx];
			return udpateER(is_sender, le, se->inbound_gain, 0);
		} else {
			// empty ER
			se = &signalMap[0];
			return udpateER(is_sender, le, se->inbound_gain, OFFSET);
		}
	}
}


default async event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len) {
	return msg;
}

}

