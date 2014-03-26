/*
 * @ author: Xiaohui Liu (whulxh@gmail.com)
 * 
 * @ updated: 	04/07/2012 03:23:55 PM 
 * @ updated:	04/08/2012 09:42:00 PM  
 				wire data plane thru link estimator, BUT only data packet and ACK contributes to estimation, not 					RTS/CTS; they are differentiated by check whether they are broadcast or not
 * @ acknowledgement: based on 4bitle
 * @ description: add inbound/outbound quality; remove footer since it is not used anyway
 */
 
#include "LinkEstimator.h"

module LinkEstimatorP {
	provides {
		interface AsyncAMSend as AMSend;
		interface AsyncReceive as Receive;
		interface AsyncReceive as Snoop;
		interface AsyncPacket as Packet;

		interface LinkEstimator;
		interface Init;
	}

	uses {
		interface AsyncAMSend as SubSend;
		interface AsyncReceive as SubReceive;
		interface AsyncReceive as SubSnoop;
		interface AsyncPacket as SubPacket;
		interface AsyncAMPacket as SubAMPacket;
		
		interface IMACController as Controller;
		interface Util;
		interface UartLog;
//		interface LocalTime<T32khz>;
	}
}

implementation {
// TODO: remove broadcast case

// keep information about links from the neighbors
neighbor_table_entry_t NeighborTable[NEIGHBOR_TABLE_SIZE];
// link estimation sequence, increment every time a beacon is sent
uint8_t linkEstSeq = 0;
// if there is not enough room in the packet to put all the neighbor table
// entries, in order to do round robin we need to remember which entry
// we sent in the last beacon
uint8_t prevSentIdx = 0;

am_addr_t my_ll_addr;
// get the link estimation header in the packet
linkest_header_t* getHeader(message_t* m) {
	return (linkest_header_t*)call SubPacket.getPayload(m, sizeof(linkest_header_t));
}

// get the link estimation footer (neighbor entries) in the packet
linkest_footer_t* getFooter(message_t* m, uint8_t len) {
// To get a footer at offset "len", the payload must be len + sizeof large.
	return (linkest_footer_t*)(len + (uint8_t *)call Packet.getPayload(m, len + sizeof(linkest_footer_t)));
}

uint16_t ver_match_cnt;
uint16_t ver_mismatch_cnt;

//-------------------------------------------------------------------------------------
//	table management
//-------------------------------------------------------------------------------------
// initialize the given entry in the table for neighbor ll_addr
void initNeighborIdx(uint8_t i, am_addr_t ll_addr) {
	neighbor_table_entry_t *ne;
	ne = &NeighborTable[i];
	
	ne->ll_addr = ll_addr;
	ne->lastseq = 0;
	ne->rcvcnt = 0;
	ne->failcnt = 0;
	ne->flags = (INIT_ENTRY | VALID_ENTRY);
	// essentially means no sample comes to this EWMA yet; set it as 0 would slow down the rising
	//ne->inquality = 0;
	ne->inquality = INVALID_INQUALITY;
	ne->inquality_version = 0;
	ne->inquality_sample = 0;
	ne->outquality = 0;
	// make this initial version # far away from inquality's initial version 0
	ne->last_outquality_version = 100;
	ne->ack_quality_sample = 0;
	ne->eetx = 0;
}

// find the index to the entry for neighbor ll_addr
uint8_t findIdx(am_addr_t ll_addr) {
	uint8_t i;
	
	atomic {
		for (i = 0; i < NEIGHBOR_TABLE_SIZE; i++) {
			if (NeighborTable[i].flags & VALID_ENTRY) {
				if (NeighborTable[i].ll_addr == ll_addr) {
					return i;
				}
			}
		}
	}
	return INVALID_RVAL;
}

// find an empty slot in the neighbor table
uint8_t findEmptyNeighborIdx() {
	uint8_t i;
	
	atomic {
		for (i = 0; i < NEIGHBOR_TABLE_SIZE; i++) {
			if (NeighborTable[i].flags & VALID_ENTRY) {
			} else {
				return i;
			}
		}
	}
	return INVALID_RVAL;
}

// update the EETX estimator
// not called when new beacon estimate is done
// only called when new DEETX estimate is done
void updateEETX(neighbor_table_entry_t *ne, uint16_t newEst) {
	ne->eetx = (ALPHA * ne->eetx + (10 - ALPHA) * newEst) / 10;
}

// update data driven EETX
void updateDEETX(neighbor_table_entry_t *ne) {
	uint16_t estETX;

	ne->flags |= EETX_MATURE_ENTRY;
	if (ne->data_success == 0) {
		// if there were no successful packet transmission in the
		// last window, our current estimate is the number of failed
		// transmissions
		estETX = (ne->data_total - 1) * 10;
	} else {
		estETX = (10 * ne->data_total) / ne->data_success - 10;
		ne->data_success = 0;
		ne->data_total = 0;
	}
	updateEETX(ne, estETX);
#ifdef TX_ER
	//inbound ACK pdr updated
	signal LinkEstimator.inLinkPdrUpdated(ne->ll_addr, TRUE);
#endif
	if (ne->outquality != 0) {
		uint16_t bi_pdr = 1000 / (estETX + 10);
		// only sample ack y(t) upon bidirectional update, not outquality update
		if (bi_pdr <= ne->outquality)
			ne->ack_quality_sample = (uint8_t)(100 * bi_pdr / ne->outquality);
		else
			ne->ack_quality_sample = 100;
	}
}


// EETX (Extra Expected number of Transmission)
// EETX = ETX - 1
// computeEETX returns EETX*10
uint8_t computeEETX(uint8_t q1) {
	uint16_t q;
	if (q1 > 0) {
		q =  1000 / q1 - 10;
		if (q > 100) {
			q = VERY_LARGE_EETX_VALUE;
		}
		return (uint8_t)q;
	} else {
		return VERY_LARGE_EETX_VALUE;
	}
}

///*
// * called when update rx ER to update PDR (ne->inquality) using the whole window
// * postpone inbound PDR update till ER update so packets btw. now and (now + d0) are utilized
// * called from IMACControllerPUtil$IMACController$updateCurERContendFlags()
// */
//async command error_t LinkEstimator.updateNeighborTableEstD0(am_addr_t neighbor) {
//	//uint8_t totalPkt;
//	uint16_t totalPkt;
//	uint8_t newEst;
//	uint8_t idx;
//	neighbor_table_entry_t *ne;
//	
//    idx = findIdx(neighbor);
//    if (idx == INVALID_RVAL) {
//      	return FAIL;
//    }
//	ne = &NeighborTable[idx];
//	// TODO: deal w/ # of pkts overflow
//	totalPkt = ne->rcvcnt + ne->failcnt;
//	newEst = (100 * (uint16_t)ne->rcvcnt) / totalPkt;
//	// inquality sample in the latest window
//	// change to the alternative form; otherwise inequality cannot exceed 246 (pdr 96%)
//	//ne->inquality = (ALPHA * ne->inquality + (10-ALPHA) * newEst)/10;
//	ne->inquality = ne->inquality - (10 - ALPHA) * ne->inquality / 10 + (10 - ALPHA) * newEst / 10;
//	ne->rcvcnt = 0;
//	ne->failcnt = 0;
//	// clear update controller bit
//	ne->flags &= ~CONTROLLER_UPDATE_ENTRY;
//	ne->inquality_version++;
//	return SUCCESS;
//}

//// called when update controller
//// update ne->inquality_sample only w/ samples in current window so far
//void updateNeighborTableEst(uint8_t idx) {
//	uint8_t totalPkt;
//	uint8_t newEst;
//	neighbor_table_entry_t *ne;
//	
//	ne = &NeighborTable[idx];
//	ne->flags |= MATURE_ENTRY;
//	totalPkt = ne->rcvcnt + ne->failcnt;
//	newEst = (100 * (uint16_t)ne->rcvcnt) / totalPkt;
//	// inquality sample in the latest window
//	ne->inquality_sample = 100 * (uint16_t)newEst / 100;
//}


// update the inbound link quality by
// munging receive, fail count since last update
void updateNeighborTableEst(uint8_t idx) {
	uint8_t totalPkt;
//	uint8_t newEst;
	neighbor_table_entry_t *ne;
	
	ne = &NeighborTable[idx];
	ne->flags |= MATURE_ENTRY;
	totalPkt = ne->rcvcnt + ne->failcnt;
//	newEst = (255 * (uint16_t)ne->rcvcnt) / totalPkt;
//	ne->inquality_sample = 100 * (uint16_t)newEst / 255;
	// inquality sample in the latest window
	ne->inquality_sample = (100 * (uint16_t)ne->rcvcnt) / totalPkt;
	// change to the alternative form; otherwise inequality cannot exceed 246 (pdr 96%)
	//ne->inquality = (ALPHA * ne->inquality + (10-ALPHA) * newEst)/10;
	if (ne->inquality != INVALID_INQUALITY) {
		ne->inquality = ne->inquality - (10 - ALPHA) * ne->inquality / 10 + (10 - ALPHA) * ne->inquality_sample / 10;
	} else {
		// first sample
		// if use 0%: slow rising
		// is use 100%: cause huge delta_i, reducing ER from maximal to empty, unstable
		ne->inquality = ne->inquality_sample * ALPHA / 10;
	}
	ne->inquality_version++;
	ne->rcvcnt = 0;
	ne->failcnt = 0;
	// need this bcoz some packets are skipped when sender is not using the latest ER computed by receiver
	ne->flags |= INIT_ENTRY;
}

// we received seq from the neighbor in idx
// update the last seen seq, receive and fail count
// refresh the age
void updateNeighborEntryIdx(uint8_t idx, uint8_t seq) {
	uint8_t packetGap;
//	neighbor_table_entry_t *localNeighborTable;
	neighbor_table_entry_t *ne;
	
//	localNeighborTable = NeighborTable;
	ne = &NeighborTable[idx];
	if (ne->flags & INIT_ENTRY) {
		dbg("LI", "Init entry update\n");
		ne->lastseq = seq;
		ne->flags &= ~INIT_ENTRY;
	}
	
	packetGap = seq - ne->lastseq;
	ne->lastseq = seq;
	ne->rcvcnt++;
	if (packetGap > 0) {
		ne->failcnt += packetGap - 1;
	}
	// XL
//	if (packetGap > MAX_PKT_GAP) {
//		ne->failcnt = 0;
//		ne->rcvcnt = 1;
//		ne->inquality = 0;
//	}
	
//	if (ne->rcvcnt >= BLQ_PKT_WINDOW) {
	if (((uint16_t)ne->rcvcnt + ne->failcnt) >= BLQ_PKT_WINDOW) {
		// check update controller bit
		//if (!(ne->flags & CONTROLLER_UPDATE_ENTRY)) {
			updateNeighborTableEst(idx);
			// set update controller bit
			//ne->flags |= CONTROLLER_UPDATE_ENTRY;
			// data pdr
			signal LinkEstimator.inLinkPdrUpdated(ne->ll_addr, FALSE);
			atomic call UartLog.logEntry(DBG_FLAG, DBG_RX_FLAG, ver_match_cnt, ver_mismatch_cnt);
		//}
	}
}

// initialize the neighbor table in the very beginning
void initNeighborTable() {
	uint8_t i;
	
	for (i = 0; i < NEIGHBOR_TABLE_SIZE; i++) {
		NeighborTable[i].flags = 0;
	}
}

// initialize the link estimator
command error_t Init.init() {
	dbg("LI", "Link estimator init\n");
	initNeighborTable();
	my_ll_addr = call SubAMPacket.address();
	return SUCCESS;
}

// ---------------------------------------------------------------------------------------------------
// interface LinkEstimator
// ---------------------------------------------------------------------------------------------------
async command neighbor_table_entry_t *LinkEstimator.getNeighborTable() {
	return NeighborTable;
}
// TODO: differentiate neighbors in and not in the table: currently both can be 0
// return FAIL if neighbor not found
// @param *inquality: PDR moving average till the last interval
// @param *inquality_sample: PDR sample in the current interval
// return inbound data pdr from the neighbor; scaled by 100 times, not 255 as in 4bitle
// treat unknown pdr as worst (0%)
async command error_t LinkEstimator.getInDataPdr(am_addr_t neighbor, uint8_t *inquality, uint8_t *inquality_sample, uint8_t *inquality_version) {
    uint8_t idx;
    neighbor_table_entry_t *ne;
	
	// valid for only existent and mature negihbors
	*inquality = 0;
	*inquality_sample = 0;

    idx = findIdx(neighbor);
    if (idx == INVALID_RVAL) {
      	return FAIL;
    } else {
    	ne = &NeighborTable[idx];
      	if (ne->flags & MATURE_ENTRY) {
			*inquality = ne->inquality;
			*inquality_sample = ne->inquality_sample;
			*inquality_version = ne->inquality_version;
      	} else {
      		return FAIL;
      	}
    }
    return SUCCESS;
}

// return inbound ACK pdr from the neighbor; scaled by 100 times, not 255 as in 4bitle
// treat unknown pdr as worst (0%)
async command error_t LinkEstimator.getInAckPdr(am_addr_t neighbor, uint8_t *ack_quality, uint8_t *ack_quality_sample) {
	uint8_t idx;
	uint16_t bi_pdr;
	neighbor_table_entry_t *ne;

	*ack_quality = 0;
	*ack_quality_sample = 0;
	
	idx = findIdx(neighbor);
	if (idx == INVALID_RVAL) {
		return FAIL;
	} else {
		ne = &NeighborTable[idx];
		if (ne->flags & EETX_MATURE_ENTRY) {
			// scale 100
			bi_pdr = 1000 / (ne->eetx + 10);
			// inbound pdr = bidirectional pdr / outbound pdr
			if (bi_pdr <= ne->outquality) {
				// avoid divided by 0 bcoz outquality remains uninitialized
				if (ne->outquality != 0)
					*ack_quality = (uint8_t)(100 * bi_pdr / ne->outquality);
			} else {
				*ack_quality = 100;
			}
			*ack_quality_sample = ne->ack_quality_sample;
		}
	}
	return SUCCESS;
}

// called when an acknowledgement is received; sign of a successful
// data transmission; to update forward link quality
async command error_t LinkEstimator.txAck(am_addr_t neighbor) {
	neighbor_table_entry_t *ne;
	uint8_t nidx = findIdx(neighbor);
	
	if (nidx == INVALID_RVAL) {
		return FAIL;
	}
	ne = &NeighborTable[nidx];
	ne->data_success++;
	ne->data_total++;
	if (ne->data_total >= DLQ_PKT_WINDOW) {
		updateDEETX(ne);
	}
	return SUCCESS;
}

// called when an acknowledgement is not received; could be due to
// data pkt or acknowledgement loss; to update forward link quality
async command error_t LinkEstimator.txNoAck(am_addr_t neighbor) {
	neighbor_table_entry_t *ne;
	uint8_t nidx = findIdx(neighbor);
	
	if (nidx == INVALID_RVAL) {
		return FAIL;
	}	
	ne = &NeighborTable[nidx];
	ne->data_total++;
	if (ne->data_total >= DLQ_PKT_WINDOW) {
		updateDEETX(ne);
	}
	return SUCCESS;
}

uint8_t findUnpinnedNeighborIdx() {
	uint8_t i;
	neighbor_table_entry_t *ne;
	
	for (i = 0; i < NEIGHBOR_TABLE_SIZE; i++) {
		ne = &NeighborTable[i];
		if (!(ne->flags & PINNED_ENTRY))
			return i;
	}
	return INVALID_RVAL;
}

//	insert the neighbor at any cost except all neighbors are "pinned"
//	a neighbor is "pinned" if it is called this command
//	if found
//		do nothing
//	else
//		if empty found
//			insert
//		else
//			if neighbor not "pinned" found
//				replace
async command error_t LinkEstimator.pinNeighbor(am_addr_t neighbor) {
	uint8_t nidx;
	neighbor_table_entry_t *ne;

	nidx = findIdx(neighbor);
	if (nidx != INVALID_RVAL)
		return SUCCESS;
	nidx = findEmptyNeighborIdx();
	if (nidx != INVALID_RVAL) {
		initNeighborIdx(nidx, neighbor);
	} else {
		nidx = findUnpinnedNeighborIdx();
		if (nidx != INVALID_RVAL) {
			ne = &NeighborTable[nidx];
			initNeighborIdx(nidx, neighbor);
			ne->flags |= PINNED_ENTRY;
			//call UartLog.logEntry(DBG_FLAG, DBG_ER_FLAG, 33, neighbor);
			return SUCCESS;
		}
	}
	return FAIL;
}


//------------------------------------------------------------------------------------------
// interface AMSend
//------------------------------------------------------------------------------------------
// add the link estimation header (seq no) and link estimation
// footer (neighbor entries) in the packet. Call just before sending the packet.
uint8_t addLinkEstHeaderAndFooter(am_addr_t addr, message_t *msg, uint8_t len) {
	uint8_t newlen;
	linkest_header_t * ONE hdr;
	linkest_footer_t * ONE footer;
	uint8_t i, j, k;
	uint8_t maxEntries, newPrevSentIdx;
	dbg("LI", "newlen1 = %d\n", len);
	hdr = getHeader(msg);
	footer = getFooter(msg, len);

	maxEntries = ((call SubPacket.maxPayloadLength() - len - sizeof(linkest_header_t)) / sizeof(linkest_footer_t));

	// Depending on the number of bits used to store the number
	// of entries, we can encode up to NUM_ENTRIES_FLAG using those bits
	if (maxEntries > NUM_ENTRIES_FLAG) {
		maxEntries = NUM_ENTRIES_FLAG;
	}
	
	// to shorten pkt len; this works bcoz each node only receives from 4 nodes, so round robin is fast enough
	// even though many can be in the neighbor table, only senders can become mature and be piggybacked
//	if (maxEntries > 1)
//		maxEntries = 1;
	
	dbg("LI", "Max payload is: %d, maxEntries is: %d\n", call SubPacket.maxPayloadLength(), maxEntries);
	
	j = 0;
	newPrevSentIdx = 0;
//#warning disable le footer
	for (i = 0; i < NEIGHBOR_TABLE_SIZE && j < maxEntries; i++) {
		k = (prevSentIdx + i + 1) % NEIGHBOR_TABLE_SIZE;
		if ((NeighborTable[k].flags & VALID_ENTRY) && (NeighborTable[k].flags & MATURE_ENTRY)) {
			footer[j].ll_addr = NeighborTable[k].ll_addr;
			footer[j].inquality = NeighborTable[k].inquality;
			footer[j].inquality_version = NeighborTable[k].inquality_version;
			newPrevSentIdx = k;
			j++;
		}
	}
	prevSentIdx = newPrevSentIdx;

	hdr->flags = 0;
	hdr->flags |= (NUM_ENTRIES_FLAG & j);
    hdr->seq = linkEstSeq++;
    hdr->rx_er_version = call Controller.getNbERVer(addr, TRUE);
	newlen = sizeof(linkest_header_t) + len + j * sizeof(linkest_footer_t);
	dbg("LI", "newlen2 = %d\n", newlen);
	dbg("LE", "%s: Sending seq: %d of length %d\n", __FUNCTION__, hdr->seq, newlen);
	return newlen;
}

// user of link estimator calls send here
// slap the header and footer before sending the message
async command error_t AMSend.send(am_addr_t addr, message_t* msg, uint8_t len) {
	uint8_t newlen;
//	uint32_t start_time = call LocalTime.get();
	newlen = addLinkEstHeaderAndFooter(addr, msg, len);
//	call UartLog.logEntry(DBG_FLAG, DBG_DELAY_FLAG, 10, call LocalTime.get() - start_time);
	dbg("LITest", "%s packet of length %hhu became %hhu\n", __FUNCTION__, len, newlen);
	return call SubSend.send(addr, msg, newlen);
}

// done sending the message that originated by
// the user of this component
async event void SubSend.sendDone(message_t* msg, error_t error ) {
	// debug pkt loss
//	linkest_header_t *hdr = getHeader(msg);
//	if (SUCCESS == error)
//		call UartLog.logEntry(DBG_FLAG, DBG_TX_FLAG, 0, hdr->seq);
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


//------------------------------------------------------------------------------------------
// interface Receive
//------------------------------------------------------------------------------------------
void updateNeighborOutQuality(uint8_t nidx, uint8_t outquality, uint8_t outquality_version) {
	neighbor_table_entry_t *localNeighborTable;
	
	localNeighborTable = NeighborTable;
	localNeighborTable[nidx].outquality = outquality;
	if (localNeighborTable[nidx].last_outquality_version != outquality_version) {
		// outbound data pdr updated (i.e., ack pdr update) only when outbound PDR also updates
		// may receive one version of outbound PDR multiple time, only update for the 1st time
	#ifdef TX_ER
		signal LinkEstimator.inLinkPdrUpdated(localNeighborTable[nidx].ll_addr, TRUE);
	#endif
		localNeighborTable[nidx].last_outquality_version = outquality_version;
	}
}

// called when link estimator generator packet or
// packets from upper layer that are wired to pass through
// link estimator is received
void updateNeighborInOutQuality(message_t *msg, uint8_t nidx, am_addr_t from, bool is_broadcast, uint8_t seq, bool is_for_me, uint8_t outquality, uint8_t outquality_version, uint8_t rx_er_version) {
	// only unicast, i.e., data packet, contributes to estimation
	if (!is_broadcast) {
		// only sample PDR and NI if the sender is using the latest rx ER, but still signal up even not
		//call UartLog.logEntry(DBG_FLAG, call Controller.getNbERVer(from, FALSE), __LINE__, rx_er_version);
		if (call Controller.getNbERVer(from, FALSE) == rx_er_version) {
			atomic ver_match_cnt++;
			updateNeighborEntryIdx(nidx, seq);
			// sample link NI
			call Controller.sampleNI(msg);
		} else {
			atomic ver_mismatch_cnt++;
		}
	}
	
	// outbound
	if (is_for_me) {
		//call UartLog.logTxRx(DBG_FLAG, DBG_CONTROLLER_FLAG, 10, 0, NeighborTable[nidx].ll_addr, seq, outquality, outquality_version, NeighborTable[nidx].last_outquality_version);
		updateNeighborOutQuality(nidx, outquality, outquality_version);
	}
}

// update outbound link quality to nb
// called from IMACController$updateNbLinkERTable bcoz outbound link quality is in control pkt
async command void LinkEstimator.updateNeighborOutQuality(am_addr_t nb, uint8_t outquality, uint8_t outquality_version) {
	uint8_t nidx;

	nidx = findIdx(nb);
	if (nidx != INVALID_RVAL) {
		updateNeighborOutQuality(nidx, outquality, outquality_version);
	}
}

void processReceivedMessage(message_t* ONE msg, void* COUNT_NOK(len) payload, uint8_t len) {
	uint8_t i, nidx;
	uint8_t num_entries;
	uint8_t outquality = 0;
	uint8_t outquality_version = 0;
	// contains outbound link quality from me
	bool is_for_me = FALSE;
	bool is_broadcast = (call SubAMPacket.destination(msg) == AM_BROADCAST_ADDR);
	//if (call SubAMPacket.destination(msg) == AM_BROADCAST_ADDR) {
	linkest_header_t* hdr = getHeader(msg);
	linkest_footer_t* footer = getFooter(msg, call Packet.payloadLength(msg));

	am_addr_t from;
//	call UartLog.logEntry(DBG_FLAG, DBG_RX_FLAG, 0, hdr->seq);
	//obtain number of neighbor entries in footer
	num_entries = hdr->flags & NUM_ENTRIES_FLAG;
	// see if the packet contains outbound gain for me
	for (i = 0; i < num_entries; i++) {
		//contains my outbound gain
		if (my_ll_addr == footer[i].ll_addr) {
			outquality = footer[i].inquality;
			outquality_version = footer[i].inquality_version;
			is_for_me = TRUE;
		}
	}
	from = call SubAMPacket.source(msg);
	dbg("LE", "%s: receiving seq: %d, %u %u\n", __FUNCTION__, hdr->seq, is_for_me, outquality);

	// FIFO
	// if found
	//   	update the entry
	// else
	//   	if an empty entry found
	//     		initialize the entry
	//   	else
	//      	we cannot accommodate this neighbor in the table
	nidx = findIdx(from);
	if (nidx != INVALID_RVAL) {
		updateNeighborInOutQuality(msg, nidx, from, is_broadcast, hdr->seq, is_for_me, outquality, outquality_version, hdr->rx_er_version);
	} else {
		nidx = findEmptyNeighborIdx();
		if (nidx != INVALID_RVAL) {
			dbg("LI", "Found an empty entry\n");
			initNeighborIdx(nidx, from);
			updateNeighborInOutQuality(msg, nidx, from, is_broadcast, hdr->seq, is_for_me, outquality, outquality_version, hdr->rx_er_version);
		}
	}
}

// new messages are received here
// update the neighbor table with the header
// and footer in the message
// then signal the user of this component
async event message_t* SubReceive.receive(message_t* msg, void* payload, uint8_t len) {
	processReceivedMessage(msg, payload, len);
	return signal Receive.receive(msg, call Packet.getPayload(msg, call Packet.payloadLength(msg)), call Packet.payloadLength(msg));
}

async event message_t* SubSnoop.receive(message_t* msg, void* payload, uint8_t len) {
	return signal Snoop.receive(msg, call Packet.getPayload(msg, call Packet.payloadLength(msg)), call Packet.payloadLength(msg));
}

//------------------------------------------------------------------------------------------
// interface Packet
//------------------------------------------------------------------------------------------
async command void Packet.clear(message_t* msg) {
	call SubPacket.clear(msg);
}

// subtract the space occupied by the link estimation header and footer from the incoming payload size
async command uint8_t Packet.payloadLength(message_t* msg) {
	linkest_header_t *hdr;
	hdr = getHeader(msg);
	return call SubPacket.payloadLength(msg) - sizeof(linkest_header_t) - sizeof(linkest_footer_t) * (NUM_ENTRIES_FLAG & hdr->flags);
}

// account for the space used by header and footer
// while setting the payload length
async command void Packet.setPayloadLength(message_t* msg, uint8_t len) {
	linkest_header_t *hdr;
	hdr = getHeader(msg);
	call SubPacket.setPayloadLength(msg, len + sizeof(linkest_header_t) + sizeof(linkest_footer_t) * (NUM_ENTRIES_FLAG & hdr->flags));
}

async command uint8_t Packet.maxPayloadLength() {
	return call SubPacket.maxPayloadLength() - sizeof(linkest_header_t);
}

// application payload pointer is just past the link estimation header
async command void* Packet.getPayload(message_t* msg, uint8_t len) {
	void* payload = call SubPacket.getPayload(msg, len + sizeof(linkest_header_t));
	if (payload != NULL) {
		payload += sizeof(linkest_header_t);
	}
	return payload;
}

default async event message_t *Receive.receive(message_t *m, void *payload, uint8_t len) {
	return m;
}
}
