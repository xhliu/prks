/* *
 * @ author: 	Xiaohui Liu (whulxh@gmail.com) 
 * @ updated: 	12/28/2011
 * @ updated: 	04/07/2012 03:38:54 PM
 *				integrate into iMAC
 *				04/08/2012 03:00:55 PM 
 *				add current noise + interference; sort signal map
 *				04/23/2012 
 *				add bidirectional power ctrl
 *				07/02/2012 TDMA
 * @ description: implement signal map, which offers signal gain from and to a neighbor
 *					also run controller
 */
#include "SignalMap.h"
#include "IMAC.h"

module SignalMapP {
	provides {
		interface AsyncAMSend as AMSend;
		interface AsyncReceive as Receive;
		interface AsyncPacket as Packet;
		
		interface SignalMap;
		interface Init;
	};
	
	uses {
		interface AsyncAMSend as SubSend;
		interface AsyncReceive as SubReceive;
		interface AsyncPacket as SubPacket;
		interface AsyncAMPacket as SubAMPacket;

		interface LinkEstimator;
		interface IMACController as Controller;
		
		interface CC2420Packet;
		interface RadioState;

		interface Util;
		interface UartLog;
		interface LocalTime<TMicro>;
		interface ForwarderInfo;
	};
}

implementation {
am_addr_t my_ll_addr;
// signal map
sm_entry_t signalMap[SM_SIZE];

// store neighbor signal map, between whom and me there is a link
nb_signal_map_entry_t nbSignalMap[NB_SIGNAL_MAP_SIZE];

// if there is not enough room in the packet to put all the signal map entries, in order to do round robin we need to remember which entry
// we sent in the last beacon
int16_t prevSentIdx = 0;
//uint16_t seqno;


bool freezed = FALSE;
// divide into 2 seperate files for readability
#include "SignalMapPUtils.nc"


// get the link estimation header in the packet
sm_header_t* getHeader(message_t* m) {
	return (sm_header_t*)call SubPacket.getPayload(m, sizeof(sm_header_t));
}

// get the signal map footer (neighbor entries) in the packet
// @param len: payload length to upper layer
sm_footer_t* getFooter(message_t* m, uint8_t len) {
	// To get a footer at offset "len", the payload must be len + sizeof large.
	return (sm_footer_t*)(len + (uint8_t *)call Packet.getPayload(m, len + sizeof(sm_footer_t)));
}

// initialize the link estimator
command error_t Init.init() {
	initSignalMap();
	initNbSignalMap();
	my_ll_addr = call SubAMPacket.address();
	return SUCCESS;
}


//-------------------------------------------------------------------------------------
// Interface SignalMap
//-------------------------------------------------------------------------------------
// only SUCCESS if both idx and in_gain are valid
// called from IMACController$initLinkERTable
async command error_t SignalMap.findNbIdxGain(am_addr_t nb, int16_t *idx_, int16_t *in_gain) {
	int16_t idx;
	sm_entry_t *se;
	
	idx = findIdx(nb);
	if (idx < SM_SIZE) {
		se = &signalMap[idx];
		if (INVALID_GAIN == se->inbound_gain)
			return FAIL;
		*idx_ = idx;
		*in_gain = se->inbound_gain;
		return SUCCESS;
	} else {
		return FAIL;
	}
}

async command error_t SignalMap.getLocalGain(am_addr_t nb, int16_t *in_gain, int16_t *out_gain) {
	int16_t idx;
	sm_entry_t *se;
	
	idx = findIdx(nb);
	if (idx < SM_SIZE) {
		se = &signalMap[idx];
		*in_gain = se->inbound_gain;
		*out_gain = se->outbound_gain;
		return SUCCESS;
	} else {
		return FAIL;
	}
}

// called from controller
async command sm_entry_t *SignalMap.getSignalMap() {
	return signalMap;
}

async command void SignalMap.printSignalMap(uint32_t time) {
	int16_t i;
//	sm_entry_t *se;
	
//	dbg("SM", "%s\n", __FUNCTION__);
	for (i = 0; i < SM_SIZE; i = i + 6) {
//		se = &signalMap[i];
//		if (se->valid) {
		dbg_clear("SM", "%u: <%u, %d>\n", i, se->nb, se->inbound_gain);
		//if (INVALID_GAIN == se->inbound_gain || INVALID_GAIN == se->outbound_gain)
//			call UartLog.logTxRx(DBG_FLAG, DBG_TDMA_FLAG, 10, 0, i, se->nb, se->inbound_gain, se->outbound_gain, time);
		call UartLog.logTxRx(DBG_FLAG, DBG_TDMA_FLAG, 
		signalMap[i].valid * ((signalMap[i].inbound_gain << 1) & 0xFF00) | signalMap[i].nb, 
		10,
		signalMap[i+1].valid * ((signalMap[i+1].inbound_gain << 1) & 0xFF00) | signalMap[i+1].nb, 
		signalMap[i+2].valid * ((signalMap[i+2].inbound_gain << 1) & 0xFF00) | signalMap[i+2].nb, 
		signalMap[i+3].valid * ((signalMap[i+3].inbound_gain << 1) & 0xFF00) | signalMap[i+3].nb, 
		signalMap[i+4].valid * ((signalMap[i+4].inbound_gain << 1) & 0xFF00) | signalMap[i+4].nb, 
		signalMap[i+5].valid * ((signalMap[i+5].inbound_gain << 1) & 0xFF00) | signalMap[i+5].nb);
	}
}

// inbound gain not to be updated, outbound and nb signal map can still be
async command void SignalMap.freeze() {
	freezed = TRUE;
}

//-------------------------------------------------------------------------------------
// Interface AMSend
//-------------------------------------------------------------------------------------
// slap the header and footer before sending the message
// add the header footer in the packet. Call just before sending the packet
uint8_t addLinkEstHeaderAndFooter(message_t *msg, uint8_t len);

async command error_t AMSend.send(am_addr_t addr, message_t* msg, uint8_t len) {
	uint8_t newlen;
	//uint32_t start_time = call LocalTime.get();
	newlen = addLinkEstHeaderAndFooter(msg, len);
   	//call UartLog.logEntry(DBG_FLAG, DBG_DELAY_FLAG, __LINE__, call LocalTime.get() - start_time);
	dbg("LI", "%s packet of length %hhu became %hhu\n", __FUNCTION__, len, newlen);
	if (newlen <= call SubPacket.maxPayloadLength()) {
		return call SubSend.send(addr, msg, newlen);
	} else {
		// careful not to exceed max payload length
		assert(newlen);
		return ESIZE;
	}
}

// add the header footer in the packet. Call iust before sending the packet
uint8_t addLinkEstHeaderAndFooter(message_t *msg, uint8_t len) {
	int16_t i, j, k;
	uint8_t maxEntries, newPrevSentIdx;
	uint8_t newlen;
	sm_header_t *hdr;
	sm_footer_t *footer;
	sm_footer_t *footer_p;
	sm_entry_t *se;

	hdr = getHeader(msg);
	footer = getFooter(msg, len);	
	maxEntries = (call SubPacket.maxPayloadLength() - len - sizeof(sm_header_t)) / sizeof(sm_footer_t);
	//call UartLog.logTxRx(DBG_FLAG, DBG_SM_FLAG, 256, 0, sizeof(sm_header_t), sizeof(sm_footer_t), call SubPacket.maxPayloadLength(), len, maxEntries);
	dbg("LI", "Max payload is: %d, maxEntries is: %d\n", call SubPacket.maxPayloadLength(), maxEntries);

	// add footer
	j = 0;
	newPrevSentIdx = 0;
//#warning diable SM footer
	for (i = 0; j < maxEntries && i < SM_SIZE; i++) {
	  	// signal map
	  	//k = (prevSentIdx + i + 1) % SM_SIZE;
	  	//k = (prevSentIdx + i + 1) & SM_SIZE_MODULAR;
	  	k = (prevSentIdx + i + 1);
	  	if (k >= SM_SIZE)
	  		k -= SM_SIZE;
	  	
	  	se = &signalMap[k];
	  	if (se->valid) {
	  		footer_p = &footer[j];
	  		footer_p->nb = se->nb;
	  		footer_p->inbound_gain = se->inbound_gain;
	  		footer_p->outbound_gain = se->outbound_gain;
	  		j++;
	  		newPrevSentIdx = k;
	  	}
	}
	prevSentIdx = newPrevSentIdx;
	
	// signal map is built using beacon, transmitted using maximal power
	// TODO: replace w/ actual level when power control is used
	hdr->power_level = CONTROL_POWER_LEVEL;
	hdr->footer_entry_cnts = j;
	//hdr->seqno = seqno++;
	hdr->data_tx_slot_ratio = call ForwarderInfo.getDataTxSlotRatio();
	newlen = sizeof(sm_header_t) + len + j * sizeof(sm_footer_t);
	return newlen;
}


// done sending the message that originated by the user of this component
async event void SubSend.sendDone(message_t* msg, error_t error ) {
//	sm_header_t *hdr = getHeader(msg);
//	call UartLog.logTxRx(DBG_FLAG, DBG_SM_FLAG, 1, 0, hdr->footer_entry_cnts, hdr->power_level, call Packet.payloadLength(msg), call SubPacket.payloadLength(msg), hdr->seqno);
	
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
// Interface SubReceive and SubSnoop
//-------------------------------------------------------------------------------------
// new messages are received here
// update the signal map with the header and footer in the message, then signal the user of this component
void processReceivedMessage(message_t *msg, void *payload, uint8_t len);

async event message_t* SubReceive.receive(message_t* msg, void* payload, uint8_t len) {
	//uint32_t start_time = call LocalTime.get();
	dbg("LI", "Received upper packet. Will signal up\n");
	processReceivedMessage(msg, payload, len);
	//call UartLog.logEntry(DBG_FLAG, DBG_EXEC_TIME_FLAG, 1, call LocalTime.get() - start_time);
	return signal Receive.receive(msg, call Packet.getPayload(msg, call Packet.payloadLength(msg)), call Packet.payloadLength(msg));
}

// called when signal map generated packet or packets from upper layer that are wired to pass through
// signal map is received
void processReceivedMessage(message_t *msg, void *payload, uint8_t len) {
	int16_t i;
	uint8_t tx_power_level;
	// signal strength right b4/after end of pkt reception
	int16_t pre_rss, post_rss;
	int16_t in_gain = INVALID_GAIN;
	int16_t out_gain = INVALID_GAIN;
	
	sm_header_t* hdr = getHeader(msg);
	sm_footer_t *footer = getFooter(msg, call Packet.payloadLength(msg));
	
	uint8_t footer_entry_cnts = hdr->footer_entry_cnts;
	am_addr_t nb = call SubAMPacket.source(msg);

//	call UartLog.logTxRx(DBG_FLAG, DBG_SM_FLAG, __LINE__, len, hdr->footer_entry_cnts, hdr->power_level, call Packet.payloadLength(msg), nb, hdr->seqno);
	// sample inbound gain from this packet
	tx_power_level = hdr->power_level;
	pre_rss = call CC2420Packet.getRssi(msg);
	post_rss = call CC2420Packet.getRssiIdle(msg);
	if (pre_rss > post_rss)
		// compute inbound gain
		in_gain = calcInboundGain(pre_rss, post_rss, tx_power_level);
	//call UartLog.logTxRx(DBG_FLAG, DBG_SM_FLAG, __LINE__, nb, call RadioState.getChannel(), -pre_rss, -post_rss, in_gain, in_gain / 128);
	
	// see if the packet contains outbound gain for me
	for (i = 0; i < footer_entry_cnts; i++) {
		//contains my outbound gain
		if (footer[i].nb == my_ll_addr) {
			out_gain = footer[i].inbound_gain;
		}
	}
	// update signal map
	updateSignalMap(nb, in_gain, out_gain, hdr->data_tx_slot_ratio);
	updateNbSignalMap(nb, footer, footer_entry_cnts);
	//call UartLog.logTxRx(DBG_FLAG, DBG_SM_FLAG, __LINE__, 0, 0, nb, pre_rss, post_rss, in_gain);
}
tasklet_async event void RadioState.done() {}
//-------------------------------------------------------------------------------------
// Interface Packet
//-------------------------------------------------------------------------------------
async command void Packet.clear(message_t* msg) {
	call SubPacket.clear(msg);
}

// subtract the space occupied by the signal map header and footer from the incoming payload size
async command uint8_t Packet.payloadLength(message_t* msg) {
	sm_header_t *hdr = getHeader(msg);
	return (call SubPacket.payloadLength(msg) - sizeof(sm_header_t) - sizeof(sm_footer_t) * hdr->footer_entry_cnts);
}

// account for the space used by header and footer while setting the payload length
async command void Packet.setPayloadLength(message_t* msg, uint8_t len) {
	sm_header_t *hdr = getHeader(msg);
	call SubPacket.setPayloadLength(msg, len + sizeof(sm_header_t) + sizeof(sm_footer_t) * hdr->footer_entry_cnts);
}

async command uint8_t Packet.maxPayloadLength() {
	return (call SubPacket.maxPayloadLength() - sizeof(sm_header_t));
}

// application payload pointer is just past the link estimation header
async command void* Packet.getPayload(message_t* msg, uint8_t len) {
	void* payload = call SubPacket.getPayload(msg, len + sizeof(sm_header_t));
	if (payload != NULL) {
		payload += sizeof(sm_header_t);
	}
	return payload;
}


default async event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len) {
	return msg;
}

// new inbound data/ack pdr arrives
async event error_t LinkEstimator.inLinkPdrUpdated(am_addr_t nb, bool is_ack) {	return SUCCESS;		}

}
