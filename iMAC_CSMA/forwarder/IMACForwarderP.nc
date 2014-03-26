/* *
 * @ author: Xiaohui Liu (whulxh@gmail.com) 
 * @ updated: 04/05/2012 08:37:26 PM 
 	04/09/2012: deal w/ receiving RTS from others while own RTS is in transit
 * @ description: data plane, i.e., handshake, of iMAC
 */
#include "IMACForwarder.h"

module IMACForwarderP {
	provides {
		interface Init;

		interface AMSend as Send;
		interface Receive;
		interface Packet;
		
		interface IMac;
	};
	
	uses {
		interface AMSend as SubSend;
		interface Receive as SubReceive;
		interface Receive as SubSnoop;
		interface Packet as SubPacket;
		interface AMPacket as SubAMPacket;
#ifndef TOSSIM
		interface RadioBackoff as Backoff;
		interface Random;
		// for power control
		interface CC2420Packet;
		interface Read<uint16_t> as ReadRssi;
#endif
		interface Timer<TMilli> as CtsTimeoutTimer;
		interface Timer<TMilli> as DataTimeoutTimer;
		
		interface LocalTime<TMilli>;		
		interface State;
		interface PacketAcknowledgements as Acks;
		interface LinkEstimator;
		interface SignalMap;
		interface UartLog;
	};
}

implementation {

uint16_t seqno = 0;

// # of RTS retries
uint8_t rts_retries;

am_addr_t my_ll_addr;

message_t m_data;
message_t m_rts;
message_t m_cts;

message_t *m_data_p;
message_t *m_rts_p;
message_t *m_cts_p;

// use these specific headers to avoid overriding generic header *hdr, e.g., in Receive.receive() event
imac_header_t *data_hdr;
imac_header_t *rts_hdr;
imac_header_t *cts_hdr;

// receiver address of the unicast data pkt being transmitted
am_addr_t m_data_addr;

// sender conversating w/ me
am_addr_t current_sender;

// neighbor info to respond later
am_addr_t pending_nb;
uint8_t pending_type;

// function forward declarations
error_t sendCTS(am_addr_t nb);
error_t sendData();

// debug
uint32_t rts_sent_timestamp, cts_sent_timestamp, cts_rx_timestamp, rts_tx_timestamp, cts_tx_timestamp, data_tx_timestamp;

/*
 * a node cannot tx btw [now .. nav]
 * NAV being conservative, assuming all pkts use max payload len; anyhow, thruput is not a concern now
 * the earliest time not to freeze
 * invariant: nav == maxCache();
 */
//uint32_t nav;
// store largest link NAVs
link_nav_entry_t linkNAVCache[LINK_NAV_CACHE_SIZE];
// cache management function prototypes
void initCache();
uint8_t findCacheIdx(am_addr_t sender, am_addr_t receiver);
uint8_t findEmptyCacheIdx();
uint8_t findMinCacheIdx();
uint32_t maxCache();
void printCache();
void insertCache(am_addr_t sender, am_addr_t receiver, uint32_t link_nav);
uint32_t deleteCache(am_addr_t sender, am_addr_t receiver);
uint8_t cacheSize();

// including ER and power ctrl
bool is_iMac_enabled;

command void IMac.enable() {
	is_iMac_enabled = TRUE;
}

command void IMac.disable() {
	is_iMac_enabled = FALSE;
}

command bool IMac.isEnabled() {
	return is_iMac_enabled;
}
/*
 * virtual carrier sensing
 * return TRUE if CCA is busy reflected by NAV
 * called from CC2420TransmitP$BackoffTimer$fired
 */
async command bool IMac.virtualCca() {
	// only read cache, no writing, so no racing condition
	return (call LocalTime.get() <= maxCache());
}

// get the link estimation header in the packet
imac_header_t* getHeader(message_t* m) {
	return (imac_header_t*)call SubPacket.getPayload(m, sizeof(imac_header_t));
}

command error_t Init.init() {
	rts_retries = 0;
	my_ll_addr 	= call SubAMPacket.address();
	// nav = 0;
	initCache();
	// ER and power ctrl disabled by default
	is_iMac_enabled = FALSE;
	call State.toIdle();
	// no buffer needed to store data pkt being transmitted; just to be uniform
	m_data_p = &m_data;
	data_hdr = getHeader(m_data_p);
	m_rts_p	= &m_rts;
	rts_hdr = getHeader(m_rts_p);
	m_cts_p	= &m_cts;
	cts_hdr = getHeader(m_cts_p);
	// no neighbor pending to be responded initially
	pending_type = TYPE_INVALID;
	return SUCCESS;
}


/* *
 * * Interface Send
 * */
command error_t Send.send(am_addr_t addr, message_t* msg, uint8_t len) {
	error_t ret;
	int16_t interference_threshold, min_interference_threshold, node_i;
	uint8_t power_level;
	
	if (len > call Send.maxPayloadLength())		{	return ESIZE;	}
	if (!call State.isIdle())					{	return EBUSY;	}
	// not accept broadcast now for simplicity
	if (AM_BROADCAST_ADDR == addr) 				{	return FAIL;	}
	
	// "store" data pkt being transmitted
	m_data_addr = addr;
	atomic m_data_p = msg;
	call Packet.setPayloadLength(msg, len);
	
	// prepare RTS; backoff is not here, it's in interface Backoff
	rts_retries = 0;
	rts_hdr->type = TYPE_RTS;
	rts_hdr->dst = addr;
// basic RTS/CTS mode
if (is_iMac_enabled) {
	// load ER
	ret = call SignalMap.getInterferenceThresholdPowerLevelNI(addr, TRUE, &interference_threshold, &min_interference_threshold, &power_level, &node_i);
	if (ret != SUCCESS) {
		dbg("forwarder", "%s: send fail 0\n", __FUNCTION__);
		//call UartLog.logEntry(DBG_FLAG, DBG_ER_FLAG, 10, addr);
		return ret;
	}
	//call UartLog.logTxRx(DBG_FLAG, DBG_ER_FLAG, 13, 0, 0, ret, addr, interference_threshold, power_level);
	rts_hdr->interference_threshold = interference_threshold;
	rts_hdr->min_interference_threshold = min_interference_threshold;
	rts_hdr->node_i = node_i;
	rts_hdr->seqno = seqno++;
#ifndef TOSSIM
	// set power
	call CC2420Packet.setPower(m_rts_p, power_level);
#endif
}
	// NAV in ms, including CTS, data and ack tx time
	rts_hdr->nav = RTS_NAV;
	
	// send RTS
	ret = call SubSend.send(AM_BROADCAST_ADDR, m_rts_p, sizeof(imac_header_t));
	if (SUCCESS == ret) {
		call State.forceState(S_SENDING_RTS);
		rts_tx_timestamp = call LocalTime.get();
		dbg("iMAC", "%s: S_SENDING_RTS\n", __FUNCTION__);
	} else {
		dbg("forwarder", "%s: send fail 1\n", __FUNCTION__);
		//call UartLog.logEntry(DBG_FLAG, DBG_ER_FLAG, 11, ret);
	}
	return ret;
}

#ifndef TOSSIM
am_addr_t pending_rss_nb;
event void ReadRssi.readDone(error_t result, uint16_t val) {
	int16_t ni;
	
	// sample noise plus interference when it is valid
	if (SUCCESS == result) {
		ni = (int16_t)val - 172;
		call SignalMap.updateNI(TRUE, pending_rss_nb, FALSE, ni);
	}
}
#endif

event void SubSend.sendDone(message_t* msg, error_t error ) {
	error_t ret = SUCCESS;
	imac_header_t *hdr = getHeader(msg);
	if (SUCCESS == error) {
		//imac_header_t *hdr = getHeader(msg);
		call UartLog.logTxRx(DBG_FLAG, DBG_TX_FLAG, rts_retries, seqno, hdr->seqno, call SubAMPacket.destination(msg), hdr->type, hdr->dst, call State.getState());
//		call UartLog.logTxRx(DBG_FLAG, DBG_TX_FLAG, 0, 0, call SubAMPacket.destination(msg), hdr->type, hdr->dst, hdr->seqno, hdr->node_i);
	}
//	if (S_SENDING_RTS == call State.getState())
//		call UartLog.logTxRx(DBG_FLAG, DBG_DELAY_FLAG, 0, 0, 0, 0, call State.getState(), error, call LocalTime.get() - rts_tx_timestamp);
//	if (S_SENDING_CTS == call State.getState())
//		call UartLog.logTxRx(DBG_FLAG, DBG_DELAY_FLAG, 0, 0, 0, 0, call State.getState(), error, call LocalTime.get() - cts_tx_timestamp);
//	if (S_SENDING_DATA == call State.getState())
//		call UartLog.logTxRx(DBG_FLAG, DBG_DELAY_FLAG, 0, 0, 0, 0, call State.getState(), error, call LocalTime.get() - data_tx_timestamp);
	
	switch (call State.getState()) {
		case S_SENDING_RTS:
			// assert(msg == m_rts_p)
			if (SUCCESS == error) {
				call State.forceState(S_EXPECTING_CTS);
				dbg("iMAC", "%s: S_EXPECTING_CTS\n", __FUNCTION__);
				call CtsTimeoutTimer.startOneShot(CTS_TIMEOUT);
				rts_sent_timestamp = call LocalTime.get();
			} else {
				call State.forceState(S_IDLE);
				dbg("iMAC", "%s: S_IDLE\n", __FUNCTION__);
				signal Send.sendDone(m_data_p, error);
				//call UartLog.logEntry(DBG_FLAG, DBG_TX_FAIL_FLAG, 0, error);
			}
			break;
		
		case S_SENDING_CTS:
			// assert(msg == m_cts_p)
			if (SUCCESS == error) {
#ifndef TOSSIM
				// sample NI
				pending_rss_nb = hdr->dst;
				call ReadRssi.read();
#endif
				call State.forceState(S_EXPECTING_DATA);
				dbg("iMAC", "%s: S_EXPECTING_DATA\n", __FUNCTION__);
				call DataTimeoutTimer.startOneShot(DATA_TIMEOUT);
				cts_sent_timestamp = call LocalTime.get();
			} else {
				// one way to reach here is CTS is dropped after max # of CSMA attempts
				call State.forceState(S_IDLE);
				dbg("iMAC", "%s: S_IDLE\n", __FUNCTION__);
			}
			break;
			
		case S_SENDING_DATA:
			// assert(msg == m_data_p);
			// no retx of data
			if (SUCCESS == error) {
				call State.forceState(S_IDLE);
				dbg("iMAC", "%s: S_IDLE\n", __FUNCTION__);
				// data driven
				if (call Acks.wasAcked(msg)) {
					call LinkEstimator.txAck(m_data_addr);
					//call UartLog.logTxRx(DBG_FLAG, DBG_TIMEOUT_FLAG, 2, 0, 0, 0, 0, m_data_addr, call LocalTime.get() - rts_sent_timestamp);
					//call UartLog.logTxRx(DBG_FLAG, DBG_TIMEOUT_FLAG, 3, 0, 0, 0, 0, m_data_addr, call LocalTime.get() - cts_rx_timestamp);
				} else {
					call LinkEstimator.txNoAck(m_data_addr);
				}
			} else {
				call State.forceState(S_IDLE);
				dbg("iMAC", "%s: S_IDLE\n", __FUNCTION__);
				//call UartLog.logEntry(DBG_FLAG, DBG_TX_FAIL_FLAG, 1, error);
			}
			signal Send.sendDone(m_data_p, error);
			break;
		
		case S_CANCELLING_RTS:
			// reset to S_IDLE first after cancellation; otherwise may get stuck if ensuing operations fail
			call State.forceState(S_IDLE);
			if (TYPE_RTS == pending_type) {
				// hear RTS to me; reply CTS
				ret = sendCTS(pending_nb);
				signal Send.sendDone(m_data_p, ECANCEL);
			} else if (TYPE_CTS == pending_type) {
				// hear late CTS to me; reply DATA
				if (pending_nb == m_data_addr)
					ret = sendData();
				else
					call UartLog.logTxRx(DBG_FLAG, DBG_CANCEL_FLAG, 3, 0, 0, m_data_addr, ret, pending_type, pending_nb);
			}
			//call UartLog.logTxRx(DBG_FLAG, DBG_CANCEL_FLAG, 1, 0, 0, m_data_addr, ret, pending_type, pending_nb);
			// no neighbor expecting response
			pending_type = TYPE_INVALID;
			break;
		
		case S_CANCELLING_CTS:
			// reset to S_IDLE first after cancellation; otherwise may get stuck if ensuing operations fail		
			call State.forceState(S_IDLE);
			if (TYPE_RTS == pending_type) {
				ret = sendCTS(pending_nb);
				//call UartLog.logTxRx(DBG_FLAG, DBG_CANCEL_FLAG, 2, 0, 0, m_data_addr, ret, pending_type, pending_nb);
				pending_type = TYPE_INVALID;
			}
			break;
		
		default:
			// can reach here if pending RTS is cancelled in backoff by other RTS/CTS not destined to me
			break;
	}
}

// does not hear CTS back
event void CtsTimeoutTimer.fired() {
	error_t ret;
	// assert(state == S_EXPECTING_CTS);
	if (rts_retries++ < RTS_MAX_RETRIES) {
		// retx RTS
		ret = call SubSend.send(AM_BROADCAST_ADDR, m_rts_p, sizeof(imac_header_t));
		dbg("iMAC", "%s: retx RTS\n", __FUNCTION__);
		if (ret != SUCCESS) {
			call State.forceState(S_IDLE);
			dbg("iMAC", "%s: S_IDLE\n", __FUNCTION__);
			signal Send.sendDone(m_data_p, ret);
			//call UartLog.logEntry(DBG_FLAG, DBG_TX_FAIL_FLAG, 2, ret);
		} else {
			rts_tx_timestamp = call LocalTime.get();
			call State.forceState(S_SENDING_RTS);
			dbg("iMAC", "%s: S_SENDING_RTS\n", __FUNCTION__);
		}
	} else {
		call State.forceState(S_IDLE);
		dbg("iMAC", "%s: S_IDLE\n", __FUNCTION__);
		signal Send.sendDone(m_data_p, FAIL);
		//call UartLog.logEntry(DBG_FLAG, DBG_TX_FAIL_FLAG, 3, m_data_addr);
	}
}

/*
 * does not hear data; BUT does not retx CTS bcoz there are 2 causes
 * 1) CTS lost: sender can detect this and retx RTS so receiver can just wait
 * 2) CTS succeeds but data lost: even retx CTS, sender will not retx data, not helpful
 *
 * invariant: for every RTS received, there is at most one CTS response
 */
event void DataTimeoutTimer.fired() {
	call State.forceState(S_IDLE);
	dbg("iMAC", "%s: S_IDLE\n", __FUNCTION__);
}

uint8_t backoff_type;
task void logBackoffTask() {
	uint8_t backoff_type_;
	atomic backoff_type_ = backoff_type;
	//call UartLog.logEntry(DBG_FLAG, DBG_DELAY_FLAG, 22, call State.getState());
}
/*
 * only RTS and CTS cca and backoffs
 * underlying radio stack only backoffs when CCA is enabled; so disable CCA is enough here
 * TODO: no exponential backoff and limit of backoff times
 */
#ifndef TOSSIM
uint8_t backoff_cnt;
async event void Backoff.requestCca(message_t *msg) {
	message_t *m_data_p_;
	atomic m_data_p_ = m_data_p;
	//if (call State.isState(S_SENDING_CTS) || call State.isState(S_SENDING_DATA)) {
	if (msg == m_cts_p || msg == m_data_p_) {
		call Backoff.setCca(FALSE);
		post logBackoffTask();
	}
}

async event void Backoff.requestInitialBackoff(message_t *msg) {
	// avoid % over 0
	uint16_t window_size = 1;
	// assert(call State.isState(S_SENDING_RTS) || call State.isState(S_SENDING_CTS))
	if (call State.isState(S_SENDING_RTS)) {
		window_size = 0x1F;
	} else if (call State.isState(S_SENDING_CTS)) {
		window_size = 0x0F;
		atomic backoff_cnt = 0;
	}
	call Backoff.setInitialBackoff(call Random.rand16() % (window_size * CC2420_BACKOFF_PERIOD) + CC2420_MIN_BACKOFF);
}

message_t *cancelling_msg;
task void cancelTask() {
	error_t ret;
	message_t *cancelling_msg_;
	atomic cancelling_msg_ = cancelling_msg;
	// no need to reset state if returns SUCCESS, sendDone(ECANCEL) will come back later, resetting state to S_IDLE
	ret = call SubSend.cancel(cancelling_msg_);
	if (SUCCESS == ret) {
		call State.forceState(S_CANCELLING_CTS);
	}
	//call UartLog.logTxRx(DBG_FLAG, DBG_CANCEL_FLAG, 0, 1, 0, m_data_addr, ret, pending_type, pending_nb);
}

async event void Backoff.requestCongestionBackoff(message_t *msg) {
	// avoid % over 0
	uint16_t window_size = 1;
	uint8_t backoff_cnt_;
	// assert(call State.isState(S_SENDING_RTS) || call State.isState(S_SENDING_CTS))
	if (call State.isState(S_SENDING_RTS)) {
		window_size = 0x7;
	} else if (call State.isState(S_SENDING_CTS)) {
		window_size = 0x3;
		// drop after max # of backoff attempts
		atomic backoff_cnt_ = backoff_cnt++;
		if (backoff_cnt_ >= CTS_MAX_BACKOFF_RETRIES) {
			atomic cancelling_msg = msg;
			// cancel CTS
			post cancelTask();
		}
	}
	call Backoff.setCongestionBackoff(call Random.rand16() % (window_size * CC2420_BACKOFF_PERIOD) + CC2420_MIN_BACKOFF);
}
#endif
  
// cascade the calls down
command uint8_t Send.cancel(message_t* msg) {
	return call SubSend.cancel(msg);
}

command uint8_t Send.maxPayloadLength() {
	return call Packet.maxPayloadLength();
}

command void* Send.getPayload(message_t* msg, uint8_t len) {
	return call Packet.getPayload(msg, len);
}

// update NAV if in ER
void updateNAV(am_addr_t nb, int16_t interference_threshold, uint32_t hdr_nav, am_addr_t sender, am_addr_t receiver) {
	uint32_t now;
	bool is_in_er = call SignalMap.inExRegion(nb, interference_threshold);
	// in basic RTS/CTS mode, whoever receives RTS/CTS is in exclusion region
	if ((is_iMac_enabled && is_in_er) || !is_iMac_enabled) {
		now = call LocalTime.get();
		insertCache(sender, receiver, now + hdr_nav);
	}
//	if (call Random.rand16() % 10 == 0)
//		call UartLog.logTxRx(DBG_FLAG, DBG_ER_FLAG, 15, nb, sender, receiver, is_in_er, interference_threshold, maxCache());
}

error_t sendCTS(am_addr_t nb) {
	error_t ret;
	int16_t interference_threshold, min_interference_threshold, node_i;
	uint8_t power_level;
	
	// remember the sender I'm talking to
	current_sender = nb;
	
	// prepare CTS
	cts_hdr->type = TYPE_CTS;
	cts_hdr->dst = nb;
// not in basic RTS/CTS mode
if (is_iMac_enabled) {
	// load ER
	ret = call SignalMap.getInterferenceThresholdPowerLevelNI(nb, FALSE, &interference_threshold, &min_interference_threshold, &power_level, &node_i);
	if (ret != SUCCESS) {
		//call UartLog.logEntry(DBG_FLAG, DBG_ER_FLAG, 12, nb);
		return ret;
	}
	//call UartLog.logTxRx(DBG_FLAG, DBG_ER_FLAG, 14, 0, 0, ret, nb, interference_threshold, power_level);
	cts_hdr->interference_threshold = interference_threshold;
	cts_hdr->min_interference_threshold = min_interference_threshold;
	cts_hdr->node_i = node_i;
	cts_hdr->seqno = seqno++;
#ifndef TOSSIM
	// set power
	call CC2420Packet.setPower(m_cts_p, power_level);
#endif
}
	// include data and ack
	cts_hdr->nav = CTS_NAV;
	
	// send CTS
	ret = call SubSend.send(AM_BROADCAST_ADDR, m_cts_p, sizeof(imac_header_t));
	if (SUCCESS == ret) {
		cts_tx_timestamp = call LocalTime.get();
		call State.forceState(S_SENDING_CTS);
		dbg("iMAC", "%s: S_SENDING_CTS\n", __FUNCTION__);
	}
	return ret;
}

error_t sendData() {
	error_t ret;
	
	// prepare data
	data_hdr = getHeader(m_data_p);
	data_hdr->type = TYPE_DATA;
	// necessary for snoop to know receiver of this DATA pkt
	data_hdr->dst = m_data_addr;
	data_hdr->seqno = seqno++;
	ret = call Acks.requestAck(m_data_p);
	// assert(ret != SUCCESS);
	
	// unicast data
	ret = call SubSend.send(m_data_addr, m_data_p, call SubPacket.payloadLength(m_data_p));
	if (SUCCESS == ret) {
		data_tx_timestamp = call LocalTime.get();
		call State.forceState(S_SENDING_DATA);
		dbg("iMAC", "%s: S_SENDING_DATA\n", __FUNCTION__);
	} else {
		call State.forceState(S_IDLE);
		dbg("iMAC", "%s: S_IDLE\n", __FUNCTION__);
		signal Send.sendDone(m_data_p, ret);
		//call UartLog.logEntry(DBG_FLAG, DBG_TX_FAIL_FLAG, 4, ret);
	}
	return ret;
}

// overhear completion of a transaction, i.e., DATA
event message_t* SubSnoop.receive(message_t* msg, void* payload, uint8_t len) {
	imac_header_t* hdr = getHeader(msg);
	am_addr_t nb = call SubAMPacket.source(msg);
	
	if (TYPE_DATA == hdr->type) {
		//uint32_t link_nav, now, diff;
		//now = call LocalTime.get();
		// disable this link's nav on me
		deleteCache(nb, hdr->dst);
		//link_nav = deleteCache(nb, hdr->dst);
		//diff = (link_nav > now) ? (link_nav - now) : (now - link_nav);
		//call UartLog.logTxRx(DBG_FLAG, DBG_COUNTER_NAV_FLAG, 0, 0, 0, nb, hdr->dst, link_nav > now, diff);
	}
	return msg;
}

void sampleNI(message_t *msg, am_addr_t nb, bool is_ack) {
	uint16_t val;
	int16_t post_rss;
#ifndef TOSSIM
	// sample NI
	val = call CC2420Packet.getRssiIdle(msg);
	if (val != INVALID_RSSI) {
#else
	val = 0;
	if (TRUE) {
#endif	
		// - 127 - 45
		post_rss = (int16_t)val - 172;
		call SignalMap.updateNI(TRUE, nb, is_ack, post_rss);
		//if (post_rss > 0)
			//call UartLog.logTxRx(DBG_FLAG, DBG_ER_FLAG, 23, 0, 0, 0, 0, post_rss, val);
	}
}

// new messages are received here
event message_t* SubReceive.receive(message_t* msg, void* payload, uint8_t len) {
	error_t ret;
//	uint16_t val;
//	int16_t post_rss;
	
	imac_header_t* hdr = getHeader(msg);
	am_addr_t nb = call SubAMPacket.source(msg);
	
	//if (hdr->dst == my_ll_addr) {
	call UartLog.logTxRx(DBG_FLAG, DBG_RX_FLAG, 0, call SignalMap.inExRegion(nb, hdr->interference_threshold), hdr->interference_threshold, nb, hdr->type, hdr->dst, call State.getState());
//		if (TYPE_CTS == hdr->type)
//			call UartLog.logTxRx(DBG_FLAG, DBG_TIMEOUT_FLAG, 0, 1, 0, 0, 0, nb, call LocalTime.get() - rts_sent_timestamp);
//		if (TYPE_DATA == hdr->type)
//			call UartLog.logTxRx(DBG_FLAG, DBG_TIMEOUT_FLAG, 1, 1, 0, 0, 0, nb, call LocalTime.get() - cts_sent_timestamp);
	//}
	dbg("iMAC", "%s: Receive pkt from %hu\n", __FUNCTION__, nb);
	
	// update outbound ER info
	if (TYPE_RTS == hdr->type || TYPE_CTS == hdr->type) {
		//int16_t node_i = hdr->node_i;
		call SignalMap.updateOutboundER(nb, hdr->min_interference_threshold, hdr->node_i, hdr->type, hdr->seqno);
		//if (hdr->node_i < -100 || hdr->node_i > 0)
			//call UartLog.logTxRx(DBG_FLAG, DBG_ER_FLAG, 21, node_i == hdr->node_i, node_i, nb, hdr->type, hdr->seqno, hdr->node_i);
	}
//#ifndef TOSSIM
//	// sample NI
//	val = call CC2420Packet.getRssiIdle(msg);
//	if (val != INVALID_RSSI) {
//#else
//	val = 0;
//	if (TRUE) {
//#endif	
//		// - 127 - 45
//		post_rss = (int16_t)val - 172;
//		call SignalMap.updateNI(post_rss, call State.getState());
//		//if (post_rss > 0)
//			//call UartLog.logTxRx(DBG_FLAG, DBG_ER_FLAG, 23, 0, 0, 0, 0, post_rss, val);
//	}

	// overhear RTS/CTS; update NAV regardless of current state
	if (hdr->dst != my_ll_addr && (TYPE_RTS == hdr->type || TYPE_CTS == hdr->type)) {
		// nb is sender
		if (TYPE_RTS == hdr->type)
			updateNAV(nb, hdr->interference_threshold, hdr->nav, nb, hdr->dst);
		// nb is receiver
		if (TYPE_CTS == hdr->type)	
			updateNAV(nb, hdr->interference_threshold, hdr->nav, hdr->dst, nb);
	}
	// pin neighbor communicating w/ me to ensure they are in my link estimation neighbor table
	if (hdr->dst == my_ll_addr)
		call LinkEstimator.pinNeighbor(nb);
		
	switch (call State.getState()) {
		case S_IDLE:
			// hear RTS to me; reply CTS
			if (hdr->dst == my_ll_addr && TYPE_RTS == hdr->type) {
				sendCTS(nb);
			}
			break;
			
		case S_SENDING_RTS:
			if (hdr->dst == my_ll_addr) {
				// assert(hdr->type != TYPE_DATA)
				// cancel my own before reply; otherwise reply will fail
				ret = call SubSend.cancel(m_rts_p);
				//call UartLog.logTxRx(DBG_FLAG, DBG_CANCEL_FLAG, 0, 0, 0, hdr->type, ret, nb, m_data_addr);
				if (SUCCESS == ret) {
					call State.forceState(S_CANCELLING_RTS);
					// remember the neighbor info to send to it after sendDone() comes back
					pending_type = hdr->type;
					pending_nb = nb;
				}
				// do not change if packet cannot be cancelled
			}
			break;
			
		case S_EXPECTING_CTS:
			// hear CTS to me; reply data
			if (hdr->dst == my_ll_addr && TYPE_CTS == hdr->type) {
				// sample NI
				sampleNI(msg, nb, TRUE);
				cts_rx_timestamp = call LocalTime.get();
				//call UartLog.logTxRx(DBG_FLAG, DBG_TIMEOUT_FLAG, 0, 0, 0, 0, 0, nb, call LocalTime.get() - rts_sent_timestamp);
				// assert(m_data_addr == nb);
				// cancel RTS retx
				call CtsTimeoutTimer.stop();
				// reply data
				sendData();
			}
			break;
			
		case S_EXPECTING_DATA:
			// hear data; unicast, no need to check (hdr->dst == my_ll_addr)
			if (TYPE_DATA == hdr->type) {
				// sample NI
				sampleNI(msg, nb, FALSE);
				//call UartLog.logTxRx(DBG_FLAG, DBG_TIMEOUT_FLAG, 1, 0, 0, 0, 0, nb, call LocalTime.get() - cts_sent_timestamp);
				// assert(hdr->dst == my_ll_addr);
				// cancel data timeout timer
				call DataTimeoutTimer.stop();
				call State.forceState(S_IDLE);
				dbg("iMAC", "%s: S_IDLE\n", __FUNCTION__);
				// signal up
				return signal Receive.receive(msg, call Packet.getPayload(msg, call Packet.payloadLength(msg)), call Packet.payloadLength(msg));
			} else if (hdr->dst == my_ll_addr && TYPE_RTS == hdr->type) {
				// only reply if from the previous sender, this happens when CTS is lost and sender retx RTS; otherwise, ignore
				if (nb == current_sender)
					sendCTS(nb);
			}
			break;
		
		case S_CANCELLING_RTS:
			if (hdr->dst == my_ll_addr && TYPE_CTS == hdr->type) {
				// remember the neighbor info to send to it after sendDone() comes back
				pending_type = hdr->type;
				pending_nb = nb;
			}
			break;
		
		case S_CANCELLING_CTS:
			if (hdr->dst == my_ll_addr && TYPE_RTS == hdr->type) {
				// remember the neighbor info to send to it after sendDone() comes back
				pending_type = hdr->type;
				pending_nb = nb;			
			}
			break;
				
		default:
			break;
	}
	return msg;
}


/* *
 * Interface Packet
 */
command void Packet.clear(message_t* msg) {
	call SubPacket.clear(msg);
}

// subtract the space occupied by the signal map header and footer from the incoming payload size
command uint8_t Packet.payloadLength(message_t* msg) {
	return (call SubPacket.payloadLength(msg) - sizeof(imac_header_t));
}

// account for the space used by header and footer while setting the payload length
command void Packet.setPayloadLength(message_t* msg, uint8_t len) {
	call SubPacket.setPayloadLength(msg, len + sizeof(imac_header_t));
}

command uint8_t Packet.maxPayloadLength() {
	return (call SubPacket.maxPayloadLength() - sizeof(imac_header_t));
}

// application payload pointer is just past the link estimation header
command void* Packet.getPayload(message_t* msg, uint8_t len) {
	void* payload = call SubPacket.getPayload(msg, len + sizeof(imac_header_t));
	if (payload != NULL) {
		payload += sizeof(imac_header_t);
	}
	return payload;
}

default event void Send.sendDone(message_t* msg, error_t error ) {}
default event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len) {
	return msg;
}
// new inbound data/ack pdr arrives
event error_t LinkEstimator.inLinkPdrUpdated(am_addr_t nb, bool is_ack) {	return SUCCESS;		}

#include "IMACForwarderPUtils.nc"
}
