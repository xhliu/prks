/* *
 * @ author: Xiaohui Liu (whulxh@gmail.com) 
 * @ updated: 04/05/2012 08:37:26 PM 
 	04/09/2012: deal w/ receiving RTS from others while own RTS is in transit
 	08/15/2013: fix CMAC bugs
 	0) deadlock caused by passConcurrencyCheck()
 	1) remove conservativeness in passConcurrencyCheck()
 	2) EALREADY caused by unfinished beaconing
 	3) disable NAV
 * @ description: data plane, i.e., handshake, of iMAC
 */
#include "IMACForwarder.h"
#include "SignalMap.h"

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
		
		interface Queue<message_t*>;
		interface Pool<message_t>;
		
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

//message_t *m_data_p;
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
task void sendRTSTask();
error_t sendCTS(am_addr_t nb);
task void sendDataTask();
void sendDone(error_t error);
task void sendDoneTask();

// debug
uint32_t rts_sent_timestamp, cts_sent_timestamp, cts_rx_timestamp, rts_tx_timestamp, cts_tx_timestamp, data_tx_timestamp;

// local NI
int16_t node_ni;
// NI of the receiver at the sender: at most 1 receiver, so just 1 variable
int16_t receiver_ni;
int16_t receiver_signal;
// link table store link's SINR
link_table_entry_t linkTable[LINK_TABLE_SIZE];
uint8_t block_idx;

/*
 * a node cannot tx btw [now .. nav]
 * NAV being conservative, assuming all pkts use max payload len; anyhow, thruput is not a concern now
 * the earliest time not to freeze
 * invariant: nav == maxCache();
 */
//uint32_t nav;
// store largest link NAVs
link_nav_entry_t linkNAVCache[LINK_NAV_CACHE_SIZE];
#include "IMACForwarderPUtils.nc"

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
	// CMAC: disable virtual CCA
	return FALSE;
	// only read cache, no writing, so no racing condition
	//return (call LocalTime.get() <= maxCache());
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
//	// no buffer needed to store data pkt being transmitted; just to be uniform
//	m_data_p = &m_data;
//	data_hdr = getHeader(m_data_p);
	m_rts_p	= &m_rts;
	rts_hdr = getHeader(m_rts_p);
	m_cts_p	= &m_cts;
	cts_hdr = getHeader(m_cts_p);
	// no neighbor pending to be responded initially
	pending_type = TYPE_INVALID;
	
	node_ni = INVALID_DBM;
	receiver_ni = INVALID_DBM;
	receiver_signal = INVALID_DBM;
	initLinkTable();
	return SUCCESS;
}


/* *
 * * Interface Send
 * */
command error_t Send.send(am_addr_t addr, message_t* msg, uint8_t len) {
	message_t *m;
//	int16_t interference_threshold, min_interference_threshold, node_i;
//	uint8_t power_level;
	//call UartLog.logEntry(DBG_FLAG, len, __LINE__, call Send.maxPayloadLength());
	if (len > call Send.maxPayloadLength())		{	return ESIZE;	}
	// this can occur when the node is also receiving; both sender and receiver
	if (!call State.isIdle())					{	return EBUSY;	}
	// not accept broadcast now for simplicity
	if (AM_BROADCAST_ADDR == addr) 				{	return FAIL;	}
	// C-MAC: if check here deadlock bcoz receiver_ni never gets updated and sinr at receiver will keep failing
	//if (!passConcurrencyCheck(addr))			{	return ERETRY;	}

	// "store" data pkt being transmitted
	m_data_addr = addr;
	//atomic m_data_p = msg;
	call Packet.setPayloadLength(msg, len);
	m = call Pool.get();
	if (m != NULL) {
		memcpy(m, msg, sizeof(message_t));
		if (call Queue.enqueue(m) == SUCCESS) {
			// full block
			if (call Queue.size() == call Queue.maxSize()) {
				// initiate transaction
				post sendRTSTask();
			}
			return SUCCESS;
		} else {
			// leakage occurs
			assert(call Pool.size());
		}
	}
	// full
	return ENOMEM;
	
//	// prepare RTS; backoff is not here, it's in interface Backoff
//	rts_retries = 0;
//	rts_hdr->type = TYPE_RTS;
//	rts_hdr->dst = addr;
//	// NAV in ms, including CTS, data and ack tx time
//	rts_hdr->nav = RTS_NAV;
//	
//	block_idx = 0;
//	// send RTS
//	ret = call SubSend.send(AM_BROADCAST_ADDR, m_rts_p, sizeof(imac_header_t));
//	if (SUCCESS == ret) {
//		call State.forceState(S_SENDING_RTS);
//		rts_tx_timestamp = call LocalTime.get();
//		dbg("iMAC", "%s: S_SENDING_RTS\n", __FUNCTION__);
//	} else {
//		// CMAC: returns EALREADY bcoz beacon is being transmitted
//		dbg("forwarder", "%s: send fail 1\n", __FUNCTION__);
//	}
//	return ret;
}

am_addr_t pending_rss_nb;
event void ReadRssi.readDone(error_t result, uint16_t val) {
	int16_t ni;
	
	// sample noise plus interference when it is valid
	if (SUCCESS == result) {
		ni = (int16_t)val - 172;
		call SignalMap.updateNI(TRUE, pending_rss_nb, FALSE, ni);
	}
}

event void SubSend.sendDone(message_t* msg, error_t error ) {
	error_t ret = SUCCESS;
	imac_header_t *hdr = getHeader(msg);
//	if (SUCCESS == error) {
//		imac_header_t *hdr = getHeader(msg);
//		//call UartLog.logTxRx(DBG_FLAG, DBG_TX_FLAG, rts_retries, seqno, hdr->seqno, call SubAMPacket.destination(msg), hdr->type, hdr->dst, call State.getState());
//		call UartLog.logTxRx(DBG_FLAG, DBG_TX_FLAG, 0, 0, call SubAMPacket.destination(msg), hdr->type, hdr->dst, hdr->seqno, hdr->node_i);
//	}
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
				//signal Send.sendDone(m_data_p, error);
				sendDone(error);
				//call UartLog.logEntry(DBG_FLAG, DBG_TX_FAIL_FLAG, 0, error);
			}
			break;
		
		case S_SENDING_CTS:
			// assert(msg == m_cts_p)
			if (SUCCESS == error) {
				// sample NI
				pending_rss_nb = hdr->dst;
				call ReadRssi.read();
				
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
			// block transfer
//			if (++block_idx >= BLOCK_SIZE) {
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
				//signal Send.sendDone(m_data_p, error);
				sendDone(error);
//			} else {
//				post sendDataTask();
//			}
//			if (SUCCESS == error) {
//				// log tx here bcoz of block transfer
//				call UartLog.logEntry(TX_DONE_FLAG, m_data_addr, hdr->seqno, call LocalTime.get());
//			}
			break;
		
		case S_CANCELLING_RTS:
			// reset to S_IDLE first after cancellation; otherwise may get stuck if ensuing operations fail
			call State.forceState(S_IDLE);
			if (TYPE_RTS == pending_type) {
				// hear RTS to me; reply CTS
				ret = sendCTS(pending_nb);
				//signal Send.sendDone(m_data_p, ECANCEL);
				sendDone(ECANCEL);
			} else if (TYPE_CTS == pending_type) {
				// hear late CTS to me; reply DATA
				if (pending_nb == m_data_addr)
					post sendDataTask();
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
			//signal Send.sendDone(m_data_p, ret);
			sendDone(ret);
			//call UartLog.logEntry(DBG_FLAG, DBG_TX_FAIL_FLAG, 2, ret);
		} else {
			rts_tx_timestamp = call LocalTime.get();
			call State.forceState(S_SENDING_RTS);
			dbg("iMAC", "%s: S_SENDING_RTS\n", __FUNCTION__);
		}
	} else {
		call State.forceState(S_IDLE);
		dbg("iMAC", "%s: S_IDLE\n", __FUNCTION__);
		//signal Send.sendDone(m_data_p, FAIL);
		sendDone(FAIL);
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
//	uint8_t backoff_type_;
//	atomic backoff_type_ = backoff_type;
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
//#warning CCA disabled
//	callBackoff.setCca(FALSE);
//	message_t *m_data_p_;
//	atomic m_data_p_ = m_data_p;
	//if (call State.isState(S_SENDING_CTS) || call State.isState(S_SENDING_DATA)) {
//	if (msg == m_cts_p || msg == m_data_p_) {
//	if (msg == m_cts_p || (!call Queue.empty() && msg == call Queue.head())) {
	if (msg != m_rts_p) {
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

task void sendRTSTask() {
	error_t ret;
	
	// prepare RTS; backoff is not here, it's in interface Backoff
	rts_retries = 0;
	rts_hdr->type = TYPE_RTS;
	rts_hdr->dst = m_data_addr;
	// NAV in ms, including CTS, data and ack tx time
	rts_hdr->nav = RTS_NAV;
	
	block_idx = 0;
	// send RTS
	ret = call SubSend.send(AM_BROADCAST_ADDR, m_rts_p, sizeof(imac_header_t));
	if (SUCCESS == ret) {
		call State.forceState(S_SENDING_RTS);
		rts_tx_timestamp = call LocalTime.get();
		dbg("iMAC", "%s: S_SENDING_RTS\n", __FUNCTION__);
	} else {
		// CMAC: returns EALREADY bcoz beacon is being transmitted
		sendDone(FAIL);
	}
	//return ret;
}

error_t sendCTS(am_addr_t nb) {
	error_t ret;
//	int16_t interference_threshold, min_interference_threshold, node_i;
	int16_t in_gain;
//	uint8_t power_level;
	
	// remember the sender I'm talking to
	current_sender = nb;
	
	// prepare CTS
	cts_hdr->type = TYPE_CTS;
	cts_hdr->dst = nb;
//// not in basic RTS/CTS mode
//if (is_iMac_enabled) {
//	// load ER
//	ret = call SignalMap.getInterferenceThresholdPowerLevelNI(nb, FALSE, &interference_threshold, &min_interference_threshold, &power_level, &node_i);
//	if (ret != SUCCESS) {
//		//call UartLog.logEntry(DBG_FLAG, DBG_ER_FLAG, 12, nb);
//		return ret;
//	}
//	//call UartLog.logTxRx(DBG_FLAG, DBG_ER_FLAG, 14, 0, 0, ret, nb, interference_threshold, power_level);
//	cts_hdr->interference_threshold = interference_threshold;
//	cts_hdr->min_interference_threshold = min_interference_threshold;
//	cts_hdr->node_i = node_i;
//	cts_hdr->seqno = seqno++;
//#ifndef TOSSIM
//	// set power
//	call CC2420Packet.setPower(m_cts_p, power_level);
//#endif
//}
	// include data and ack
	cts_hdr->nav = CTS_NAV;
	
	in_gain = call SignalMap.getInboundGain(nb);
	if (in_gain != INVALID_GAIN) {
		cts_hdr->rx_signal = CC2420_DEF_RFPOWER_DBM_SCALED - in_gain;
	} else {
		assert(nb);
		cts_hdr->rx_signal = INVALID_DBM;
	}
	// local ni
	cts_hdr->ni = node_ni;
	
	// send CTS
	ret = call SubSend.send(AM_BROADCAST_ADDR, m_cts_p, sizeof(imac_header_t));
	if (SUCCESS == ret) {
		cts_tx_timestamp = call LocalTime.get();
		call State.forceState(S_SENDING_CTS);
		dbg("iMAC", "%s: S_SENDING_CTS\n", __FUNCTION__);
	}
	return ret;
}

task void sendDataTask() {
	error_t ret;
	message_t *m_data_p;
	
	if (call Queue.empty()) {
		assert(0);
		return;
	}
	
	m_data_p = call Queue.head();
	// prepare data
	data_hdr = getHeader(m_data_p);
	data_hdr->type = TYPE_DATA;
	// necessary for snoop to know receiver of this DATA pkt
	data_hdr->dst = m_data_addr;
	data_hdr->seqno = seqno++;
	// ACK requested
	ret = call Acks.requestAck(m_data_p);
	// assert(ret != SUCCESS);
	
	// SINR
	data_hdr->rx_signal = receiver_signal;
	data_hdr->ni = receiver_ni;
	data_hdr->block_idx = block_idx;
	
	// unicast data
	ret = call SubSend.send(m_data_addr, m_data_p, call SubPacket.payloadLength(m_data_p));
	if (SUCCESS == ret) {
		data_tx_timestamp = call LocalTime.get();
		call State.forceState(S_SENDING_DATA);
		dbg("iMAC", "%s: S_SENDING_DATA\n", __FUNCTION__);
	} else {
		call State.forceState(S_IDLE);
		dbg("iMAC", "%s: S_IDLE\n", __FUNCTION__);
		//signal Send.sendDone(m_data_p, ret);
		sendDone(ret);
	}
	//		return ret;
}

error_t err;
void sendDone(error_t error) {
	err = error;
	post sendDoneTask();
}

task void sendDoneTask() {
	message_t *m;
	if (call Queue.empty())
		assert(block_idx);

	m = call Queue.dequeue();
	call Pool.put(m);
	
	signal Send.sendDone(m, err);
	
	// block transfer
	if (S_SENDING_DATA == call State.getState())
		if (++block_idx < BLOCK_SIZE)
			post sendDataTask();
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
		
		// update link SINR
		updateLinkTableEntry(nb, call SubAMPacket.destination(msg), hdr->block_idx, hdr->rx_signal, hdr->ni);
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
		
		// sample ni during DATA reception
		if (!is_ack)
			node_ni = (post_rss << SCALE_L_SHIFT_BIT);
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
	//call UartLog.logTxRx(DBG_FLAG, DBG_RX_FLAG, 0, call SignalMap.inExRegion(nb, hdr->interference_threshold), hdr->interference_threshold, nb, hdr->type, hdr->dst, call State.getState());
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
				// sample NI receiving RTS, not DATA, bcoz it's fresher
				sampleNI(msg, nb, FALSE);
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
				// receiver's ni
				receiver_signal = hdr->rx_signal;
				receiver_ni = hdr->ni;
				cts_rx_timestamp = call LocalTime.get();
				//call UartLog.logTxRx(DBG_FLAG, DBG_TIMEOUT_FLAG, 0, 0, 0, 0, 0, nb, call LocalTime.get() - rts_sent_timestamp);
				// assert(m_data_addr == nb);
				// cancel RTS retx
				call CtsTimeoutTimer.stop();
				
				// C-MAC
				if (passConcurrencyCheck(m_data_addr)) {
					// reply data only pass concurrency check
					post sendDataTask();
				} else {
					call State.forceState(S_IDLE);
					//signal Send.sendDone(m_data_p, ERETRY);
					sendDone(ERETRY);
				}
			}
			break;
			
		case S_EXPECTING_DATA:
			// hear data; unicast, no need to check (hdr->dst == my_ll_addr)
			if (TYPE_DATA == hdr->type) {
				// sample NI
				//sampleNI(msg, nb, FALSE);
				//call UartLog.logTxRx(DBG_FLAG, DBG_TIMEOUT_FLAG, 1, 0, 0, 0, 0, nb, call LocalTime.get() - cts_sent_timestamp);
				// assert(hdr->dst == my_ll_addr);
				// last packet in a block received
				if (hdr->block_idx >= (BLOCK_SIZE - 1)) {
					// cancel data timeout timer
					call DataTimeoutTimer.stop();
					call State.forceState(S_IDLE);
				} else {
					// stay in S_EXPECTING_DATA to receive the remaining block
					call DataTimeoutTimer.startOneShot((BLOCK_SIZE - 1 - hdr->block_idx) * PACKET_TIME);
				}
				dbg("iMAC", "%s: S_IDLE\n", __FUNCTION__);
				//call UartLog.logEntry(RX_FLAG, nb, hdr->seqno, call LocalTime.get());
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

}
