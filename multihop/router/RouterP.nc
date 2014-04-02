/* *
 * @ author: Xiaohui Liu (whulxh@gmail.com) 
 * @ updated: 7/23/2012 
 */

module RouterP {
	provides {
		interface Init;
		
#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
		interface AsyncSend as Send;
		interface AsyncReceive as Receive;
		interface AsyncIntercept as Intercept;
		interface AsyncPacket as Packet;
#else
		interface Send;
		interface Receive;
		interface AsyncIntercept as Intercept;
		interface Packet;
#endif
		interface RootControl;
	}

	uses {
	#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
		interface AsyncAMSend as SubSend;
		interface AsyncReceive as SubReceive;
		interface AsyncPacket as SubPacket;
		interface AsyncAMPacket as SubAMPacket;
	#else
		interface AMSend as SubSend;
		interface Receive as SubReceive;
		interface Packet as SubPacket;
		interface AMPacket as SubAMPacket;
	#endif
//		interface AsyncQueue<fe_queue_entry_t*> as SendQueue;
//		interface AsyncPool<fe_queue_entry_t> as QEntryPool;
//		interface AsyncPool<message_t> as MessagePool;
		interface AsyncQueue<fe_queue_entry_t> as SendQueue;
		interface AsyncCache<message_t*> as SentCache;
//	#ifdef ACK_LAYER
		interface PacketAcknowledgements as Acks;
//	#endif
		interface Util;
		interface UartLog;
	#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
		interface GlobalTime<TMicro>;
	#endif
		interface LocalTime<TMilli>;
  	}
}
implementation {

bool sending;
bool is_root;
uint16_t seqno;

message_t m_buf;
message_t *m_buf_p;
router_header_t *m_buf_hdr;

inline uint32_t getGlobalTime() {
#if defined(DEFAULT_MAC) || defined(RTSCTS) || defined(CMAC)
#warning local time
	return call LocalTime.get();
#else
	uint32_t global_now;
	return (call GlobalTime.getGlobalTime(&global_now) == SUCCESS) ? global_now : 0;
#endif
}

//-------------------------------------------------------------------------------------------------
// forward declaration
//-------------------------------------------------------------------------------------------------
task void sendTask();
message_t* forward(message_t* m);

router_header_t* getHeader(message_t* m) {
	return (router_header_t*)call SubPacket.getPayload(m, sizeof(router_header_t));
}


command error_t Init.init() {
	seqno = 0;
	sending = FALSE;
	is_root = FALSE;
	m_buf_p = &m_buf;
	m_buf_hdr = getHeader(m_buf_p);
	return SUCCESS;
}

//-------------------------------------------------------------------------------------------------
// Interface RootControl
//-------------------------------------------------------------------------------------------------
command bool RootControl.isRoot() {
	return is_root;
}
command error_t RootControl.setRoot() {
	atomic is_root = TRUE;
	return SUCCESS;
}
command error_t RootControl.unsetRoot() {
	is_root = FALSE;
	return SUCCESS;
}

//-------------------------------------------------------------------------------------------------
// Interface Send
//-------------------------------------------------------------------------------------------------
#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
async 
#endif
command error_t Send.send(message_t* msg, uint8_t len) {
	fe_queue_entry_t qe;
	
	if (len > call Send.maxPayloadLength()) {
		return ESIZE;
	}
	
	qe.origin = TOS_NODE_ID;
	atomic qe.originSeqNo = seqno++;
	qe.retries = MAX_RETRIES;
	call Packet.setPayloadLength(m_buf_p, len);
	
	if (call SendQueue.enqueue(qe) == SUCCESS) {
		post sendTask();
		return SUCCESS;
	} else {
		return FAIL;
	}
}

task void sendTask() {
	bool sending_;
	atomic sending_ = sending;
	if (sending_) {
		return;
	} else if (call SendQueue.empty()) {
	//XL: queue can be empty but there is one packet pending retx already dequeued
	//} else if (!is_retx && call SendQueue.empty()) {
		return;
	} else {
		error_t ret;
		// must be a node in the tree if reaching here
		am_addr_t parent = call Util.getReceiver();
		fe_queue_entry_t qe = call SendQueue.head();

		m_buf_hdr->origin = qe.origin;
		m_buf_hdr->originSeqNo = qe.originSeqNo;
		call Acks.requestAck(m_buf_p);
		ret = call SubSend.send(parent, m_buf_p, call SubPacket.payloadLength(m_buf_p));
		if (SUCCESS == ret) {
			atomic sending = TRUE;
			call UartLog.logTxRx(TX_SUCCESS_FLAG, call Util.getReceiver(), qe.originSeqNo, qe.origin, call SubPacket.payloadLength(m_buf_p), 0, 0, 0, getGlobalTime());
//			if (getHeader(qe->msg)->origin == 22)
//				call UartLog.logTxRx(DBG_FLAG, DBG_TX_FLAG, __LINE__, 0, 0, 0, 0, getHeader(qe->msg)->origin, getHeader(qe->msg)->originSeqNo);
		} else {
			// retry
			post sendTask();
		}
	}
}

void dequeue(bool is_acked) {
	fe_queue_entry_t qe = call SendQueue.head();
	
	if (qe.origin == TOS_NODE_ID) {
		// local: even drop means sendDone SUCCESS
		signal Send.sendDone(m_buf_p, SUCCESS);
    } else {
		// a forwarded packet
		signal Intercept.forward(FALSE, m_buf_p, call Packet.getPayload(m_buf_p, call Packet.payloadLength(m_buf_p)), call Packet.payloadLength(m_buf_p));
		
		if (is_acked)
			call SentCache.insert(m_buf_p);
    }
	call SendQueue.dequeue();
	// next
	post sendTask();
}

#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
async 
#endif
event void SubSend.sendDone(message_t* msg, error_t error) {
	fe_queue_entry_t qe = call SendQueue.head();

	atomic sending = FALSE;
    if (m_buf_p != msg) {
		assert(0);      // Not our packet, something is very wrong...
		return;
    } else if (error != SUCCESS) {
		// Immediate retransmission is the worst thing to do.
		post sendTask();
		return;
    }
	
	// sent
	call UartLog.logTxRx(TX_DONE_FLAG, call Util.getReceiver(), qe.originSeqNo, qe.origin, call SendQueue.size(), call Acks.wasAcked(msg), 0, 0, getGlobalTime());
//	call UartLog.logTxRx(DBG_FLAG, DBG_TX_FLAG, __LINE__, 0, 0, call SendQueue.size(), call Acks.wasAcked(msg), getHeader(msg)->origin, getHeader(msg)->originSeqNo);
	if (!call Acks.wasAcked(msg)) {
		if (--qe.retries) {
			post sendTask();
		} else {
			//max retries, dropping packets
			dequeue(FALSE);
			call UartLog.logTxRx(DBG_FLAG, DBG_LOSS_FLAG, __LINE__, 0, 0, 0, 0, getHeader(msg)->origin, getHeader(msg)->originSeqNo);
		}
    } else {
    	dequeue(TRUE);
    }
}


#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
async 
#endif
command error_t Send.cancel(message_t* msg) {
	return FAIL;
}

#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
async 
#endif
command uint8_t Send.maxPayloadLength() {
	return call Packet.maxPayloadLength();
}

#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
async 
#endif
command void* Send.getPayload(message_t* msg, uint8_t len) {
	return call Packet.getPayload(msg, len);
}

//-------------------------------------------------------------------------------------------------
// Interface Receive
//-------------------------------------------------------------------------------------------------
message_t* forward(message_t *m) {
	fe_queue_entry_t qe;
	router_header_t *hdr = getHeader(m);
	
	qe.origin = hdr->origin;
	qe.originSeqNo = hdr->originSeqNo;
	qe.retries = MAX_RETRIES;
	
	if (call SendQueue.enqueue(qe) == SUCCESS) {
		post sendTask();
	} else {
		call UartLog.logTxRx(DBG_FLAG, DBG_LOSS_FLAG, __LINE__, 0, 0, 0, 0, getHeader(m)->origin, getHeader(m)->originSeqNo);
	}
	return m;
}


#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
async 
#endif
event message_t* SubReceive.receive(message_t* msg, void* payload, uint8_t len) {
    uint8_t i;
    fe_queue_entry_t qe;
    router_header_t *hdr = getHeader(msg);
    bool duplicate = FALSE;
    bool is_root_;

	//call UartLog.logTxRx(DBG_FLAG, DBG_TX_FLAG, __LINE__, len, call SubSend.maxPayloadLength(), call SubAMPacket.destination(msg), call SubAMPacket.source(msg), getHeader(msg)->origin, getHeader(msg)->originSeqNo);
    if (len > call SubSend.maxPayloadLength()) {
		return msg;
    }
	
    call UartLog.logTxRx(RX_FLAG, getHeader(msg)->origin, getHeader(msg)->originSeqNo, call SubAMPacket.source(msg), 0, 0, 0, 0, getGlobalTime());

    //See if we remember having seen this packet
    //We look in the sent cache ...
    if (call SentCache.lookup(msg)) {
        return msg;
    }
    //... and in the queue for duplicates
    for (i = call SendQueue.size(); i > 0; i--) {
		qe = call SendQueue.element(i - 1);
		if (qe.origin == hdr->origin && qe.originSeqNo == hdr->originSeqNo) {
			duplicate = TRUE;
			break;
		}
    }
    if (duplicate) {
        return msg;
    }
	
	call SubPacket.setPayloadLength(m_buf_p, len);
	
    // If I'm the root, signal receive. 
    atomic is_root_ = is_root;
   	if (is_root_) {
    	// once packet reception only once
    	// otherwise dup detection does not work for root since its queue and cache are empty
	    call SentCache.insert(msg);
	    return signal Receive.receive(msg, call Packet.getPayload(msg, call Packet.payloadLength(msg)), call Packet.payloadLength(msg));
    } else {
	    if (signal Intercept.forward(TRUE, msg, call Packet.getPayload(msg, call Packet.payloadLength(msg)), call Packet.payloadLength(msg))) {
		    return forward(msg);
		} else {
			return msg;
		}
    }
}



//-------------------------------------------------------------------------------------------------
// Interface Packet
//-------------------------------------------------------------------------------------------------
#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
async 
#endif
command void Packet.clear(message_t* msg) {
	call SubPacket.clear(msg);
}

#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
async 
#endif
command uint8_t Packet.payloadLength(message_t* msg) {
	return call SubPacket.payloadLength(msg) - sizeof(router_header_t);
}

#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
async 
#endif
command void Packet.setPayloadLength(message_t* msg, uint8_t len) {
	call SubPacket.setPayloadLength(msg, len + sizeof(router_header_t));
}

#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
async 
#endif
command uint8_t Packet.maxPayloadLength() {
	return call SubPacket.maxPayloadLength() - sizeof(router_header_t);
}

#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
async 
#endif
command void* Packet.getPayload(message_t* msg, uint8_t len) {
	uint8_t* payload = call SubPacket.getPayload(msg, len + sizeof(router_header_t));
	if (payload != NULL) {
		payload += sizeof(router_header_t);
	}
	return payload;
}


}
