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
		interface AsyncQueue<fe_queue_entry_t*> as SendQueue;
		interface AsyncPool<fe_queue_entry_t> as QEntryPool;
		interface AsyncPool<message_t> as MessagePool;
		interface AsyncCache<message_t*> as SentCache;
	#ifdef ACK_LAYER
		interface PacketAcknowledgements as Acks;
	#endif
		interface Util;
		interface UartLog;
  	}
}
implementation {

bool sending;
bool is_root;
uint8_t seqno;

fe_queue_entry_t q_buf;
fe_queue_entry_t* q_buf_p;


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
	q_buf_p = &q_buf;
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
    router_header_t *hdr;
	fe_queue_entry_t *qe;
	
	if (len > call Send.maxPayloadLength()) {	return ESIZE;	}
	atomic if (NULL == q_buf_p)				{	return EBUSY;	}
	
    call Packet.setPayloadLength(msg, len);
    hdr = getHeader(msg);
    hdr->origin = TOS_NODE_ID;
    atomic hdr->originSeqNo = seqno++;
	
	atomic qe = q_buf_p;
	qe->msg = msg;
	qe->retries = MAX_RETRIES;
	
	if (call SendQueue.enqueue(qe) == SUCCESS) {
		post sendTask();
		atomic q_buf_p = NULL;
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
		fe_queue_entry_t* qe = call SendQueue.head();
		//router_header_t *hdr = getHeader(qe->msg);
		uint8_t payloadLen = call SubPacket.payloadLength(qe->msg);
		// must be a source in the tree if reaching here
		am_addr_t parent = call Util.getReceiver();
	#ifdef ACK_LAYER	
		call Acks.requestAck(qe->msg);
	#endif
		ret = call SubSend.send(parent, qe->msg, payloadLen);
		if (SUCCESS == ret) {
			atomic sending = TRUE;
//			if (getHeader(qe->msg)->origin == 22)
//				call UartLog.logTxRx(DBG_FLAG, DBG_TX_FLAG, __LINE__, 0, 0, 0, 0, getHeader(qe->msg)->origin, getHeader(qe->msg)->originSeqNo);
		} else {
			// retry
			post sendTask();
		}
	}
}

void dequeue(bool is_acked) {
	fe_queue_entry_t *qe = call SendQueue.head();
	router_header_t *hdr = getHeader(qe->msg);
	
	if (hdr->origin == TOS_NODE_ID) {
		atomic q_buf_p = qe;
		// local: even drop means sendDone SUCCESS
		signal Send.sendDone(qe->msg, SUCCESS);
    } else {
		// a forwarded packet
		signal Intercept.forward(FALSE, qe->msg, call Packet.getPayload(qe->msg, call Packet.payloadLength(qe->msg)), call Packet.payloadLength(qe->msg));
		
		if (is_acked)
			call SentCache.insert(qe->msg);
		if (call MessagePool.put(qe->msg) != SUCCESS)
			assert(0);
		if (call QEntryPool.put(qe) != SUCCESS)
			assert(0);
    }
	call SendQueue.dequeue();
	// next
	post sendTask();
}

#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
async 
#endif
event void SubSend.sendDone(message_t* msg, error_t error) {
	fe_queue_entry_t *qe = call SendQueue.head();

	atomic sending = FALSE;
    if (NULL == qe || qe->msg != msg) {
		assert(0);      // Not our packet, something is very wrong...
		return;
    } else if (error != SUCCESS) {
		// Immediate retransmission is the worst thing to do.
		post sendTask();
		return;
    }
	
	// sent
//	call UartLog.logTxRx(DBG_FLAG, DBG_TX_FLAG, __LINE__, 0, 0, 0, call Acks.wasAcked(msg), getHeader(msg)->origin, getHeader(msg)->originSeqNo);
#ifdef ACK_LAYER
	if (!call Acks.wasAcked(msg)) {
#else
	if (TRUE) {
#endif
		if (--qe->retries) {
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
	if (call MessagePool.empty()) {
		call UartLog.logTxRx(DBG_FLAG, DBG_LOSS_FLAG, __LINE__, 0, 0, 0, 0, getHeader(m)->origin, getHeader(m)->originSeqNo);
	} else if (call QEntryPool.empty()) {
		call UartLog.logTxRx(DBG_FLAG, DBG_LOSS_FLAG, __LINE__, 0, 0, 0, 0, getHeader(m)->origin, getHeader(m)->originSeqNo);
	} else {
		message_t* newMsg;
		fe_queue_entry_t *qe;

		qe = call QEntryPool.get();
		if (NULL == qe) {
			return m;
		}

		newMsg = call MessagePool.get();
		if (NULL == newMsg) {
			// TODO: call QEntryPool.put(qe);
			return m;
		}

		memset(newMsg, 0, sizeof(message_t));
		memset(m->metadata, 0, sizeof(message_metadata_t));

		qe->msg = m;
		qe->retries = MAX_RETRIES;

		if (call SendQueue.enqueue(qe) == SUCCESS) {
			post sendTask();
			return newMsg;
		} else {
			call UartLog.logTxRx(DBG_FLAG, DBG_LOSS_FLAG, __LINE__, 0, 0, 0, 0, getHeader(m)->origin, getHeader(m)->originSeqNo);
			// There was a problem enqueuing to the send queue.
			if (call MessagePool.put(newMsg) != SUCCESS)
				assert(0);
			if (call QEntryPool.put(qe) != SUCCESS)
				assert(0);
		}
	}
	return m;
}
 
inline bool matchInstance(message_t *m1, message_t* m2) {
	return (getHeader(m1)->origin == getHeader(m2)->origin &&
			getHeader(m1)->originSeqNo == getHeader(m2)->originSeqNo);
}


#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
async 
#endif
event message_t* SubReceive.receive(message_t* msg, void* payload, uint8_t len) {
    uint8_t i;
    fe_queue_entry_t* qe;
    bool duplicate = FALSE;
    bool is_root_;

	//call UartLog.logTxRx(DBG_FLAG, DBG_TX_FLAG, __LINE__, len, call SubSend.maxPayloadLength(), call SubAMPacket.destination(msg), call SubAMPacket.source(msg), getHeader(msg)->origin, getHeader(msg)->originSeqNo);
	
    if (len > call SubSend.maxPayloadLength()) {
		return msg;
    }
    
    //See if we remember having seen this packet
    //We look in the sent cache ...
    if (call SentCache.lookup(msg)) {
        return msg;
    }
    //... and in the queue for duplicates
    for (i = call SendQueue.size(); i > 0; i--) {
		qe = call SendQueue.element(i - 1);
		if (matchInstance(qe->msg, msg)) {
			duplicate = TRUE;
			break;
		}
    }
    if (duplicate) {
        return msg;
    }

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
