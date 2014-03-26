#include "Timer.h"
#include "TestSM.h"
 
/**
 * @author Xiaohui Liu (whulxh@gmail.com)
 * @date   12/29/2011
 * @description: test signal map
 */

module TestSMC @safe() {
	uses {
		interface Leds;
		interface Boot;
		interface Timer<TMilli> as MilliTimer;
		
		interface SMSend as Send;
		interface Receive;
		interface Packet;
		interface AMPacket;
		
		interface SignalMap;
		
		interface SplitControl as AMControl;
		interface UartLog;
#ifndef TOSSIM
		// radio sync
		interface Receive as TimeSyncReceive;
		interface TimeSyncPacket<TMilli, uint32_t>;
		interface Packet as SyncPacket;
		
		// uart sync
		interface Receive as UartSyncReceive;
#endif
		interface LocalTime<TMilli>;
	}
}
implementation {

message_t packet;

bool locked;
uint16_t counter = 0;
// sync
bool is_sync = FALSE;

event void Boot.booted() {
	call AMControl.start();
}
event void AMControl.startDone(error_t err) {
	if (err == SUCCESS) {
		//uint32_t unit = PERIOD * PKT_CNT + BUFFER_TIME;
		//call MilliTimer.startOneShot(TOS_NODE_ID * unit);
		;	//call MilliTimer.startOneShot(PERIOD);
	} else {
		call AMControl.start();
	}
}

event void AMControl.stopDone(error_t err) {
// do nothing
}

event void MilliTimer.fired() {
	if (counter++ < PKT_CNT)
		call MilliTimer.startOneShot(PERIOD);
	
	if (locked) {
		return;
	} else {
		radio_count_msg_t* rcm = (radio_count_msg_t*)call Packet.getPayload(&packet, sizeof(radio_count_msg_t));
		if (rcm == NULL) {
			return;
		}
		rcm->counter[0] = counter;
		// specify power
		if (call Send.send(AM_BROADCAST_ADDR, &packet, sizeof(radio_count_msg_t), 3) == SUCCESS) {
			dbg("TestSMCDbg", "packet %hhu sent\n", counter);
			call UartLog.logEntry(TX_FLAG, counter, sizeof(radio_count_msg_t), 0);
			locked = TRUE;
		}
	}
}

event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len) {
	//radio_count_msg_t *m = (radio_count_msg_t *)call Packet.getPayload(msg, len);
	//am_addr_t nb = call AMPacket.source(msg);
	//if (0 == TOS_NODE_ID && 1 == nb)
		dbg("TestSMC", "packet %hu received over <%hu, %hu>, gain <%u, %u>\n", m->counter[0], nb, TOS_NODE_ID, call SignalMap.getInboundGain(nb), call SignalMap.getOutboundGain(nb));
	//call UartLog.logTxRx(RX_FLAG, 0, 0, 0, m->counter[0], len, nb, call SignalMap.getInboundGain(nb), call SignalMap.getOutboundGain(nb));
	return msg;
}

#ifndef TOSSIM
// listen to commander
event message_t* TimeSyncReceive.receive(message_t *msg, void *payload, uint8_t len) {
	//already sync
	if (is_sync)
		return msg;
	
	if (call TimeSyncPacket.isValid(msg)) {
		//remaining time to the first global sync
		uint32_t global_start_interval, unit;
		uint32_t local_time = call LocalTime.get();
		uint32_t local_event_time = call TimeSyncPacket.eventTime(msg);
		sync_header_t *hdr = (sync_header_t *) call SyncPacket.getPayload(msg, sizeof(sync_header_t));
		is_sync = TRUE;
		/*
		 * diff = local_event_time - hdr->globalTime;
		 * CONVERGE_TIME - (local_time - diff)
		 * CONVERGE_TIME is in commander's clock, has to be larger than clock skew among all nodes
		 */
		global_start_interval = CONVERGE_TIME + local_event_time - local_time - hdr->globalTime;
		// each node takes so much time
		unit = ((uint32_t)PERIOD) * PKT_CNT + BUFFER_TIME;
		call MilliTimer.startOneShot(global_start_interval + TOS_NODE_ID * unit);
		//call UartLog.logEntry(DBG_FLAG, is_sync, hdr->seqno, global_start_interval);
	}
	return msg;
}

// listen to serial
event message_t* UartSyncReceive.receive(message_t *msg, void *payload, uint8_t len) {
	//already sync
	if (is_sync)
		return msg;
	
	if (call TimeSyncPacket.isValid(msg)) {
		is_sync = TRUE;
		call MilliTimer.startOneShot(PERIOD);
		call UartLog.logEntry(RX_FLAG, is_sync, 0, 0);
	}
	return msg;
}
#endif


event void Send.sendDone(message_t* bufPtr, error_t error) {
	if (&packet == bufPtr) {
		locked = FALSE;
	}
}

}




