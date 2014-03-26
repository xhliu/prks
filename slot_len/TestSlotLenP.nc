/*
 * @ author: Xiaohui Liu (whulxh@gmail.com)
 * 
 * @ updated: 06/25/2012 
 */

 
#include "TestSlotLen.h"

module TestSlotLenP {
	uses {
		interface Boot;
		
		interface AMSend;
		interface Receive;
		interface Packet;
		
		interface SplitControl as AMControl;
		
		interface Timer<TMilli> as MilliTimer;
		interface UartLog;
		interface LocalTime<T32khz>;
		interface CC2420Config;
		interface RadioBackoff;
		
		interface PacketTimeStamp<T32khz, uint32_t>;
		interface GlobalTime<T32khz>;
		interface TimeSyncInfo;
	}
}
implementation {

uint32_t start_time;
uint16_t counter = 0;
bool locked = FALSE;
message_t packet;

event void Boot.booted() {
	call AMControl.start();
}

event void AMControl.startDone(error_t err) {
	if (err == SUCCESS) {
//		if (TOS_NODE_ID != SINK_NODE_ID)
//			call MilliTimer.startPeriodic(PERIOD);
		call MilliTimer.startOneShot(300000);
	} else {
		call AMControl.start();
	}
}

event void AMControl.stopDone(error_t err) {
// do nothing
}

event void CC2420Config.syncDone(error_t error) {
//	uint8_t channel = call CC2420Config.getChannel();
//	call UartLog.logEntry(DBG_FLAG, error, channel, call LocalTime.get() - start_time);
}

async event void RadioBackoff.requestCca(message_t *msg) {
//	call RadioBackoff.setCca(FALSE);
}

async event void RadioBackoff.requestInitialBackoff(message_t *msg) {}
async event void RadioBackoff.requestCongestionBackoff(message_t *msg) {}

uint32_t prev_global_time, prev_local_time;
event void MilliTimer.fired() {
//	uint8_t channel = call CC2420Config.getChannel();
//	start_time = call LocalTime.get();
//	call CC2420Config.setChannel(45 - channel);
//	call CC2420Config.sync();
		
//		float skew = call TimeSyncInfo.getSkew();
//		float abs_skew = (skew > 0) ? skew : -skew;
//		uint32_t global_now, local_interval, global_interval;
//		uint32_t local_now = call LocalTime.get();
//		error_t ret = call GlobalTime.getGlobalTime(&global_now);
//		if (ret == SUCCESS) {
//			local_interval = local_now - prev_local_time;
//			global_interval = global_now - prev_global_time;
//			prev_local_time = local_now;
//			prev_global_time = global_now;
//			if (counter != 0)
//				call UartLog.logTxRx(DBG_FLAG, DBG_FTSP_FLAG, 0, call TimeSyncInfo.getRootID(), call TimeSyncInfo.getNumEntries(), call TimeSyncInfo.getSeqNum(), global_interval, local_interval, abs_skew * 1000000UL);
//		}
//		if (counter++ < 65535U)
			call MilliTimer.startOneShot(PERIOD);
		
	if (locked) {
		return;
	} else {
		error_t ret;
		radio_count_msg_t* hdr = (radio_count_msg_t*)call Packet.getPayload(&packet, sizeof(radio_count_msg_t));
		hdr->src = TOS_NODE_ID;
		hdr->seqno = counter++;
		start_time = call LocalTime.get();
		ret = call AMSend.send(AM_BROADCAST_ADDR, &packet, sizeof(radio_count_msg_t));
		if (SUCCESS == ret) {
			locked = TRUE;
		}
	}
}

event void AMSend.sendDone(message_t* msg, error_t error) {
//	call UartLog.logEntry(DBG_FLAG, error, counter, call LocalTime.get() - start_time);
	//if (&packet == msg) {
//		radio_count_msg_t* hdr = (radio_count_msg_t*)call Packet.getPayload(msg, sizeof(radio_count_msg_t));
//		dbg("TestiMAC", "%s: Packet %hu sendDone w/ %hhu.\n", __FUNCTION__, hdr->seqno, error);
//		if (SUCCESS == error)
//			call UartLog.logEntry(TX_DONE_FLAG, receiver_, hdr->seqno, call LocalTime.get() - start_time);
//		else
//			call UartLog.logEntry(TX_DONE_FAIL_FLAG, receiver_, error, call LocalTime.get() - start_time);
		locked = FALSE;
	//}
}

//receive sync time
event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len) {
    uint32_t local_rx_timestamp = call PacketTimeStamp.timestamp(msg);
    uint32_t global_rx_timestamp = local_rx_timestamp;
    error_t is_synced = call GlobalTime.local2Global(&global_rx_timestamp);
    sync_header_t *hdr = (sync_header_t*) call Packet.getPayload(msg, sizeof(sync_header_t));
    call UartLog.logTxRx(DBG_FLAG, 18, call PacketTimeStamp.isValid(msg), 0, is_synced, 0, 0, hdr->seqno, global_rx_timestamp);
    return msg;
}

}
