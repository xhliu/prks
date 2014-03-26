/*
 * @ author: Xiaohui Liu (whulxh@gmail.com)
 * @ updated: 9/2/2012
 * @ description: test the async radio stack
 */

 
#include "TestiMAC.h"
#include "IMAC.h"
#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
	#include <Tasklet.h>
#endif

module TestiMACP {
	uses {
		interface Boot;

	#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
		interface AsyncSend as Send;
		interface AsyncReceive as Receive;
		interface AsyncIntercept as Intercept;
		interface AsyncPacket as Packet;
		interface AsyncAMPacket as AMPacket;
		
		interface AsyncSplitControl as ForwarderSwitch;
		interface AsyncStdControl as ControllerSwitch;
	#else
		interface Send;
		interface Receive;
		interface AsyncIntercept as Intercept;
		interface Packet;
	#endif
		interface SplitControl as AMControl;		
		interface Timer<TMilli> as MilliTimer;
		//interface Alarm<TMilli, uint32_t> as MilliAlarm;
		interface LocalTime<TMilli>;
		
	#if defined(TEST_FTSP)
		// ftsp
 		//interface AsyncSend as SyncSend;
 		interface AsyncReceive as SyncReceive;
 		interface AsyncPacket as SyncPacket;
 		interface PacketTimeStamp<TMicro, uint32_t>;
 		interface GlobalTime<TMicro>;
	#endif
		interface RootControl;
		interface Util;
		interface UartLog;
	}
}
implementation {

message_t packet;

bool is_sync;

bool locked;
uint16_t counter;

uint32_t start_time;

link_t *activeLinks;
uint8_t active_link_size;
am_addr_t my_receiver;

//#warning source
//am_addr_t sources[] = {104, 106, 107, 113, 116, 119, 122, 123, 125, 127};
//am_addr_t sources[] = {106, 107, 113, 116, 119, 125, 127};
am_addr_t sources[] = {22, 28};
bool isSource() {
	return TOS_NODE_ID != ROOT_NODE_ID;
//	uint8_t i;
//	
//	for (i = 0; i < sizeof(sources) / sizeof(sources[0]); i++) {
//		if (sources[i] == TOS_NODE_ID)
//			return TRUE;
//	}
//	return FALSE;
}

//#include "TestiMACPUtils.nc"

event void Boot.booted() {
	call AMControl.start();
	is_sync = FALSE;
	atomic locked = FALSE;
	atomic counter = 0;
	activeLinks = call Util.getActiveLinks(&active_link_size);
	atomic my_receiver = call Util.getReceiver();
	if (ROOT_NODE_ID == TOS_NODE_ID)
		call RootControl.setRoot();
}

event void AMControl.startDone(error_t err) {
	if (err == SUCCESS) {
		call MilliTimer.startOneShot(START_DATA_TIME);
	} else {
		call AMControl.start();
	}
}

event void AMControl.stopDone(error_t err) {}


#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
#else
task 
#endif
void sendTask() {
	error_t ret;
	radio_count_msg_t* hdr = (radio_count_msg_t*)call Packet.getPayload(&packet, sizeof(radio_count_msg_t));
	hdr->src = TOS_NODE_ID;
	atomic hdr->seqno = counter++;
	ret = call Send.send(&packet, sizeof(radio_count_msg_t));
	if (SUCCESS == ret) {
		dbg("TestiMAC", "%s: sending pkt %hu.\n", __FUNCTION__, counter);
		atomic locked = TRUE;
		call UartLog.logEntry(TX_SUCCESS_FLAG, my_receiver, hdr->seqno, call LocalTime.get());
	} else {
		dbg("TestiMAC", "%s: sending pkt %hu failed.\n", __FUNCTION__, counter);
		call UartLog.logEntry(TX_FAIL_FLAG, ret, hdr->seqno, call LocalTime.get());
	}
}

event void MilliTimer.fired() {
	bool locked_;
	uint16_t counter_;
	atomic {
	#if !defined(CMAC)
		locked_ = locked;
	#else
		// CMAC buffers message, no need to cleared; 
		locked_ = FALSE;
	#endif
		counter_ = counter;
	}
	call UartLog.logEntry(DBG_FLAG, DBG_HEARTBEAT_FLAG, __LINE__, counter_);
#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
	if (0 == counter_) {
		// turn on iMAC controller: initialize ER using signal map
		call ControllerSwitch.start();
		return;
	}
#endif
	// source only
	if (!isSource())
	//if (my_receiver == INVALID_ADDR)
		return;

	if (counter_ < MAX_PKT_CNT) {
		call MilliTimer.startOneShot(PERIOD_MILLI);
	}

	if (!locked_) {
	#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
	
	#else
		post	
	#endif
		sendTask();		
	}
}

task void startDataTask() {
	call MilliTimer.startOneShot(PERIOD_MILLI);
}
#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
async event void ForwarderSwitch.startDone(error_t error) {
	//call UartLog.logEntry(DBG_FLAG, DBG_HEARTBEAT_FLAG, __LINE__, my_receiver);
//	if (my_receiver != INVALID_ADDR)
//		sendTask();
	atomic counter++;
	post startDataTask();
}
#endif


#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
	async
#endif
event void Send.sendDone(message_t* msg, error_t error) {
	radio_count_msg_t* hdr = (radio_count_msg_t*)call Packet.getPayload(msg, sizeof(radio_count_msg_t));
	dbg("TestiMAC", "%s: Packet %hu sendDone w/ %hhu.\n", __FUNCTION__, hdr->seqno, error);
	if (SUCCESS == error) {
		// CMAC block transfer
	//#if !defined(CMAC)
		call UartLog.logEntry(TX_DONE_FLAG, hdr->src, hdr->seqno, call LocalTime.get());
	//#endif
	} else {
		call UartLog.logEntry(TX_DONE_FAIL_FLAG, error, hdr->seqno, call LocalTime.get());
	}
	atomic locked = FALSE;
}

#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
	async
#endif
event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len) {
	// CMAC block transfer
//#if !defined(CMAC)
	radio_count_msg_t* hdr = (radio_count_msg_t*)call Packet.getPayload(msg, sizeof(radio_count_msg_t));
	dbg("TestiMAC", "%s: Receive packet %hu.\n", __FUNCTION__, hdr->seqno);
	call UartLog.logEntry(RX_FLAG, hdr->src, hdr->seqno, call LocalTime.get());
//#endif
	return msg;
}

#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
	async
#endif
event bool Intercept.forward(bool is_incoming, message_t* msg, void* payload, uint8_t len) {
	radio_count_msg_t* hdr = (radio_count_msg_t*)call Packet.getPayload(msg, sizeof(radio_count_msg_t));
	dbg("TestiMAC", "%s: Receive packet %hu.\n", __FUNCTION__, hdr->seqno);
	if (is_incoming) {
		call UartLog.logEntry(RX_FLAG, hdr->src, hdr->seqno, call LocalTime.get());
	} else {
		call UartLog.logEntry(TX_DONE_FLAG, hdr->src, hdr->seqno, call LocalTime.get()); 
	}
	return TRUE;
}



#if defined(TEST_FTSP)
//receive sync time
async event message_t* SyncReceive.receive(message_t* msg, void* payload, uint8_t len) {
     uint32_t local_rx_timestamp = call PacketTimeStamp.timestamp(msg);
     uint32_t global_rx_timestamp = local_rx_timestamp;
     error_t is_synced = call GlobalTime.local2Global(&global_rx_timestamp);
     sync_header_t *hdr = (sync_header_t*) call SyncPacket.getPayload(msg, sizeof(sync_header_t));
     call UartLog.logTxRx(DBG_FLAG, 255, call PacketTimeStamp.isValid(msg), call AMPacket.source(msg), is_synced, 0, 0, hdr->seqno, global_rx_timestamp);
     return msg;
}
#endif
}
