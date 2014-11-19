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
	#ifndef MULTIHOP
		interface AsyncAMSend as AMSend;
	#else
		interface AsyncSend as AMSend;
	#endif
		interface AsyncReceive as Receive;
		interface AsyncPacket as Packet;
		interface AsyncAMPacket as AMPacket;
		
		interface AsyncSplitControl as ForwarderSwitch;
		interface AsyncStdControl as ControllerSwitch;
#else
	#ifndef MULTIHOP
		interface AMSend;
	#else
		interface Send as AMSend;
	#endif
		interface Receive;
		interface Packet;
#endif
	
	#ifdef MULTIHOP	
//		interface AsyncIntercept as Intercept;
		interface RootControl;
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
		interface Util;
		interface UartLog;
	#if defined(RANDOM_PKT_INTERVAL)
		interface Random;
	#endif
	#if defined(VARY_PERIOD)
		interface ForwarderInfo;
	#endif
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

//#include "TestiMACPUtils.nc"
inline uint32_t getGlobalTime() {
#if defined(DEFAULT_MAC) || defined(RTSCTS) || defined(CMAC)
#warning local time
	return call LocalTime.get();
#else
	uint32_t global_now;
	return (call GlobalTime.getGlobalTime(&global_now) == SUCCESS) ? global_now : 0;
#endif
}

#ifdef MULTIHOP
//#warning source
//am_addr_t sources[] = {104, 106, 107, 113, 116, 119, 122, 123, 125, 127};
//am_addr_t sources[] = {106, 107, 113, 116, 119, 125, 127};
am_addr_t sources[] = {22, 28};
bool isSource() {
	return (call Util.getReceiver() != INVALID_ADDR);
//	uint8_t i;
//	
//	for (i = 0; i < sizeof(sources) / sizeof(sources[0]); i++) {
//		if (sources[i] == TOS_NODE_ID)
//			return TRUE;
//	}
//	return FALSE;
}
#endif

event void Boot.booted() {
	call AMControl.start();
	is_sync = FALSE;
	atomic locked = FALSE;
	atomic counter = 0;
	activeLinks = call Util.getActiveLinks(&active_link_size);
	atomic my_receiver = call Util.getReceiver();
#ifdef MULTIHOP
	if (ROOT_NODE_ID == TOS_NODE_ID)
		call RootControl.setRoot();
#endif
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
void sendTask() {
#else
void task sendTask() {
#endif
	error_t ret;
	radio_count_msg_t* hdr = (radio_count_msg_t*)call Packet.getPayload(&packet, sizeof(radio_count_msg_t));
	hdr->src = TOS_NODE_ID;
	atomic hdr->seqno = counter++;
#ifndef MULTIHOP
	ret = call AMSend.send(my_receiver, &packet, sizeof(radio_count_msg_t));
#else
	ret = call AMSend.send(&packet, sizeof(radio_count_msg_t));
#endif
	if (SUCCESS == ret) {
		dbg("TestiMAC", "%s: sending pkt %hu.\n", __FUNCTION__, counter);
		atomic locked = TRUE;
	#ifndef MULTIHOP
	#if !defined(VARY_PERIOD)
		call UartLog.logEntry(TX_SUCCESS_FLAG, my_receiver, hdr->seqno, getGlobalTime());
	#else
		call UartLog.logEntry(TX_SUCCESS_FLAG, my_receiver, hdr->seqno, call ForwarderInfo.getPeriod());
	#endif
	#endif
	} else {
		dbg("TestiMAC", "%s: sending pkt %hu failed.\n", __FUNCTION__, counter);
//	#ifndef MULTIHOP
		call UartLog.logEntry(TX_FAIL_FLAG, ret, hdr->seqno, getGlobalTime());
//	#endif
//	#if defined(DEFAULT_MAC) || defined(RTSCTS) || defined(CMAC)
//		#warning repost when tx fails
//		post sendTask();
//	#endif
	}
}

//async event void MilliAlarm.fired() {
event void MilliTimer.fired() {
	bool locked_;
	uint16_t counter_;
	uint32_t period;
	
	period = PERIOD_MILLI;
#if defined(RANDOM_PKT_INTERVAL)
	// [1/2 3/2]
	period = (call Random.rand16()) % period + period / 2;
#elif defined(VARY_PERIOD)
	period = call ForwarderInfo.getPeriod();
//	// randomize to even out pkt arrivals
//	// [1/16 31/16]
//	period = (call Random.rand16()) % ((period * 30) >> 4) + (period >> 4);
#endif
//#warning VARY_PERIOD log	
//	call UartLog.logEntry(DBG_FLAG, DBG_HEARTBEAT_FLAG, __LINE__, period);
	
	atomic {
	#if !defined(CMAC)
		locked_ = locked;
	#else
		// CMAC buffers message, no need to cleared; 
		locked_ = FALSE;
	#endif
		counter_ = counter;
	}
#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
	if (0 == counter_) {
		// turn on iMAC controller: initialize ER using signal map
		call ControllerSwitch.start();
		call UartLog.logEntry(DBG_FLAG, DBG_HEARTBEAT_FLAG, __LINE__, my_receiver);
		return;
	}
#endif
#ifndef MULTIHOP
	if (my_receiver == INVALID_ADDR)
#else
	// source only
	if (!isSource())
#endif
		return;

	if (counter_ < MAX_PKT_CNT) {
		call MilliTimer.startOneShot(period);
	}
	
	if (!locked_) {
	#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
		sendTask();
	#else
		post sendTask();		
	#endif
	}
//		interval = INITIAL_FTSP_TIME - INITIAL_ER_TIME;
////		#warning sender waits for receiver
////		if (my_receiver != INVALID_ADDR)
////			interval += 20000;
//		call MilliTimer.startOneShot(PERIOD_MILLI);
//		atomic counter++;
//		return;
//	} 
//	else if (1 == counter_) {
////	#warning "freeze SM"
////		call SignalMap.freeze();
////		call SignalMap.printSignalMap(0);
//	#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
//		// turn on iMAC forwarder: tx DATA
//		call ForwarderSwitch.start();
//	#endif
//	}

//	// set back
//	atomic locked = locked_;
}

task void startDataTask() {
	call MilliTimer.startOneShot(PERIOD_MILLI);
}
#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
async event void ForwarderSwitch.startDone(error_t error) {
//	if (my_receiver != INVALID_ADDR)
//		sendTask();
	atomic counter++;
	post startDataTask();
}
#endif

#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
async event void AMSend.sendDone(message_t* msg, error_t error) {
#else
event void AMSend.sendDone(message_t* msg, error_t error) {
#endif
#ifndef MULTIHOP
	radio_count_msg_t* hdr = (radio_count_msg_t*)call Packet.getPayload(msg, sizeof(radio_count_msg_t));
	dbg("TestiMAC", "%s: Packet %hu sendDone w/ %hhu.\n", __FUNCTION__, hdr->seqno, error);
	if (SUCCESS == error) {
		// CMAC block transfer
		call UartLog.logEntry(TX_DONE_FLAG, my_receiver, hdr->seqno, getGlobalTime());
	} else {
		call UartLog.logEntry(TX_DONE_FAIL_FLAG, error, hdr->seqno, getGlobalTime());
	}
#endif
	atomic locked = FALSE;
//#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
//	#warning saturate
//	sendTask();
//#endif
}

#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
async event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len) {
#else
event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len) {
#endif
	// CMAC block transfer
#ifndef MULTIHOP
	radio_count_msg_t* hdr = (radio_count_msg_t*)call Packet.getPayload(msg, sizeof(radio_count_msg_t));
	dbg("TestiMAC", "%s: Receive packet %hu.\n", __FUNCTION__, hdr->seqno);
	call UartLog.logEntry(RX_FLAG, hdr->src, hdr->seqno, getGlobalTime());
#endif
	return msg;
}

//#ifdef MULTIHOP
//#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
//	async
//#endif
//event bool Intercept.forward(bool is_incoming, message_t* msg, void* payload, uint8_t len) {
//#warning intercept disabled
//	radio_count_msg_t* hdr = (radio_count_msg_t*)call Packet.getPayload(msg, sizeof(radio_count_msg_t));
//	dbg("TestiMAC", "%s: Receive packet %hu.\n", __FUNCTION__, hdr->seqno);
//	if (is_incoming) {
//		call UartLog.logEntry(RX_FLAG, hdr->src, hdr->seqno, call LocalTime.get());
//	} else {
//		call UartLog.logEntry(TX_DONE_FLAG, hdr->src, hdr->seqno, call LocalTime.get()); 
//	}
//	return TRUE;
//}
//#endif


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
