#include "Timer.h"
#include "TestFtsp.h"

module TestFtspC @safe() {
  uses {
    interface Leds;
    interface Boot;
    interface Receive;
    interface AMSend;
    interface Timer<TMilli> as MilliTimer;
    interface SplitControl as AMControl;
    interface Packet;
    
    interface AMPacket;
    interface PacketTimeStamp<TMicro, uint32_t>;
    interface UartLog;

	// ftsp
	interface AMSend as SyncSend;
	interface Receive as SyncReceive;
	interface Packet as SyncPacket;
	interface GlobalTime<TMicro>;
  }
}
implementation {

  message_t packet;

  bool locked;
  uint16_t counter = 0;
  
  event void Boot.booted() {
    call AMControl.start();
  }

  event void AMControl.startDone(error_t err) {
    if (err == SUCCESS) {
      call MilliTimer.startPeriodic(100);
    }
    else {
      call AMControl.start();
    }
  }

  event void AMControl.stopDone(error_t err) {
    // do nothing
  }
  
  event void MilliTimer.fired() {
    counter++;
    dbg("TestFtspC", "TestFtspC: timer fired, counter is %hu.\n", counter);
    if (locked) {
      return;
    }
    else {
      radio_count_msg_t* rcm = (radio_count_msg_t*)call Packet.getPayload(&packet, sizeof(radio_count_msg_t));
      if (rcm == NULL) {
	return;
      }

      rcm->counter = counter;
      if (call AMSend.send(AM_BROADCAST_ADDR, &packet, sizeof(radio_count_msg_t)) == SUCCESS) {
	dbg("TestFtspC", "TestFtspC: packet sent.\n", counter);	
	locked = TRUE;
      }
    }
  }

  event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len) {
    dbg("TestFtspC", "Received packet of length %hhu.\n", len);
    if (len != sizeof(radio_count_msg_t)) {return msg;}
    else {
//      	radio_count_msg_t* rcm = (radio_count_msg_t*)payload;
//    	call UartLog.logTxRx(255, 16, __LINE__, 0, call PacketTimeStamp.isValid(msg), call AMPacket.source(msg), rcm->counter, 0, call PacketTimeStamp.timestamp(msg));
      }
      return msg;
  }

  event void AMSend.sendDone(message_t* msg, error_t error) {
    dbg("dbg", "%s, %s, %d\n", __FUNCTION__, __FILE__, __LINE__);
    if (&packet == msg) {
//    	radio_count_msg_t* rcm = (radio_count_msg_t*)call Packet.getPayload(msg, sizeof(radio_count_msg_t));
      locked = FALSE;
//	call UartLog.logTxRx(255, 16, __LINE__, 0, call PacketTimeStamp.isValid(msg), 0, rcm->counter, 0, call PacketTimeStamp.timestamp(msg));
    }
  }

//receive sync time
event message_t* SyncReceive.receive(message_t* msg, void* payload, uint8_t len) {
    uint32_t local_rx_timestamp = call PacketTimeStamp.timestamp(msg);
    uint32_t global_rx_timestamp = local_rx_timestamp;
    error_t is_synced = call GlobalTime.local2Global(&global_rx_timestamp);
    sync_header_t *hdr = (sync_header_t*) call SyncPacket.getPayload(msg, sizeof(sync_header_t));
    call UartLog.logTxRx(255, 255, call PacketTimeStamp.isValid(msg), call AMPacket.source(msg), is_synced, 0, 0, hdr->seqno, global_rx_timestamp);
    return msg;
}

event void SyncSend.sendDone(message_t* msg, error_t error) {};
}




