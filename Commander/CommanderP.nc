#include "Timer.h"
#include "Commander.h"

module CommanderP @safe() {
  uses {
    interface Boot;

    interface SplitControl as AMControl;
	interface AMSend;
	interface Timer<TMilli> as MilliTimer;
    interface UartLog;
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
      call MilliTimer.startPeriodic(SYNC_PERIOD);
      //call UartLog.logEntry(33, 33, 0, 0);
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
    //call UartLog.logEntry(44, 44, sizeof(radio_count_msg_t), call Packet.maxPayloadLength());
    dbg("RadioCountToLedsC", "RadioCountToLedsC: timer fired, counter is %hu.\n", counter);
    if (locked) {
        //call UartLog.logEntry(55, 55, counter, 0);
      return;
    }
    else {
      sync_header_t* rcm = (sync_header_t*)call AMSend.getPayload(&packet, sizeof(sync_header_t));
      if (rcm == NULL) {
	    //call UartLog.logEntry(55, 66, counter, 0); 
	    return;
      }

      rcm->seqno = counter;
      call UartLog.logEntry(255, 0, 0, rcm->seqno);
      if (call AMSend.send(AM_BROADCAST_ADDR, &packet, sizeof(sync_header_t)) == SUCCESS) {
		dbg("RadioCountToLedsC", "RadioCountToLedsC: packet sent.\n", counter);
		//call UartLog.logEntry(0, 0, 0, counter);
		locked = TRUE;
      }
    }
  }

  event void AMSend.sendDone(message_t* bufPtr, error_t error) {
    if (&packet == bufPtr) {
      locked = FALSE;
    }
  }

}




