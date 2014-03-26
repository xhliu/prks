// $Id: RadioCountToLedsC.nc,v 1.7 2010-06-29 22:07:17 scipio Exp $

/*									tab:4
 * Copyright (c) 2000-2005 The Regents of the University  of California.  
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the University of California nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Copyright (c) 2002-2003 Intel Corporation
 * All rights reserved.
 *
 * This file is distributed under the terms in the attached INTEL-LICENSE     
 * file. If you do not find these files, copies can be found by writing to
 * Intel Research Berkeley, 2150 Shattuck Avenue, Suite 1300, Berkeley, CA, 
 * 94704.  Attention:  Intel License Inquiry.
 */
 
#include "Timer.h"
#include "RadioCountToLeds.h"
 
/**
 * Implementation of the RadioCountToLeds application. RadioCountToLeds 
 * maintains a 4Hz counter, broadcasting its value in an AM packet 
 * every time it gets updated. A RadioCountToLeds node that hears a counter 
 * displays the bottom three bits on its LEDs. This application is a useful 
 * test to show that basic AM communication and timers work.
 *
 * @author Philip Levis
 * @date   June 6 2005
 */

module RadioCountToLedsC @safe() {
  uses {
    interface Leds;
    interface Boot;
    interface Timer<TMilli> as MilliTimer;
    interface SplitControl as AMControl;
//    interface Receive;
  //  interface AMSend;
    //interface Packet;

	interface FastSend as AMSend;
	interface FastReceive as Receive;
	interface FastPacket as Packet;
	
	interface PacketAcknowledgements as Acks;
	interface GeneralIO as CCA;
	interface UartLog;
  }
}
implementation {

  message_t packet;

  bool locked;
  uint16_t counter = 0;
  
//  task void serialPowerUp()
//  {
//    if( call SplitControl.start() != SUCCESS )
//      post serialPowerUp();
//  }

//  event void SplitControl.startDone(error_t error)
//  {
//    if( error != SUCCESS )
//      post serialPowerUp();
//    else
//      call AMControl.start();
//  }

//  event void SplitControl.stopDone(error_t error)
//  {
//  }

//  event void Boot.booted() {
//    post serialPowerUp();
//  }

  event void Boot.booted() {
    call AMControl.start();
  }
  
  event void AMControl.startDone(error_t err) {
    if (err == SUCCESS) {
      	call MilliTimer.startPeriodic(50);
    }
    else {
      call AMControl.start();
    }
  }

  event void AMControl.stopDone(error_t err) {
    // do nothing
  }
  
  event void MilliTimer.fired() {
    bool locked_;
    counter++;
    dbg("RadioCountToLedsC", "RadioCountToLedsC: timer fired, counter is %hu.\n", counter);
      call UartLog.logEntry(255, call CCA.get(), __LINE__, counter);
      #warning no tx
      if (TOS_NODE_ID >= 0)
      	return;

    atomic locked_ = locked;
    if (locked_) {
      return;
    }
    else {
      radio_count_msg_t* rcm = (radio_count_msg_t*)call Packet.getPayload(&packet, sizeof(radio_count_msg_t));
      if (rcm == NULL) {
	return;
      }

      rcm->counter = counter;
      //#warning
      call Acks.requestAck(&packet);
      //if (call AMSend.send(AM_BROADCAST_ADDR, &packet, sizeof(radio_count_msg_t)) == SUCCESS) {
      if (call AMSend.send(28, &packet, sizeof(radio_count_msg_t)) == SUCCESS) {
	dbg("RadioCountToLedsC", "RadioCountToLedsC: packet sent.\n", counter);	
	atomic locked = TRUE;
      }
    }
  }

  async event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len) {
      call UartLog.logEntry(255, len, __LINE__, sizeof(radio_count_msg_t));
    dbg("RadioCountToLedsC", "Received packet of length %hhu.\n", len);
    if (len != sizeof(radio_count_msg_t)) {return msg;}
    else {
      radio_count_msg_t* rcm = (radio_count_msg_t*)payload;
      call Leds.led1Toggle();
      return msg;
    }

  }

  async event void AMSend.sendDone(message_t* msg, error_t error) {
    if (&packet == msg) {
      atomic locked = FALSE;
      call UartLog.logEntry(255, error, __LINE__, call Acks.wasAcked(msg));
      //#warning
      //call UartLog.logEntry(255, error, __LINE__, 0);
    }
  }

}




