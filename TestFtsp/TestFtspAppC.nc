// $Id: TestFtspAppC.nc,v 1.5 2010-06-29 22:07:17 scipio Exp $

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
 
#include "TestFtsp.h"

/**
 * Configuration for the TestFtsp application. TestFtsp 
 * maintains a 4Hz counter, broadcasting its value in an AM packet 
 * every time it gets updated. A TestFtsp node that hears a counter 
 * displays the bottom three bits on its LEDs. This application is a useful 
 * test to show that basic AM communication and timers work.
 *
 * @author Philip Levis
 * @date   June 6 2005
 */

configuration TestFtspAppC {}
implementation {
	components MainC, TestFtspC as App, LedsC;
	components new TimerMilliC();

//	components new AMSenderC(AM_RADIO_COUNT_MSG);
//	components new AMReceiverC(AM_RADIO_COUNT_MSG);
//	components ActiveMessageC;
//	App.Receive -> AMReceiverC;
//	App.AMSend -> AMSenderC;
//	App.AMControl -> ActiveMessageC;
//	App.Packet -> AMSenderC;
//	App.AMPacket -> ActiveMessageC;
//	components CC2420PacketC;
//	App.PacketTimeStamp32khz -> CC2420PacketC;
	
	// CC2420X
	components CC2420XActiveMessageC as AM;
	App.Receive -> AM.Receive[AM_RADIO_COUNT_MSG];
	App.AMSend -> AM.AMSend[AM_RADIO_COUNT_MSG];
	App.AMControl -> AM;
	App.Packet -> AM;
	App.AMPacket -> AM;
	App.PacketTimeStamp -> AM.PacketTimeStampRadio;


	App.Boot -> MainC.Boot;

	App.Leds -> LedsC;
	App.MilliTimer -> TimerMilliC;
	components UartLogC;
	App.UartLog -> UartLogC;
	
	// ftsp
//	components new AMSenderC(TYPE_SYNC);
//	components new AMReceiverC(TYPE_SYNC);
//	App.SyncSend -> AMSenderC;
//	App.SyncPacket -> AMSenderC;
//	App.SyncReceive -> AMReceiverC;
//	components FastCC2420TransceiverC as FastAM;
	App.SyncSend -> AM.AMSend[TYPE_SYNC];
	App.SyncPacket -> AM;
	App.SyncReceive -> AM.Receive[TYPE_SYNC];
//	components TimeSync32kC as TimeSyncC;
	components TimeSyncMicroC as TimeSyncC;
	MainC.SoftwareInit -> TimeSyncC;
	TimeSyncC.Boot -> MainC;
	App.GlobalTime -> TimeSyncC;

}
