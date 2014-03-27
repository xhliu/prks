/*
 * Copyright (c) 2010, Vanderbilt University
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement is
 * hereby granted, provided that the above copyright notice, the following
 * two paragraphs and the author appear in all copies of this software.
 * 
 * IN NO EVENT SHALL THE VANDERBILT UNIVERSITY BE LIABLE TO ANY PARTY FOR
 * DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT
 * OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE VANDERBILT
 * UNIVERSITY HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE VANDERBILT UNIVERSITY SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND THE VANDERBILT UNIVERSITY HAS NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 *
 * Author: Miklos Maroti, Janos Sallai
 */

#include <RadioConfig.h>

configuration CC2420XTimeSyncMessageC
{
	provides
	{
		interface SplitControl;

		interface AsyncReceive as Receive[uint8_t id];
//		interface Receive as Snoop[am_id_t id];
//		interface Packet;
//		interface AMPacket;
//		interface PacketAcknowledgements;
//		interface LowPowerListening;

//		interface PacketTimeStamp<TRadio, uint32_t> as PacketTimeStampRadio;
		interface AsyncTimeSyncAMSend<TRadio, uint32_t> as TimeSyncAMSendRadio[am_id_t id];
		interface AsyncTimeSyncPacket<TRadio, uint32_t> as TimeSyncPacketRadio;

//		interface PacketTimeStamp<TMilli, uint32_t> as PacketTimeStampMilli;
//		interface TimeSyncAMSend<TMilli, uint32_t> as TimeSyncAMSendMilli[am_id_t id];
//		interface TimeSyncPacket<TMilli, uint32_t> as TimeSyncPacketMilli;
	}
}

implementation
{
//	components CC2420XActiveMessageC as ActiveMessageC, new TimeSyncMessageLayerC();
	components AsyncCC2420TransceiverC as ActiveMessageC, new TimeSyncMessageLayerC();
  
	SplitControl	= ActiveMessageC;
//	AMPacket	= TimeSyncMessageLayerC;
  	Receive		= TimeSyncMessageLayerC.Receive;
//	Snoop		= TimeSyncMessageLayerC.Snoop;
//	Packet		= TimeSyncMessageLayerC;
//	PacketAcknowledgements	= ActiveMessageC;
//	LowPowerListening	= ActiveMessageC;

//	PacketTimeStampRadio	= ActiveMessageC.PacketTimeStampRadio;
	TimeSyncAMSendRadio	= TimeSyncMessageLayerC;
	TimeSyncPacketRadio	= TimeSyncMessageLayerC;

//	PacketTimeStampMilli	= ActiveMessageC.PacketTimeStampMilli;
//	TimeSyncAMSendMilli	= TimeSyncMessageLayerC;
//	TimeSyncPacketMilli	= TimeSyncMessageLayerC;
	
	TimeSyncMessageLayerC.PacketTimeStampRadio -> ActiveMessageC.PacketTimeStampRadio;
//	TimeSyncMessageLayerC.PacketTimeStampMilli -> ActiveMessageC.PacketTimeStampMilli;

	components CC2420XDriverLayerC as DriverLayerC;
	TimeSyncMessageLayerC.LocalTimeRadio -> DriverLayerC;
	TimeSyncMessageLayerC.PacketTimeSyncOffset -> DriverLayerC.PacketTimeSyncOffset;
}
