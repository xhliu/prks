/* *
 * @ author:    Xiaohui Liu (whulxh@gmail.com) 
 * @ updated:   08/31/2012 02:55:03 PM 
 * @ description: a sender/receiver that skips all layers from CC2420CsmaC to AMSenderC/AMReceiverC to speed up tx/rx
 */

configuration FastCC2420TransceiverC {
	provides {
		interface FastSend as Send[uint8_t client];
		interface FastReceive as Receive[uint8_t client];
		interface FastPacket as Packet;
		interface FastAMPacket as AMPacket;
//		interface PacketTimeSyncOffset;
	}
}

implementation {
	components FastCC2420TransceiverP;
	Send = FastCC2420TransceiverP;
	Receive = FastCC2420TransceiverP;
	Packet = FastCC2420TransceiverP;
	AMPacket = FastCC2420TransceiverP;
//	PacketTimeSyncOffset = FastCC2420TransceiverP;

	components MainC;
	MainC.SoftwareInit -> FastCC2420TransceiverP;

	components CC2420TransmitC;
	FastCC2420TransceiverP.SubSend -> CC2420TransmitC;
	components CC2420ReceiveC;
	FastCC2420TransceiverP.SubReceive -> CC2420ReceiveC;
	
	components ActiveMessageAddressC;
	FastCC2420TransceiverP.ActiveMessageAddress -> ActiveMessageAddressC;
	components CC2420PacketC;
	FastCC2420TransceiverP.CC2420PacketBody -> CC2420PacketC;
	components CC2420ControlC;
	FastCC2420TransceiverP.CC2420Config -> CC2420ControlC;
	
	components RandomC;
	FastCC2420TransceiverP.Random -> RandomC;
	
	components CrcC;
	FastCC2420TransceiverP.Crc -> CrcC;
	
	components UartLogC;
	FastCC2420TransceiverP.UartLog -> UartLogC;
}
