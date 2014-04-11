/*
 * @ author: Xiaohui Liu (whulxh@gmail.com)
 * 
 * @ updated: 	06/27/2012 
 				04/07/2012 03:23:55 PM 
 * @ description: wiring for iMAC
 */
configuration SignalMapC {
 	provides {
		interface AsyncAMSend as AMSend;
		interface AsyncReceive as Receive;
		interface AsyncPacket as Packet;
		
 		interface SignalMap;
 	};
}

implementation {
	components SignalMapP as SM;
	AMSend = SM;
	Receive = SM;
	Packet = SM;
	SignalMap = SM;
	
	components MainC;
	MainC.SoftwareInit -> SM;


//	components new AMSenderC(AM_IMAC_SM);
//	components new AMReceiverC(AM_IMAC_SM);
//	SM.SubSend -> AMSenderC;
//	SM.SubReceive -> AMReceiverC;
//	SM.SubPacket -> AMSenderC;
//	SM.SubAMPacket -> AMSenderC;
	components AsyncCC2420TransceiverC as AM;
	SM.SubSend -> AM.AMSend[AM_IMAC_SM];
	SM.SubReceive -> AM.Receive[AM_IMAC_SM];
	SM.SubPacket -> AM;
	SM.SubAMPacket -> AM;
	
	components LinkEstimatorC as LE;
	SM.LinkEstimator -> LE;
	
	components IMACControllerC as Controller;
	SM.Controller -> Controller;

	components CC2420XDriverLayerC as Driver;
	SM.RadioState -> Driver;
	SM.CC2420Packet -> AM;
	
	components UtilC;
	SM.Util -> UtilC;
	components UartLogC;
	SM.UartLog -> UartLogC;
	
	components LocalTimeMicroC;
	SM.LocalTime -> LocalTimeMicroC;
}
