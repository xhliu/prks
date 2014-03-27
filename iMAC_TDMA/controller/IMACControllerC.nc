/*
 * @ author: Xiaohui Liu (whulxh@gmail.com)
 * 
 * @ updated: 07/02/2012 
 */
configuration IMACControllerC {
 	provides {
		interface AsyncAMSend as AMSend;
		interface AsyncReceive as Receive;
		interface AsyncPacket as Packet;
		
		interface AsyncStdControl as StdControl;
		interface IMACController;
	}
}

implementation {
	components IMACControllerP as Controller;
	AMSend = Controller;
	Receive = Controller;
	Packet = Controller;
	StdControl = Controller;
	IMACController = Controller;
	
	components MainC;
	MainC.SoftwareInit -> Controller;
	
	components SignalMapC as SM;
	Controller.SubSend -> SM;
	Controller.SubReceive -> SM;
	Controller.SubPacket -> SM;
//	components ActiveMessageC;
//	Controller.SubAMPacket -> ActiveMessageC;
	components AsyncCC2420TransceiverC as AM;
	Controller.SubAMPacket -> AM;
	Controller.CC2420Packet -> AM;
	
	components IMACForwarderC as Forwarder;
	Controller.ForwarderInfo -> Forwarder;
	components LinkEstimatorC as LE;
	Controller.LinkEstimator -> LE;
	Controller.SignalMap -> SM;
	
	// ftsp
	components TimeSyncMicroC as TimeSyncC;
	MainC.SoftwareInit -> TimeSyncC;
	TimeSyncC.Boot -> MainC;
	Controller.GlobalTime -> TimeSyncC;
	
	components UtilC;
	Controller.Util -> UtilC;
	components UartLogC;
	Controller.UartLog -> UartLogC;


	components LocalTimeMicroC;
	Controller.LocalTime -> LocalTimeMicroC;
	
	components RandomC;
	Controller.Random -> RandomC;
	
	//components CrcC;
	//Controller.Crc -> CrcC;
}
