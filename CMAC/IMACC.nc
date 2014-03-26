/*
 * @ author: Xiaohui Liu (whulxh@gmail.com)
 * 
 * @ updated: 04/07/2012 03:23:55 PM 
 * @ description: wiring for iMAC
 */
configuration IMACC {
 	provides {
 		interface AMSend;
 		interface Receive;
 		interface Packet;
 		
 		interface IMac;
 	};
}

implementation {
	components IMACForwarderP as Forwarder;
	AMSend = Forwarder;
	Receive = Forwarder;
	Packet = Forwarder;
	IMac = Forwarder;
	
	components MainC;
	components SignalMapP as SM;
	components LinkEstimatorP as LE;
	//components new AMSenderC(AM_IMAC_SM) as SMAMSenderC;
	components ActiveMessageC;
	components UartLogC;
	
	// forwarder
	MainC.SoftwareInit -> Forwarder;
	Forwarder.SubSend -> LE;
	Forwarder.SubReceive -> LE.Receive;
	Forwarder.SubSnoop -> LE.Snoop;
	Forwarder.SubPacket -> LE;
	//Forwarder.SubAMPacket -> LEAMSenderC;
	Forwarder.SubAMPacket -> ActiveMessageC;
#ifndef TOSSIM
	components CC2420ActiveMessageC;
	// does not interfere w/ non-iMac packets such as signal map beacons
	Forwarder.Backoff -> CC2420ActiveMessageC.RadioBackoff[AM_IMAC_LE];
	components RandomC;
	Forwarder.Random -> RandomC;
	components CC2420PacketC;
	Forwarder.CC2420Packet -> CC2420PacketC;
#endif
	components new TimerMilliC() as CtsTimeoutTimer;
	components new TimerMilliC() as DataTimeoutTimer;
	Forwarder.CtsTimeoutTimer -> CtsTimeoutTimer;
	Forwarder.DataTimeoutTimer -> DataTimeoutTimer;

	components LocalTimeMilliC;
	Forwarder.LocalTime -> LocalTimeMilliC;

	components new StateC();
	Forwarder.State -> StateC;
	
	//Forwarder.Acks -> SMAMSenderC;
	Forwarder.Acks -> ActiveMessageC;
	Forwarder.LinkEstimator -> LE;
	Forwarder.SignalMap -> SM;
	components new QueueC(message_t*, BLOCK_SIZE);
	Forwarder.Queue -> QueueC;
	components new PoolC(message_t, BLOCK_SIZE);
	Forwarder.Pool -> PoolC;
	
	Forwarder.UartLog -> UartLogC;
#ifndef TOSSIM
	components CC2420ControlC;
	Forwarder.ReadRssi -> CC2420ControlC;
#endif

	// beaconing
	components IMACBeaconP as Beacon;
	MainC.SoftwareInit -> Beacon;
	Beacon.BeaconSend -> SM;
	Beacon.BeaconReceive -> SM;
	
	components new TimerMilliC() as BeaconTimer;
	Beacon.BeaconTimer -> BeaconTimer;
#ifndef TOSSIM
	Beacon.CC2420Packet -> CC2420ActiveMessageC;
#endif	
	
	// link estimator
	MainC.SoftwareInit -> LE;
/*
	components new AMSenderC(AM_IMAC_LE) as LEAMSenderC;
	components new AMReceiverC(AM_IMAC_LE) as LEAMReceiverC;
	components new AMSnooperC(AM_IMAC_LE) as LEAMSnooperC;
	LE.SubSend -> LEAMSenderC;
	LE.SubReceive -> LEAMReceiverC;
	LE.SubSnoop -> LEAMSnooperC;
	LE.SubPacket -> LEAMSenderC;
	LE.SubAMPacket -> LEAMSenderC;
*/
	LE.SubSend -> ActiveMessageC.AMSend[AM_IMAC_LE];
	LE.SubReceive -> ActiveMessageC.Receive[AM_IMAC_LE];
	LE.SubSnoop -> ActiveMessageC.Snoop[AM_IMAC_LE];
	LE.SubPacket -> ActiveMessageC;
	LE.SubAMPacket -> ActiveMessageC;
	
	LE.UartLog -> UartLogC;
	
	
	// controller
	components IMACControllerP as Controller;
	
	// signal map
	//components new AMReceiverC(AM_IMAC_SM) as SMAMReceiverC;
	components new TimerMilliC() as OutboundERTimer;
	//components new TimerMilliC() as NISampleTimer;
	MainC.SoftwareInit -> SM;
/*
	SM.SubSend -> SMAMSenderC;
	SM.SubReceive -> SMAMReceiverC;
	SM.SubPacket -> SMAMSenderC;
	SM.SubAMPacket -> SMAMSenderC;
*/
	SM.SubSend -> ActiveMessageC.AMSend[AM_IMAC_SM];
	SM.SubReceive -> ActiveMessageC.Receive[AM_IMAC_SM];
	SM.SubPacket -> ActiveMessageC;
	SM.SubAMPacket -> ActiveMessageC;
#ifndef TOSSIM
	SM.CC2420Packet -> CC2420ActiveMessageC;
	//components CC2420ControlC;
	//SM.ReadRssi -> CC2420ControlC;
#endif
	SM.Controller -> Controller;
	SM.LinkEstimator -> LE;
	SM.UartLog -> UartLogC;
	SM.OutboundERTimer -> OutboundERTimer;
	//SM.NISampleTimer -> NISampleTimer;
	SM.LocalTime -> LocalTimeMilliC;
	// debug
	SM.IMac -> Forwarder;
}
