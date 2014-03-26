/* *
 * @ author: Xiaohui Liu (whulxh@gmail.com) 
 * @ updated: 06/27/2012 
 * @ description: data plane of iMAC, including data & ctrl packets
 */
#include "IMACForwarder.h"
#include "IMAC.h"

configuration IMACForwarderC {
 	provides {
		interface FastSend as AMSend;
		interface FastReceive as Receive;
		interface FastPacket as Packet;
 		
 		interface AsyncSplitControl as SplitControl;
 		interface ForwarderInfo;
 	};
}

implementation {
	components IMACForwarderP as Forwarder;
	AMSend = Forwarder;
	Receive = Forwarder;
	Packet = Forwarder;
	SplitControl = Forwarder;
	ForwarderInfo = Forwarder;

	components MainC;
	MainC.SoftwareInit -> Forwarder;
	
	components LinkEstimatorC as LE;
	Forwarder.SubSend -> LE;
	Forwarder.SubReceive -> LE.Receive;
	Forwarder.SubSnoop -> LE.Snoop;
	Forwarder.SubPacket -> LE;
//	Forwarder.SubAMPacket -> ActiveMessageC;
	components FastCC2420TransceiverC as AM;
	Forwarder.SubAMPacket -> AM;
	
	components IMACControllerC as Controller;
	Forwarder.CtrlSend -> Controller;
#ifdef SCREAM
	components HplCC2420XC as HplC;
	Forwarder.CCA -> HplC.CCA;
#endif
	components FastCC2420TransceiverC;
	Forwarder.Acks -> FastCC2420TransceiverC;
//	components CC2420ControlC;
//	Forwarder.CC2420Config -> CC2420ControlC;
//	components CC2420ActiveMessageC;
//	Forwarder.CC2420Packet -> CC2420ActiveMessageC;
//	Forwarder.CC2420Packet -> AM;
	components CC2420XDriverLayerC as Driver;
	Forwarder.RadioState -> Driver;
	Forwarder.PacketTransmitPower -> Driver.PacketTransmitPower;
	
	//components HplCC2420PinsC as Pins;
	//Forwarder.CCA -> Pins.CCA;
	
	
	Forwarder.LinkEstimator -> LE;
	components SignalMapC as SM;
	Forwarder.SignalMap -> SM;
	Forwarder.Controller -> Controller;
	
	
	// ForwarderInfoting time
	components LocalTimeMicroC;
	Forwarder.LocalTime -> LocalTimeMicroC;
	// ftsp
	components TimeSyncMicroC as TimeSyncC;
	MainC.SoftwareInit -> TimeSyncC;
	TimeSyncC.Boot -> MainC;
	Forwarder.GlobalTime -> TimeSyncC;
	
//	components new AlarmMicro16C() as BackoffTimer;
//	Forwarder.BackoffTimer -> BackoffTimer;
	//components new AlarmMicro16C() as SlotTimer;
	components new Alarm32khz32C() as SlotTimer32khz;
	Forwarder.SlotTimer32khz -> SlotTimer32khz;
	// added later; seems fine w/o for AlarmMicro16C
	MainC.SoftwareInit -> SlotTimer32khz;
	
	components new AlarmMicro32C() as CommSubSlotTimerMicro;
	Forwarder.CommSubSlotTimerMicro -> CommSubSlotTimerMicro;
	//MainC.SoftwareInit -> CommSubSlotTimerMicro;
	
	components new AlarmMicro32C() as ComputationSubSlotTimerMicro;
	Forwarder.ComputationSubSlotTimerMicro -> ComputationSubSlotTimerMicro;
	//MainC.SoftwareInit -> ComputationSubSlotTimerMicro;
	
	
	Forwarder.Boot -> MainC;
	
	components RandomC;
	Forwarder.Random -> RandomC;
	components UtilC;
	Forwarder.Util -> UtilC;
	components BusyWaitMicroC;
	Forwarder.BusyWait -> BusyWaitMicroC;
	components UartLogC;
	Forwarder.UartLog -> UartLogC;
	components CC2420XDriverLayerP;
	Forwarder.DriverInfo -> CC2420XDriverLayerP;
}
