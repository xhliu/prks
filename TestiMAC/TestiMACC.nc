/*
 * @ author: Xiaohui Liu (whulxh@gmail.com)
 * @ updated: 9/2/2012
 * @ description: test the async radio stack
 */
 
#include "TestiMAC.h"

configuration TestiMACC {}
implementation {
	components MainC, TestiMACP as App;
	
	App.Boot -> MainC.Boot;

#if defined(DEFAULT_MAC)
#warning IMAC disabled
 	components ActiveMessageC as AM;
 	App.AMSend -> AM.AMSend[AM_IMAC_LE];
 	App.Receive -> AM.Receive[AM_IMAC_LE];
 	App.Packet -> AM;
	App.AMControl -> AM;
	//App.AMPacket -> AM;
#elif defined(RTSCTS) || defined(CMAC)
 	components IMACC;
 	components ActiveMessageC as AM;
 	App.AMSend -> IMACC;
 	App.Receive -> IMACC;
 	App.Packet -> IMACC;
	App.AMControl -> AM;
#else
	components IMACC;
	components AsyncCC2420TransceiverC as AM;
	App.AMSend -> IMACC;
	App.Receive -> IMACC;
	App.Packet -> IMACC;
 	App.ForwarderSwitch -> IMACC.ForwarderSwitch;
 	App.ControllerSwitch -> IMACC.ControllerSwitch;
	App.AMPacket -> AM;
	App.AMControl -> AM;
#endif
	components new TimerMilliC();
	App.MilliTimer -> TimerMilliC;
//	components new AlarmMilli32C();
//	App.MilliAlarm -> AlarmMilli32C;
	components LocalTimeMilliC;
	App.LocalTime -> LocalTimeMilliC;
	
	components UtilC;
	App.Util -> UtilC;
	components UartLogC;
	App.UartLog -> UartLogC;

#if defined(TEST_FTSP)
	// ftsp
	components AsyncCC2420TransceiverC as AsyncAM;
 	App.SyncPacket -> AsyncAM;
 	App.SyncReceive -> AsyncAM.Receive[TYPE_SYNC];
 	App.PacketTimeStamp -> AsyncAM;

 	components TimeSyncMicroC as TimeSyncC;
 	MainC.SoftwareInit -> TimeSyncC;
 	TimeSyncC.Boot -> MainC;
 	App.GlobalTime -> TimeSyncC;
#endif
}
