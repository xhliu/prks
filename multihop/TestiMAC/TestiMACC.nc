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

	components RouterC;
	App.Send -> RouterC;
	App.Receive -> RouterC;
 	App.Intercept -> RouterC;
	App.Packet -> RouterC;
#if defined(DEFAULT_MAC)
 	components ActiveMessageC as AM;
	App.AMControl -> AM;
	//App.AMPacket -> AM;
#elif defined(RTSCTS) || defined(CMAC)
 	components RouterC;
 	components ActiveMessageC as AM;
 	App.AMSend -> RouterC;
 	App.Receive -> RouterC;
 	App.Packet -> RouterC;
	App.AMControl -> AM;
#else
	components AsyncCC2420TransceiverC as AM;
 	App.ForwarderSwitch -> RouterC.ForwarderSwitch;
 	App.ControllerSwitch -> RouterC.ControllerSwitch;
	App.AMPacket -> AM;
	App.AMControl -> AM;
#endif
	App.RootControl -> RouterC;
	components new TimerMilliC();
	App.MilliTimer -> TimerMilliC;
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
