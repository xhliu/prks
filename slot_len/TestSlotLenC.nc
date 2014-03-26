/*
 * @ author: Xiaohui Liu (whulxh@gmail.com)
 * 
 * @ updated: 06/25/2012 
 */
 
#include "TestSlotLen.h"

configuration TestSlotLenC {}
implementation {
	components MainC, TestSlotLenP as App;
	components new TimerMilliC();
	components ActiveMessageC;
	
	App.Boot -> MainC.Boot;
	components new AMSenderC(AM_TYPE);
	components new AMReceiverC(AM_TYPE_SYNC);
	App.AMSend -> AMSenderC;
	App.Receive -> AMReceiverC;
	App.Packet -> AMSenderC;
	App.AMControl -> ActiveMessageC;
	
	App.MilliTimer -> TimerMilliC;
	
	components UartLogC;
	App.UartLog -> UartLogC;

//	components LocalTime32khzC;
	components Counter32khz32C, new CounterToLocalTimeC(T32khz) as LocalTime32khzC;
	LocalTime32khzC.Counter -> Counter32khz32C;
	App.LocalTime -> LocalTime32khzC;
	
	components CC2420ControlC;
	App.CC2420Config -> CC2420ControlC;
	
	components CC2420ActiveMessageC;
	App.RadioBackoff -> CC2420ActiveMessageC.RadioBackoff[AM_TYPE];
	
	App.PacketTimeStamp -> ActiveMessageC;
	// ftsp
	components TimeSync32kC as TimeSyncC;
	MainC.SoftwareInit -> TimeSyncC;
	TimeSyncC.Boot -> MainC;
	App.GlobalTime -> TimeSyncC;
	App.TimeSyncInfo -> TimeSyncC;
}
