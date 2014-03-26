#include "TestSM.h"

/**
 * @author Xiaohui Liu
 * @date   12/29/2011
 */

configuration TestSMAppC {}
implementation {
	components MainC, TestSMC as App, LedsC;
	components new AMSenderC(AM_RADIO_COUNT_MSG);
	components new AMReceiverC(AM_RADIO_COUNT_MSG);
	components new TimerMilliC();
	components ActiveMessageC;
	
	components SignalMapP as SM;
#ifdef TOSSIM	
	components DummyUartLogC as LogC;
#else
	components UartLogC as LogC;
#endif
	
	App.Boot -> MainC.Boot;
	App.AMControl -> ActiveMessageC;
	App.Leds -> LedsC;
	App.MilliTimer -> TimerMilliC;
 
	//wire SignalMapP: lower layer
	SM.SubSend -> AMSenderC;	
	SM.SubReceive -> AMReceiverC;
	SM.SubPacket -> AMSenderC;
	SM.SubAMPacket -> AMSenderC;
	
	MainC.SoftwareInit -> SM;
#ifndef TOSSIM
	components CC2420ActiveMessageC;
	// necessary to read packet RSSI
	SM.CC2420Packet -> CC2420ActiveMessageC;
#endif
	SM.UartLog -> LogC;
	
	App.Send -> SM;	
	App.Receive -> SM;
	App.Packet -> SM;
	App.AMPacket -> AMSenderC;
	App.SignalMap -> SM;
	App.UartLog -> LogC;

#ifndef TOSSIM	
	// radio sync
    components TimeSyncMessageC;
    App.TimeSyncReceive -> TimeSyncMessageC.Receive[AM_TYPE_SYNC];
    App.TimeSyncPacket -> TimeSyncMessageC.TimeSyncPacketMilli;
    App.SyncPacket -> TimeSyncMessageC;
    
    components new SerialAMReceiverC(AM_UART_SYNC);
    App.UartSyncReceive -> SerialAMReceiverC;
#endif    

    components LocalTimeMilliC;
    App.LocalTime -> LocalTimeMilliC;
}


