#include "Router.h"

/* *
 * @ author: Xiaohui Liu (whulxh@gmail.com) 
 * @ updated: 7/23/2012 
 * @ description: networking layer buffering packets to be forwarded and retx
 */

configuration RouterC {
	provides {
#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
 		interface AsyncSplitControl as ForwarderSwitch;
 		interface AsyncStdControl as ControllerSwitch;
		interface AsyncSend as Send;
		interface AsyncReceive as Receive;
		interface AsyncIntercept as Intercept;
		interface AsyncPacket as Packet;
#else
		interface Send;
		interface Receive;
		interface AsyncIntercept as Intercept;
		interface Packet;
#endif
		interface RootControl;
	}
}

implementation {
#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
	components IMACForwarderC as Forwarder;
	ForwarderSwitch = Forwarder;
	components IMACControllerC as Controller;
	ControllerSwitch = Controller;
#else
	components ActiveMessageC as Forwarder;
#endif
	components RouterP as ImplP;
	
	components MainC;
	MainC.SoftwareInit -> ImplP;
	
	Send = ImplP;
	Receive = ImplP;
	Intercept = ImplP;
	Packet = ImplP;
	
	RootControl = ImplP;
	
#if defined(DEFAULT_MAC)
	ImplP.SubSend -> Forwarder.AMSend[AM_IMAC_LE];
	ImplP.SubReceive -> Forwarder.Receive[AM_IMAC_LE];
#else
	ImplP.SubSend -> Forwarder;
	ImplP.SubReceive -> Forwarder;
#endif
	ImplP.SubPacket -> Forwarder;
	
	components new AsyncQueueC(fe_queue_entry_t, QUEUE_SIZE) as SendQueueC;
	ImplP.SendQueue -> SendQueueC;
	components new AsyncCacheC(CACHE_SIZE) as SentCacheC;
	ImplP.SentCache -> SentCacheC;
	SentCacheC.SubPacket -> Forwarder;
//#ifdef ACK_LAYER	
#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
	components AsyncCC2420TransceiverC as AM;
#else
	components ActiveMessageC as AM;
#endif
	ImplP.Acks -> AM;
//#endif
	ImplP.SubAMPacket -> AM;
		
	components UtilC;
	ImplP.Util -> UtilC;
	components UartLogC;
	ImplP.UartLog -> UartLogC;
	
#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
 	components TimeSyncMicroC as TimeSyncC;
 	MainC.SoftwareInit -> TimeSyncC;
 	TimeSyncC.Boot -> MainC;
 	ImplP.GlobalTime -> TimeSyncC;	
#endif
 	components LocalTimeMilliC;
	ImplP.LocalTime -> LocalTimeMilliC;
}
