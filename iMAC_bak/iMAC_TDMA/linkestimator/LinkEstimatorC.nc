/*
 * @ author: Xiaohui Liu (whulxh@gmail.com)
 * @ updated: 07/02/2012 
 */
#include "IMAC.h"

configuration LinkEstimatorC {
    provides {
		interface FastSend as AMSend;
		interface FastReceive as Receive;
		interface FastReceive as Snoop;
		interface FastPacket as Packet;

        interface LinkEstimator;
    };
}
implementation {
	components LinkEstimatorP as LE;
	AMSend = LE;
	Receive = LE.Receive;
	Snoop = LE.Snoop;
	Packet = LE;
	LinkEstimator = LE;
	
	components MainC;
	MainC.SoftwareInit -> LE;
	
//	components new AMSenderC(AM_IMAC_LE);
//	components new AMReceiverC(AM_IMAC_LE);
//	LE.SubSend -> AMSenderC;
//	LE.SubReceive -> AMReceiverC;
//	LE.SubPacket -> AMSenderC;
//	LE.SubAMPacket -> AMSenderC;
	components FastCC2420TransceiverC as AM;
	LE.SubSend -> AM.Send[AM_IMAC_LE];
	LE.SubReceive -> AM.Receive[AM_IMAC_LE];
	LE.SubSnoop -> AM.Snoop[AM_IMAC_LE];
	LE.SubPacket -> AM;
	LE.SubAMPacket -> AM;
	
	components IMACControllerC;
	LE.Controller -> IMACControllerC;
	components UtilC;
	LE.Util -> UtilC;
	components UartLogC;
	LE.UartLog -> UartLogC;
	
	
//	components Counter32khz32C, new CounterToLocalTimeC(T32khz) as LocalTime32khzC;
//	LocalTime32khzC.Counter -> Counter32khz32C;
//	LE.LocalTime -> LocalTime32khzC;
}
