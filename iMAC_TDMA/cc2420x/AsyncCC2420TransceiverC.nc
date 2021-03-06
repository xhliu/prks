/* *
 * @ author:    Xiaohui Liu (whulxh@gmail.com) 
 * @ updated:   12/31/12 12:10 PM
 * @ description: migrate to CC2420X stack for time sync to work
 */
#include <RadioConfig.h>

configuration AsyncCC2420TransceiverC {
	provides {
		interface SplitControl;
		interface AsyncAMSend as AMSend[am_id_t id];
		interface AsyncReceive as Receive[am_id_t id];
		interface AsyncReceive as Snoop[am_id_t id];
		interface AsyncPacket as Packet;
		interface AsyncAMPacket as AMPacket;
		interface PacketTimeStamp<TRadio, uint32_t> as PacketTimeStampRadio;
		interface CC2420Packet;
		
		interface PacketAcknowledgements;
	}
}

implementation {
	components AsyncCC2420TransceiverP as Impl;
	SplitControl = Impl;
	AMSend = Impl;
	Receive = Impl.Receive;
	Snoop = Impl.Snoop;
	Packet = Impl;
	AMPacket = Impl;
	PacketTimeStampRadio = Impl;
	CC2420Packet = Impl;
	
	components MainC;
	MainC.SoftwareInit -> Impl;
	
	components CC2420XDriverLayerC as Driver;
//	Impl.SubSend -> Driver;
//	Impl.SubReceive -> Driver;
	Impl.SubPacket -> Driver;
	Impl.RadioState -> Driver;
	
	components ActiveMessageAddressC;
	Impl.ActiveMessageAddress -> ActiveMessageAddressC;
	components new Ieee154PacketLayerC();
	Impl.Ieee154PacketLayer -> Ieee154PacketLayerC;
	Ieee154PacketLayerC.SubPacket -> Driver;
	
	components new SimpleFcfsArbiterC(RADIO_SEND_RESOURCE) as SendResourceC;
	Impl.Resource -> SendResourceC.Resource[unique(RADIO_SEND_RESOURCE)];

	Impl.PacketRSSI -> Driver.PacketRSSI;
	Impl.PacketRSSIIdle -> Driver.PacketRSSIIdle;

	// ------- Driver
#define UQ_METADATA_FLAGS	"UQ_CC2420X_METADATA_FLAGS"
#define UQ_RADIO_ALARM		"UQ_CC2420X_RADIO_ALARM"
	Driver.Config -> Impl;
	Driver.PacketTimeStamp -> Impl;
	Impl.TimeStampFlag -> Impl.PacketFlag[unique(UQ_METADATA_FLAGS)];
	Driver.TransmitPowerFlag -> Impl.PacketFlag[unique(UQ_METADATA_FLAGS)];
	Driver.RSSIFlag -> Impl.PacketFlag[unique(UQ_METADATA_FLAGS)];
	Driver.RSSIIdleFlag -> Impl.PacketFlag[unique(UQ_METADATA_FLAGS)];
	Driver.TimeSyncFlag -> Impl.PacketFlag[unique(UQ_METADATA_FLAGS)];
	components new RadioAlarmC();
	RadioAlarmC.Alarm -> Driver;
	Driver.RadioAlarm -> RadioAlarmC.RadioAlarm[unique(UQ_RADIO_ALARM)];
	
    // ------ SoftwareAckLayerC
    components new SoftwareAckLayerC() as AckLayer;
    Impl.SubSend -> AckLayer;
    Impl.SubReceive -> AckLayer;
    PacketAcknowledgements = AckLayer;
    AckLayer.SubSend -> Driver;
    AckLayer.SubReceive -> Driver;
    AckLayer.RadioAlarm -> RadioAlarmC.RadioAlarm[unique(UQ_RADIO_ALARM)];
    AckLayer.Config -> Impl;
    AckLayer.AckReceivedFlag -> Impl.PacketFlag[unique(UQ_METADATA_FLAGS)];
	
	components UartLogC;
	Impl.UartLog -> UartLogC;
}
