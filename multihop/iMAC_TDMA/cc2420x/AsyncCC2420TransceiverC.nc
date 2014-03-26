/* *
 * @ author:    Xiaohui Liu (whulxh@gmail.com) 
 * @ updated:   12/31/12 12:10 PM
 * @ description: migrate to CC2420X stack for time sync to work
 */
#include <RadioConfig.h>

configuration AsyncCC2420TransceiverC {
	provides {
		interface SplitControl;
		interface AsyncAMSend as Send[am_id_t id];
		interface AsyncReceive as Receive[am_id_t id];
		interface AsyncReceive as Snoop[am_id_t id];
		interface AsyncPacket as Packet;
		interface AsyncAMPacket as AMPacket;
		interface PacketTimeStamp<TRadio, uint32_t> as PacketTimeStampRadio;
		interface CC2420Packet;
		
	#if defined(ACK_LAYER)
		interface PacketAcknowledgements;
	#endif
	}
}

implementation {
	components AsyncCC2420TransceiverP as ImplP;
	SplitControl = ImplP;
	Send = ImplP;
	Receive = ImplP.Receive;
	Snoop = ImplP.Snoop;
	Packet = ImplP;
	AMPacket = ImplP;
	PacketTimeStampRadio = ImplP;
	CC2420Packet = ImplP;
	
	components MainC;
	MainC.SoftwareInit -> ImplP;
	
	components CC2420XDriverLayerC as Driver;
#if !defined(ACK_LAYER)
	#warning ACK_LAYER disabled
	ImplP.SubSend -> Driver;
	ImplP.SubReceive -> Driver;
#endif
	ImplP.SubPacket -> Driver;
	ImplP.RadioState -> Driver;
	
#if defined(ACK_LAYER)
	components new SoftwareAckLayerC() as AckLayer;
	ImplP.SubSend -> AckLayer;
	ImplP.SubReceive -> AckLayer;
#endif	
	components ActiveMessageAddressC;
	ImplP.ActiveMessageAddress -> ActiveMessageAddressC;
	components new Ieee154PacketLayerC();
	ImplP.Ieee154PacketLayer -> Ieee154PacketLayerC;
	Ieee154PacketLayerC.SubPacket -> Driver;
	
	components new SimpleFcfsArbiterC(RADIO_SEND_RESOURCE) as SendResourceC;
	ImplP.Resource -> SendResourceC.Resource[unique(RADIO_SEND_RESOURCE)];

	ImplP.PacketRSSI -> Driver.PacketRSSI;
	ImplP.PacketRSSIIdle -> Driver.PacketRSSIIdle;

	// ------- Driver
#define UQ_METADATA_FLAGS	"UQ_CC2420X_METADATA_FLAGS"
#define UQ_RADIO_ALARM		"UQ_CC2420X_RADIO_ALARM"
	Driver.Config -> ImplP;
	Driver.PacketTimeStamp -> ImplP;
	ImplP.TimeStampFlag -> ImplP.PacketFlag[unique(UQ_METADATA_FLAGS)];
	Driver.TransmitPowerFlag -> ImplP.PacketFlag[unique(UQ_METADATA_FLAGS)];
	Driver.RSSIFlag -> ImplP.PacketFlag[unique(UQ_METADATA_FLAGS)];
	Driver.RSSIIdleFlag -> ImplP.PacketFlag[unique(UQ_METADATA_FLAGS)];
	Driver.TimeSyncFlag -> ImplP.PacketFlag[unique(UQ_METADATA_FLAGS)];
	components new RadioAlarmC();
	RadioAlarmC.Alarm -> Driver;
	Driver.RadioAlarm -> RadioAlarmC.RadioAlarm[unique(UQ_RADIO_ALARM)];
	
#if defined(ACK_LAYER)
	// ------ SoftwareAckLayerC
	PacketAcknowledgements = AckLayer;
	AckLayer.SubSend -> Driver;
	AckLayer.SubReceive -> Driver;
	AckLayer.RadioAlarm -> RadioAlarmC.RadioAlarm[unique(UQ_RADIO_ALARM)];
	AckLayer.Config -> ImplP;
	AckLayer.AckReceivedFlag -> ImplP.PacketFlag[unique(UQ_METADATA_FLAGS)];
#endif
	
	components UartLogC;
	ImplP.UartLog -> UartLogC;
}
