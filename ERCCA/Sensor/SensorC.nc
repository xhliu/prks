/*
 * @ author: Xiaohui Liu (whulxh@gmail.com)
 * 
 * @ updated: 05/22/2012 
 */
 
#include "Sensor.h"

configuration SensorC {}
implementation {
	components MainC, SensorP as App;
	App.Boot -> MainC.Boot;
	
//	components new TimerMilliC();
//	App.Timer -> TimerMilliC;
	
	components LocalTimeMilliC;
	App.LocalTime -> LocalTimeMilliC;
	components LocalTimeMicroC;
	App.LocalTimeMicro -> LocalTimeMicroC;
	
	components CC2420ControlC;
	App.RssiRead -> CC2420ControlC;

	components UartLogC;
	App.UartLog -> UartLogC;
	
//	components new AMSenderC(AM_TYPE_SYNC);
//	components new AMReceiverC(AM_TYPE_SYNC);
	components ActiveMessageC;
//	App.AMSend -> AMSenderC;
//	App.Receive -> AMReceiverC;
	App.SplitControl -> ActiveMessageC;
	
	components LedsC;
	App.Leds -> LedsC;
	
	components new CC2420SpiC() as Spi;
	App.Resource -> Spi;
	
	components new Alarm32khz32C() as Alarm;
	App.Alarm -> Alarm;
}
