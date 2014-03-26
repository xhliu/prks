#include "Commander.h"

configuration CommanderC {}
implementation {
  components MainC, CommanderP as App;    // UartLogC;
  //components TimeSyncMessageC;
  components LocalTimeMilliC;
  components new TimerMilliC();
  components UartLogC;
  
  App.Boot -> MainC.Boot;
  
  //App.AMSend -> TimeSyncMessageC.TimeSyncAMSendMilli[AM_TYPE_SYNC];
  //App.AMControl -> TimeSyncMessageC;
  components new AMSenderC(AM_TYPE_SYNC);
  components ActiveMessageC;
  App.AMSend -> AMSenderC;
  App.AMControl -> ActiveMessageC;
  
  App.LocalTime -> LocalTimeMilliC;
  
  App.MilliTimer -> TimerMilliC;
  
  App.UartLog -> UartLogC;
}


