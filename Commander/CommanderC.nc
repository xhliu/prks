#include "Commander.h"

configuration CommanderC {}
implementation {
  components MainC, CommanderP as App;    // UartLogC;
  components new AMSenderC(AM_TYPE_SYNC);
  components new TimerMilliC();
  components ActiveMessageC;
  components UartLogC;
  
  App.Boot -> MainC.Boot;
  
  App.AMSend -> AMSenderC;
  App.AMControl -> ActiveMessageC;
  
  App.MilliTimer -> TimerMilliC;
  
  App.UartLog -> UartLogC;
}


