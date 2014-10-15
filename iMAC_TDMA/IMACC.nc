/*
 * @ author: Xiaohui Liu (whulxh@gmail.com)
 * 
 * @ updated: 	06/27/2012 
 				04/07/2012 03:23:55 PM
 				07/02/2012  
 * @ description: wiring for iMAC
 */
configuration IMACC {
 	provides {
		interface AsyncAMSend as AMSend;
		interface AsyncReceive as Receive;
		interface AsyncPacket as Packet;

 		interface AsyncSplitControl as ForwarderSwitch;
 		interface AsyncStdControl as ControllerSwitch;
	#if defined(VARY_PERIOD)
		interface ForwarderInfo;
	#endif
 	};
}

implementation {
	components IMACForwarderC as Forwarder;
	AMSend = Forwarder;
	Receive = Forwarder;
	Packet = Forwarder;
	ForwarderSwitch = Forwarder;
#if defined(VARY_PERIOD)	
	ForwarderInfo = Forwarder;
#endif	
	components IMACControllerC as Controller;
	ControllerSwitch = Controller;
}
