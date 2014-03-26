/*
 * @ author: Xiaohui Liu (whulxh@gmail.com)
 * 
 * @ updated: 04/05/2012 08:56:00 PM 
 * @ description: beaconing for update signal map and link estimator; use max power to cover largest area
 */
#include "IMACBeacon.h"

module IMACBeaconP {
	provides {
		interface Init;
	}
	uses {
		interface AMSend as BeaconSend;
		interface Receive as BeaconReceive;
#ifndef TOSSIM		
		interface CC2420Packet;
#endif
		interface Timer<TMilli> as BeaconTimer;
	}
}
implementation {

message_t m_beacon;
message_t *m_beacon_p;

bool sending;
uint16_t beacon_seq;
uint32_t period;

command error_t Init.init() {
	sending = FALSE;
	beacon_seq = 0;
	m_beacon_p = &m_beacon;
	period = BEACON_PERIOD_SHORT;
	call BeaconTimer.startOneShot(period);
	return SUCCESS;
}

task void sendBeaconTask() {
	error_t ret;
	beacon_header_t *hdr;
	
	if (beacon_seq++ < BEACON_PERIOD_SHORT_CNT)
		// stop
		call BeaconTimer.startOneShot(period);

		// slow down after initial sampling
		//period = BEACON_PERIOD_LONG;
	if (sending)
		return;
	
	hdr = (beacon_header_t*) call BeaconSend.getPayload(m_beacon_p, sizeof(beacon_header_t));
	hdr->seq = beacon_seq;
#ifndef TOSSIM
	call CC2420Packet.setPower(m_beacon_p, BEACON_SM_POWER_LEVEL);
#endif
	ret = call BeaconSend.send(AM_BROADCAST_ADDR, m_beacon_p, sizeof(beacon_header_t));
	if (SUCCESS == ret) {
		sending = TRUE;
	}
}

event void BeaconSend.sendDone(message_t *msg, error_t error) {
	sending = FALSE;
	dbg("beacon", "%s: sending %hu\n", __FUNCTION__, ((beacon_header_t*) call BeaconSend.getPayload(msg, sizeof(beacon_header_t)))->seq);
}
 
event void BeaconTimer.fired() {
	post sendBeaconTask();
}

// just for debug
event message_t *BeaconReceive.receive(message_t *msg, void *payload, uint8_t len) {
//	beacon_header_t *hdr;
//	hdr = (beacon_header_t*) call BeaconSend.getPayload(msg, len);
	dbg("beacon", "%s: receiving %hu\n", __FUNCTION__, hdr->seq);
	return msg;
}

}
