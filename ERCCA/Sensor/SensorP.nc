/*
 * @ author: Xiaohui Liu (whulxh@gmail.com)
 * 
 * @ updated: 05/22/2012 
 */

 
#include "Sensor.h"

module SensorP {
	uses {
		interface Boot;
//		interface Timer<TMilli>;
		interface LocalTime<TMilli>;
		interface LocalTime<TMicro> as LocalTimeMicro;
		//interface Read<uint16_t> as ReadRssi;
		interface RssiRead;
//		interface AMSend;
//		interface Receive;
		interface SplitControl;
		interface Resource;
		interface UartLog;
		interface Leds;
		
		interface Alarm<T32khz, uint32_t>;
	}
}
implementation {
inline int8_t readRssiFast();

bool started = FALSE;
uint32_t start_time, start_time_mu, last_time;
uint32_t interval, interval_mu, max_interval, max_interval_mu;
uint32_t fire_cnt, grant_cnt;

nx_uint8_t samples[SAMPLE_CNT];
uint8_t sample_idx = 0;

event void Resource.granted() {
	uint32_t fire_cnt_, start_time_, duration, now;
	int8_t rssi_sample;
	atomic {
		fire_cnt_ = fire_cnt;
		start_time_ = start_time;
		rssi_sample = readRssiFast();
	}
	call Resource.release();
	now = call Alarm.getNow();
	duration = now - last_time;
	last_time = now;
	samples[sample_idx++] = -rssi_sample;
	// full
	if (sample_idx == SAMPLE_CNT) {
		sample_idx = 0;
		call UartLog.logByteStream(samples, SAMPLE_CNT);
//		call UartLog.logTxRx(DBG_FLAG, DBG_HEARTBEAT_FLAG, 0, 0, 0, -rssi_sample, grant_cnt, fire_cnt_, duration);
	}
	grant_cnt++;
	call Resource.request();
//	int8_t rssi = 0;
////	error_t ret = SUCCESS;
////	uint16_t val = 0;
//	//call Leds.led0Toggle();
//	atomic {
//		rssi = readRssiFast();
//	}
//	//ret = call RssiRead.read(&val);
//	call Resource.release();
//	interval = call LocalTime.get() - start_time;
//	interval_mu = call LocalTimeMicro.get() - start_time_mu;
//	if (max_interval <  interval)
//		max_interval =  interval;
//	if (max_interval_mu <  interval_mu)
//		max_interval_mu =  interval_mu;
//	// request ASAP
//	start_time = call LocalTime.get();
//	start_time_mu = call LocalTimeMicro.get();
//	call Resource.request();
}
int8_t rssi_sample = 0;

task void logTask() {
//	int8_t rssi_sample_;
//	//error_t ret;
//	uint32_t interval_, seq_, start_time_, i;
//	//uint32_t interval1 = 0;
//	uint32_t interval2 = 0;
//	//uint32_t interval3 = 0;		
//	atomic {
//		rssi_sample_ = rssi_sample;
//		interval_ = interval;
//		seq_ = seq;
//		start_time_ = start_time;
//	}
////	start_time_ = call Alarm.getNow();
////	ret = call Resource.immediateRequest();
////	if (SUCCESS == ret) {
////		interval1 = call Alarm.getNow() - start_time_;
////		start_time_ = call Alarm.getNow();
//		atomic {
//			rssi_sample = readRssiFast();
//		}
////		interval2 = call Alarm.getNow() - start_time_;
////		start_time_ = call Alarm.getNow();
////		call Resource.release();
////		interval3 = call Alarm.getNow() - start_time_;
////	}
//	call UartLog.logTxRx(DBG_FLAG, DBG_HEARTBEAT_FLAG, 0, 0, DIVS0, -rssi_sample_, interval1, interval2, interval3);
//	post logTask();
	call UartLog.logEntry(DBG_FLAG, DBG_HEARTBEAT_FLAG, 0, 0);
}

async event void Alarm.fired() {
//event void Timer.fired() {
	//uint32_t i = 0;
	//call Leds.led0Toggle();
	atomic fire_cnt++;
	atomic start_time = call Alarm.getNow();
//	call Resource.request();
	if (fire_cnt < 100) {
		call Alarm.start(PERIOD);
		post logTask();
	} else {
		call Resource.request();
	}

	//if (seq++ < 3000U)
		//call Timer.startOneShot(10);
//	error_t ret = SUCCESS;
//	uint16_t val = 0;
	//while (TRUE) {
//	for (i = 0; i < 6000; i++) {
//		start_time = call LocalTime.get();
//		start_time_mu = call LocalTimeMicro.get();
//		if (SUCCESS == call Resource.immediateRequest()) {
//			atomic {
//				rssi_sample = readRssiFast();
//			}
//			call Resource.release();
//			interval = call LocalTime.get() - start_time;
//			interval_mu = call LocalTimeMicro.get() - start_time_mu;
//			atomic interval = call Alarm.getNow() - start_time;
//			atomic start_time = call Alarm.getNow();
//			post logTask();
//		}
//		call UartLog.logEntry(DBG_FLAG, DBG_HEARTBEAT_FLAG, 0, seq++);
		//call Alarm.start(PERIOD);
//	}
//	if (!started) {
//		started = TRUE;
//		start_time = call LocalTime.get();
//		start_time_mu = call LocalTimeMicro.get();
//		call Resource.request();
//	} else {
//		call UartLog.logTxRx(DBG_FLAG, DBG_HEARTBEAT_FLAG, 0, interval, interval_mu, start_time, start_time_mu, max_interval, max_interval_mu);
//	}
//	int8_t rssi;
//	uint32_t start_time, start_time_mu;
//	error_t ret;
//	uint16_t val;
//	uint32_t my_interval, my_interval_mu;
//	uint32_t hauer_interval, hauer_interval_mu;
//	
//	atomic {
//		rssi = readRssiFast();
//	}
//	hauer_interval_mu = call LocalTimeMicro.get() - start_time_mu;	
//	hauer_interval = call LocalTime.get() - start_time;
	
//	start_time = call LocalTime.get();
//	start_time_mu = call LocalTimeMicro.get();
//	ret = call RssiRead.read(&val);
//	my_interval_mu = call LocalTimeMicro.get() - start_time_mu;
//	my_interval = call LocalTime.get() - start_time;
//	
	//call UartLog.logEntry(255, hauer_interval, hauer_interval_mu, rssi);
	//call UartLog.logEntry(ret, my_interval, val, my_interval_mu);
//	call UartLog.logTxRx(255, rssi == ((int16_t)val - 127), rssi, hauer_interval, hauer_interval_mu, ret, my_interval, my_interval_mu, val);
}

event void Boot.booted() {
	max_interval = 0;
	max_interval_mu = 0;
	atomic fire_cnt = 0;
	grant_cnt = 0;
	last_time = 0;
	call SplitControl.start();
}

event void SplitControl.startDone(error_t err) {
	if (err == SUCCESS) {
		//call Timer.startPeriodic(PERIOD);
		call Alarm.start(PERIOD);
		//atomic start_time = call Alarm.getNow();
		//post logTask();
	} else {
		call SplitControl.start();
	}
}

event void SplitControl.stopDone(error_t err) {}

//event void AMSend.sendDone(message_t *m, error_t error) {

//}

//event message_t *Receive.receive(message_t *m, void *payload, uint8_t len) {
//	call UartLog.logEntry(DBG_FLAG, DBG_RX_FLAG, 0, 0);
//	return m;
//}

// XL returns -1 if ActiveMessageC.SplitControl.start() not call
inline int8_t readRssiFast() {
   int8_t rssi;
   P4OUT &= ~0x04;      // clear CSN, CS low
   // write address 0x53  (0x40 for register read, 0x13 for RSSI register address)
   // XL: msp430 tx buffer
   U0TXBUF = 0x53;
   // wait until data has moved from UxTXBUF to the TX shift register
   // and UxTXBUF is ready for new data. It does not indicate RX/TX completion.
   // XL: first byte sent
   while (!(IFG1 & UTXIFG0))
	 ;
   U0TXBUF = 0;
   // XL: second byte sent
   while (!(IFG1 & UTXIFG0))
	 ;
   U0TXBUF = 0;
   // XL: third byte sent
   while (!(IFG1 & UTXIFG0))
	 ;
   // XL: rx buffer ready
   while (!(U0TCTL & TXEPT))
	 ;
   // XL: msp430 rx buffer	 
   rssi = U0RXBUF;
   P4OUT |= 0x04;      // CS high
   return rssi;
}
}
