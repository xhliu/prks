/*
 * @ author: Xiaohui Liu (whulxh@gmail.com)
 * 
 * @ date: 05/22/2012 
 */

#ifndef SENSOR_H
#define SENSOR_H

// RssiRead interface
#define SIGNAL_MAP

enum {
	AM_TYPE_SYNC = 12,
	
	PERIOD = 320,
	
	SAMPLE_CNT = 22,
	
	DBG_FLAG = 255,
	
	DBG_RSSI_FLAG = 0,
	DBG_HEARTBEAT_FLAG = 1,
	DBG_RX_FLAG = 2,
};

#endif
