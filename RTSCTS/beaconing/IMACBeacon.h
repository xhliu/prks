/*
 * @ author: Xiaohui Liu (whulxh@gmail.com)
 * 
 * @ updated: 04/05/2012 08:56:00 PM 
 */
#ifndef IMAC_BEACON_H
#define IMAC_BEACON_H

enum {
	BEACON_PERIOD_SHORT = 50,	//5000
	BEACON_PERIOD_LONG = 600000,
	BEACON_PERIOD_SHORT_CNT = 1,	//60
	// use max to build signal map for the largest ER possible
	BEACON_SM_POWER_LEVEL = 31,
};

typedef nx_struct {
	nx_uint16_t seq;
} beacon_header_t;

#endif
