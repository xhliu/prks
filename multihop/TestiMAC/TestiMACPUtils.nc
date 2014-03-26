
//uint32_t fire_cnt, grant_cnt;
//uint32_t last_time = 0;

//// XL returns -1 if ActiveMessageC.SplitControl.start() not call
//inline int8_t readRssiFast() {
//   int8_t rssi;
//   P4OUT &= ~0x04;      // clear CSN, CS low
//   // write address 0x53  (0x40 for register read, 0x13 for RSSI register address)
//   // XL: msp430 tx buffer
//   U0TXBUF = 0x53;
//   // wait until data has moved from UxTXBUF to the TX shift register
//   // and UxTXBUF is ready for new data. It does not indicate RX/TX completion.
//   // XL: first byte sent
//   while (!(IFG1 & UTXIFG0))
//	 ;
//   U0TXBUF = 0;
//   // XL: second byte sent
//   while (!(IFG1 & UTXIFG0))
//	 ;
//   U0TXBUF = 0;
//   // XL: third byte sent
//   while (!(IFG1 & UTXIFG0))
//	 ;
//   // XL: rx buffer ready
//   while (!(U0TCTL & TXEPT))
//	 ;
//   // XL: msp430 rx buffer	 
//   rssi = U0RXBUF;
//   P4OUT |= 0x04;      // CS high
//   return rssi;
//}

//event void Resource.granted() {
//	uint32_t fire_cnt_, duration, now;
//	int8_t rssi_sample;
//	atomic {
//		fire_cnt_ = fire_cnt;
////		start_time_ = start_time;
//		rssi_sample = readRssiFast();
//	}
//	call Resource.release();
//	now = call Alarm.getNow();
//	duration = now - last_time;
//	last_time = now;
//	if (grant_cnt % 100 == 0) {
//		call UartLog.logTxRx(DBG_FLAG, DBG_HEARTBEAT_FLAG, 0, 0, 0, -rssi_sample - 45, grant_cnt, fire_cnt_, duration);
//	}
//	grant_cnt++;
//}

//async event void Alarm.fired() {
////event void Timer.fired() {
//	//uint32_t i = 0;
//	//call Leds.led0Toggle();
//	atomic fire_cnt++;
//	call Resource.request();
//	call Alarm.start(32);
//}

//// convert power in dBm to power level; if there are multiple levels using the power, return the min
//// round up
//inline uint8_t power2LevelCeil(int16_t power) {
//	uint8_t i;
//	
//	for (i = 0; i < sizeof(levelPowerTable) / sizeof(levelPowerTable[0]); i++) {
//		if (power <= levelPowerTable[i])
//			return i;
//	}
//	// power > max power of level 31
//	return (i - 1);
//}
//// round down
//inline uint8_t power2LevelFloor(int16_t power) {
//	uint8_t i;
//	
//	for (i = 0; i < sizeof(levelPowerTable) / sizeof(levelPowerTable[0]); i++) {
//		if (power < levelPowerTable[i])
//			if (i > 0)
//				return (i - 1);
//			else
//				return 0;
//		if (power == levelPowerTable[i])
//			return i;
//	}
//	return (i - 1);
//}
//// TODO: debug
//enum {
//	MIN_DBM = 0x8000,
//	SCALE_L_SHIFT_BIT = 7,
//};

//// =========================================================================================
//// power arithmic based on dBm
//// functions ending in U means only dealing w/ positive power, S w/ negative power as well
//// =========================================================================================
//// pre-computed offset table; scaled; start from 1
//int16_t diffDeltaTable[] = {879, 554, 387, 282, 211, 161, 124, 96, 75, 59, 46, 36, 29, 23, 18, 14, 11, 9, 7, 6, 4, 4, 3, 2, 2, 1, 1, 1, 1, 1};
//// z = x - y; all scaled in dBm
//// assert((x - y) >=  (0x1 << SCALE_L_SHIFT_BIT))
//inline int16_t dbmDiffU(int16_t x, int16_t y) {
//	int16_t n, delta;
//	
//	// start from 1
//	n = ((x - y) >> SCALE_L_SHIFT_BIT) - 1;
//	if (-1 == n)
//		// NaN if x == y; use infitesimally small mW to represent 0 mW
//		return MIN_DBM;
//	if (n < sizeof(diffDeltaTable) / sizeof(diffDeltaTable[0])) {
//		delta = diffDeltaTable[n];
//	} else {
//		delta = 0;
//	}
//	return (x - delta);
//}

//// start from 0
//int16_t sumDeltaTable[] = {385, 325, 272, 226, 186, 153, 125, 101, 82, 66, 53, 42, 34, 27, 22, 17, 14, 11, 9, 7, 6, 4, 3, 3, 2, 2, 1, 1, 1, 1};
//// z = x + y; all scaled in dBm
//inline int16_t dbmSumU(int16_t x, int16_t y) {
//	int16_t n, delta, tmp;
//	
//	// swap
//	if (x < y) {
//		tmp = y;
//		y = x;
//		x = tmp;
//	}
//	
//	// assert(x >= y)
//	// start from 0
//	n = x - y;
//	n >>= SCALE_L_SHIFT_BIT;
//	if (n < sizeof(sumDeltaTable) / sizeof(sumDeltaTable[0])) {
//		delta = sumDeltaTable[n];
//	} else {
//		delta = 0;
//	}
//	return (x + delta);
//}

//// compute z = x + y
//// can be negative power
//dbm_t dbmSumS(dbm_t x, dbm_t y) {
//	dbm_t z;
//	
//	// plus 0
//	if (0 == x.sign)
//		return y;
//	if (0 == y.sign)
//		return x;
//	
//	if (x.sign == y.sign) {
//		z.sign = x.sign;
//		z.abs = dbmSumU(x.abs, y.abs);
//	} else {
//		if (x.abs >= (y.abs + (0x1 << SCALE_L_SHIFT_BIT))) {
//			z.sign = x.sign;
//			z.abs = dbmDiffU(x.abs, y.abs);
//		} else if (y.abs >= (x.abs + (0x1 << SCALE_L_SHIFT_BIT))) {
//			z.sign = y.sign;
//			z.abs = dbmDiffU(y.abs, x.abs);
//		} else {
//			// (x.abs == y.abs) in unit not scaled
//			z.sign = 0;
//			z.abs = 0;
//		}
//	}
//	return z;	
//}

//// compute z = x - y
//// z = x + (-y)
//dbm_t dbmDiffS(dbm_t x, dbm_t y) {
//	y.sign *= -1;
//	return dbmSumS(x, y);	
//}

//// compute z = c * x + (1 - c) * y
//// z = x' + y'; then x' = x + 10 * lg(c), y' = y + 10 * lg(1 - c)
//// c = 0.9
//dbm_t dbmWeightedSumS(dbm_t x, dbm_t y) {
//	dbm_t x_, y_;
//	
//	x_.sign = x.sign;
//	x_.abs = x.abs - 59;
//	y_.sign = y.sign;
//	y_.abs = y.abs - 1280;
//	return dbmSumS(x_, y_);
//}
