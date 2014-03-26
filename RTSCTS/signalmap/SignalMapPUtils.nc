// power for power levels [0 .. 31], in dBm
int16_t levelPowerTable[] = {-38, -33, -29, -25, -22, -19, -17, -15, -13, -12, -11, -10, -9, -8, -8, -7, -6, -6, -5, -5, -5, -4, -4, -3, -2, -2, -1, -1, -1, 0, 0, 0};

// convert power in dBm to power level; round up; if there are multiple levels using the power, return the min
inline uint8_t power2Level(int16_t power) {
	uint8_t i;
	
	for (i = 0; i < sizeof(levelPowerTable) / sizeof(levelPowerTable[0]); i++) {
		if (power <= levelPowerTable[i])
			return i;
	}
	return i;
}

// convert power level into tx power in dBm
inline int16_t level2Power(uint8_t power_level) {
	uint8_t max_power_level = sizeof(levelPowerTable) / sizeof(levelPowerTable[0]) - 1;
	return ((power_level <= max_power_level) ? levelPowerTable[power_level] : levelPowerTable[max_power_level]);
}


// =========================================================================================
// power arithmic based on dBm
// functions ending in U means only dealing w/ positive power, S w/ negative power as well
// =========================================================================================
// pre-computed offset table; scaled; start from 1
int16_t diffDeltaTable[] = {879, 554, 387, 282, 211, 161, 124, 96, 75, 59, 46, 36, 29, 23, 18, 14, 11, 9, 7, 6, 4, 4, 3, 2, 2, 1, 1, 1, 1, 1};
// z = x - y; all scaled in dBm
// assert((x - y) >=  (0x1 << SCALE_L_SHIFT_BIT))
inline int16_t dbmDiffU(int16_t x, int16_t y) {
	int16_t n, delta;
	
	// start from 1
	n = ((x - y) >> SCALE_L_SHIFT_BIT) - 1;
	if (-1 == n)
		// NaN if x == y; use infitesimally small mW to represent 0 mW
		return MIN_DBM;
	if (n < sizeof(diffDeltaTable) / sizeof(diffDeltaTable[0])) {
		delta = diffDeltaTable[n];
	} else {
		delta = 0;
	}
	return (x - delta);
}

// start from 0
int16_t sumDeltaTable[] = {385, 325, 272, 226, 186, 153, 125, 101, 82, 66, 53, 42, 34, 27, 22, 17, 14, 11, 9, 7, 6, 4, 3, 3, 2, 2, 1, 1, 1, 1};
// z = x + y; all scaled in dBm
inline int16_t dbmSumU(int16_t x, int16_t y) {
	int16_t n, delta, tmp;
	
	// swap
	if (x < y) {
		tmp = y;
		y = x;
		x = tmp;
	}
	
	// assert(x >= y)
	// start from 0
	n = x - y;
	n >>= SCALE_L_SHIFT_BIT;
	if (n < sizeof(sumDeltaTable) / sizeof(sumDeltaTable[0])) {
		delta = sumDeltaTable[n];
	} else {
		delta = 0;
	}
	return (x + delta);
}

// compute z = x + y
// can be negative power
dbm_t dbmSumS(dbm_t x, dbm_t y) {
	dbm_t z;
	
	// plus 0
	if (0 == x.sign)
		return y;
	if (0 == y.sign)
		return x;
	
	if (x.sign == y.sign) {
		z.sign = x.sign;
		z.abs = dbmSumU(x.abs, y.abs);
	} else {
		if (x.abs >= (y.abs + (0x1 << SCALE_L_SHIFT_BIT))) {
			z.sign = x.sign;
			z.abs = dbmDiffU(x.abs, y.abs);
		} else if (y.abs >= (x.abs + (0x1 << SCALE_L_SHIFT_BIT))) {
			z.sign = y.sign;
			z.abs = dbmDiffU(y.abs, x.abs);
		} else {
			// (x.abs == y.abs) in unit not scaled
			z.sign = 0;
			z.abs = 0;
		}
	}
	return z;	
}

// compute z = x - y
// z = x + (-y)
dbm_t dbmDiffS(dbm_t x, dbm_t y) {
	y.sign *= -1;
	return dbmSumS(x, y);	
}

// compute z = c * x + (1 - c) * y
// z = x' + y'; then x' = x + 10 * lg(c), y' = y + 10 * lg(1 - c)
// c = 0.9
dbm_t dbmWeightedSumS(dbm_t x, dbm_t y) {
	dbm_t x_, y_;
	
	x_.sign = x.sign;
	x_.abs = x.abs - 59;
	y_.sign = y.sign;
	y_.abs = y.abs - 1280;
	return dbmSumS(x_, y_);
}




// compute inbound gain and sample local (interference+noise)
uint16_t calcInboundGain(int16_t pre_rss, int16_t post_rss, uint8_t tx_power_level) {
	uint16_t gain = INVALID_GAIN;
	// scaled
	int16_t tx_signal, rx_signal;

	// scale to use in dbmDiff
	rx_signal = dbmDiffU(pre_rss << SCALE_L_SHIFT_BIT, post_rss << SCALE_L_SHIFT_BIT);	
	// scale
	tx_signal = level2Power(tx_power_level) << SCALE_L_SHIFT_BIT;
	if (tx_signal > rx_signal) {
		gain = tx_signal - rx_signal;
	}
	//call UartLog.logTxRx(DBG_FLAG, neighbor, noise, rssi, rx_signal, tx_signal, 0, neighbor, gain);
	return gain;
}


// ===========================================
// signal map table management functions
// ===========================================
// initialize the signal map in the very beginning
void initSignalMap() {
	int16_t i;
	sm_entry_t *se;
	
	for (i = 0; i < SM_SIZE; i++) {
		se = &signalMap[i];
		se->valid = FALSE;
	}
}

// find the index to the entry for neighbor ll_addr
int16_t findIdx(am_addr_t nb) {
	int16_t i;
	sm_entry_t *se;
	
	for (i = 0; i < SM_SIZE; i++) {
		se = &signalMap[i];
		if (se->valid && se->nb == nb) {
			break;
		}
	}
	return i;
}

// find an empty slot in the neighbor table
int16_t findEmptyIdx() {
	int16_t i;
	sm_entry_t *se;
	
	for (i = 0; i < SM_SIZE; i++) {
		se = &signalMap[i];
		if (!se->valid)
			return i;
	}
	return i;
}

// # of active entries in signal map
int16_t getSignalMapSize() {
	int16_t i, total;
	sm_entry_t *se;
	
	total = 0;
	for (i = 0; i < SM_SIZE; i++) {
		se = &signalMap[i];
		if (se->valid)
			total++;
	}
	return total;
}

// find the maximal ER: [0 .. max]
int16_t maxERBorderIdx() {
	int16_t i, max;
	sm_entry_t *se;
	
	max = -1;
	for (i = 0; i < SM_SIZE; i++) {
		se = &signalMap[i];
		if (se->valid) {
			if (max < se->tx_er_border_idx)
				max = se->tx_er_border_idx;
			if (max < se->rx_er_border_idx)
				max = se->rx_er_border_idx;
		}
	}
	return max;
}
