// ------------------------------------------------------------
// forward declarations
// ------------------------------------------------------------

uint16_t calcInboundGain(int16_t pre_rss, int16_t post_rss, uint8_t tx_power_level);

// signal map table management functions
void initSignalMap();
int16_t findIdx(am_addr_t nb);
int16_t findEmptyIdx();
void updateSignalMap(am_addr_t nb, int16_t in_gain, int16_t out_gain);
void sortSignalMap(int16_t idx);

// neighbor signal map
void initNbSignalMap();
uint8_t findNbSignalMapIdx(am_addr_t nb);
uint8_t findEmptyNbSignalMapIdx();
void mergeNbSignalMap(signal_map_entry_t *signal_map, sm_footer_t *footer, uint8_t size);
void updateNbSignalMap(am_addr_t nb, sm_footer_t *footer, uint8_t size);

// -----------------------------------------------------------------------------------------
// power conversion
// -----------------------------------------------------------------------------------------
// power for power levels [0 .. 31], in dBm
// based on interpolation here http://mail.millennium.berkeley.edu/pipermail/tinyos-help/2008-February/031386.html
int16_t levelPowerTable[] = {-38, -33, -29, -25, -22, -19, -17, -15, -13, -12, -11, -10, -9, -8, -8, -7, -6, -6, -5, -5, -5, -4, -4, -3, -2, -2, -1, -1, -1, 0, 0, 0};

// convert power level into tx power in dBm
inline async command int16_t SignalMap.level2Power(uint8_t power_level) {
	uint8_t max_power_level = sizeof(levelPowerTable) / sizeof(levelPowerTable[0]) - 1;
	return ((power_level <= max_power_level) ? levelPowerTable[power_level] : levelPowerTable[max_power_level]);
}


// -----------------------------------------------------------------------------------------
//	compute inbound gain
// -----------------------------------------------------------------------------------------
//#ifdef RIDB
// scaled
int16_t noise = INVALID_DBM;
inline async command int16_t SignalMap.getLocalNoise() {
	atomic return noise;
}
//#endif

// TODO: pre_rss, post_rss >= -256 dBm to be scalable w/o overflow
uint16_t calcInboundGain(int16_t pre_rss, int16_t post_rss, uint8_t tx_power_level) {
	uint16_t gain = INVALID_GAIN;
	// scaled
	int16_t tx_signal, rx_signal;

//#ifdef RIDB
	if (post_rss <= MAX_NOISE) {
		atomic {
			if (noise != INVALID_DBM) {
				noise = noise - (noise >> EWMA_R_SHIFT_BIT) + ((post_rss << SCALE_L_SHIFT_BIT) >> EWMA_R_SHIFT_BIT);
			} else {
				noise = (post_rss << SCALE_L_SHIFT_BIT);
			}
		}
	}
//#endif

	// scale to use in dbmDiff
	rx_signal = call Controller.dbmDiffU(pre_rss << SCALE_L_SHIFT_BIT, post_rss << SCALE_L_SHIFT_BIT);
	// scale
	tx_signal = call SignalMap.level2Power(tx_power_level) << SCALE_L_SHIFT_BIT;
	if (tx_signal > rx_signal) {
		gain = tx_signal - rx_signal;
	}
	//call UartLog.logTxRx(DBG_FLAG, neighbor, noise, rssi, rx_signal, tx_signal, 0, neighbor, gain);
	return gain;
}


//-------------------------------------------------------------------------------------
// signal map table management functions
//-------------------------------------------------------------------------------------
// initialize the signal map in the very beginning
void initSignalMap() {
	int16_t i;
	sm_entry_t *se;
	
	for (i = 0; i < SM_SIZE; i++) {
		se = &signalMap[i];
		se->valid = FALSE;
	}
}

// find the index to the entry for a neighbor
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
async command int16_t SignalMap.getSignalMapSize() {
	int16_t i, total;
	sm_entry_t *se;
	
	total = 0;
	for (i = 0; i < SM_SIZE; i++) {
		se = &signalMap[i];
		// ASSUMPTION: entries are consecutive bcoz of FIFO
		if (!se->valid)
			break;
		total++;
	}
	return total;
}

async command void SignalMap.getSignalMapSizeDbg(int16_t *num, int16_t *in_num, int16_t *out_num) {
	int16_t i, total, in_total, out_total;
	sm_entry_t *se;
	
	total = 0;
	in_total = 0;
	out_total = 0;
	for (i = 0; i < SM_SIZE; i++) {
		se = &signalMap[i];
		// ASSUMPTION: entries are consecutive bcoz of FIFO
		if (!se->valid)
			break;
		total++;
		if (INVALID_GAIN == se->inbound_gain)
			in_total++;
		if (INVALID_GAIN == se->outbound_gain)
			out_total++;
	}
	*num = total;
	*in_num = in_total;
	*out_num = out_total;
}


void updateSignalMap(am_addr_t nb, int16_t in_gain, int16_t out_gain) {
	int16_t idx;
	sm_entry_t *se;

	// freeze SM
	atomic {
		if (freezed)
			return;
	}
	/*
	 * update signal map: keep the closest SM_SIZE neighbors, in terms of inbound gain
		 if found
			update
		 else
	 		if exists empty entry
				initialize
			else
				if gain < largest gain in signal map (i.e., the last entry)
					replace last entry
				else
					drop
	 */
	idx = findIdx(nb);
	if (idx < SM_SIZE) {
		se = &signalMap[idx];
		// a new inbound gain sample; sample on if it is valid
		if (in_gain != INVALID_GAIN) {
			if (se->inbound_gain != INVALID_GAIN) {
				se->inbound_gain = se->inbound_gain - (se->inbound_gain >> EWMA_R_SHIFT_BIT) + (in_gain >> EWMA_R_SHIFT_BIT);
			} else {
				se->inbound_gain = in_gain;
			}
		}

		// update outbound gain only if it is valid
		if (out_gain != INVALID_GAIN)
			se->outbound_gain = out_gain;

		sortSignalMap(idx);
	} else {
		// only admit neighbors whose inbound gain is known bcoz 
		// 1) otherwise corrupted neighbors, neighbor w/ id not even in testbed, can crop in, whose inbound gain remains unknown and cause codes requiring inbound gain (e.g., ER update) to halt
		// 2) anyway, admit such neighbors is useless since inbound gain unknown; the only issue is we may miss some initial outbound gain updates and hopefully can catch up w/ more coming later!!
		if (in_gain != INVALID_GAIN) {
			idx = findEmptyIdx();
			if (idx < SM_SIZE) {
				se = &signalMap[idx];
				// initialize
				se->nb = nb;
				se->valid = TRUE;
				// no need for validity check for 1st time
				se->inbound_gain = in_gain;
				se->outbound_gain = out_gain;
				
				sortSignalMap(idx);
			} else {
				// last entry
				idx = SM_SIZE - 1;
				se = &signalMap[idx];
				// "closer": add MIN_GAIN_GAP to prevent oscillation bcoz of gain estimation fluctuation
				if (se->inbound_gain > (in_gain + MIN_GAIN_GAP)) {
					//call UartLog.logTxRx(DBG_FLAG, DBG_SM_FLAG, call SignalMap.getSignalMapSize(), signalMap[SM_SIZE - 2].nb, signalMap[SM_SIZE - 2].inbound_gain, se->nb, se->inbound_gain, nb, in_gain);
					// replace
					se->nb = nb;
					se->valid = TRUE;
					// no need for validity check for 1st time
					se->inbound_gain = in_gain;
					se->outbound_gain = out_gain;

					sortSignalMap(idx);
				}
			}
		}
	}
}

// re-sort signal map by ascending inbound gain after its update
// @param idx: index of the neighbor whose inbound gain changes
void sortSignalMap(int16_t idx) {
	int16_t i, sm_size;
	int16_t new_in_gain;
#ifdef HETER_TX_POWER
	int16_t interference, new_interference;
	uint8_t tx_power_level;
#endif
	sm_entry_t tmp;
	sm_entry_t *se;
	sm_entry_t *localSignalMap;
	
//	uint32_t start_time = call LocalTime.get();
	// local alias for concurrency
	localSignalMap = signalMap;
	tmp = localSignalMap[idx];
	new_in_gain = tmp.inbound_gain;
#ifdef HETER_TX_POWER
	tx_power_level = call Util.getNodeTxPowerLevel(localSignalMap[idx].nb);
	new_interference = (call SignalMap.level2Power(tx_power_level) << SCALE_L_SHIFT_BIT) - new_in_gain;
#endif
	
	// closer
	if (idx > 0) {
	#ifdef HETER_TX_POWER
		tx_power_level = call Util.getNodeTxPowerLevel(localSignalMap[idx - 1].nb);
		interference = (call SignalMap.level2Power(tx_power_level) << SCALE_L_SHIFT_BIT) - localSignalMap[idx - 1].inbound_gain;
		if (new_interference > interference) {
	#else
		if (new_in_gain < localSignalMap[idx - 1].inbound_gain) {		
	#endif
			// moving forward
			for (i = idx; i > 0; i--) {
				se = &localSignalMap[i - 1];
			#ifdef HETER_TX_POWER
				tx_power_level = call Util.getNodeTxPowerLevel(se->nb);
				interference = (call SignalMap.level2Power(tx_power_level) << SCALE_L_SHIFT_BIT) - se->inbound_gain;
				if (new_interference > interference) {
			#else
				if (se->inbound_gain > new_in_gain) {
			#endif
					localSignalMap[i] = localSignalMap[i - 1];
				} else {
					break;
				}
			}
			// found right location
			localSignalMap[i] = tmp;
		}
	}
	
	// further
	sm_size = call SignalMap.getSignalMapSize();
	if ((idx + 1) < sm_size) {
	#ifdef HETER_TX_POWER
		tx_power_level = call Util.getNodeTxPowerLevel(localSignalMap[idx + 1].nb);
		interference = (call SignalMap.level2Power(tx_power_level) << SCALE_L_SHIFT_BIT) - localSignalMap[idx + 1].inbound_gain;
		if (new_interference < interference) {
	#else
		if (new_in_gain > localSignalMap[idx + 1].inbound_gain) {
	#endif
			// moving backward
			for (i = idx; (i + 1) < sm_size; i++) {
				se = &localSignalMap[i + 1];
			#ifdef HETER_TX_POWER
				tx_power_level = call Util.getNodeTxPowerLevel(se->nb);
				interference = (call SignalMap.level2Power(tx_power_level) << SCALE_L_SHIFT_BIT) - se->inbound_gain;
				if (new_interference < interference) {
			#else
				if (se->inbound_gain < new_in_gain) {
			#endif
					localSignalMap[i] = localSignalMap[i + 1];
				} else {
					break;
				}
			}
			// found right location
			localSignalMap[i] = tmp;
		}
	}
//	call UartLog.logEntry(DBG_FLAG, DBG_DELAY_FLAG, __LINE__, call LocalTime.get() - start_time);
	//dbg("SM", "%s after sorting %u\n", __FUNCTION__, idx);
//	call SignalMap.printSignalMap();
//	call UartLog.logTxRx(DBG_FLAG, 	signalMap[0].valid * signalMap[0].nb, signalMap[0].inbound_gain,
//								 	signalMap[1].valid * signalMap[1].nb, signalMap[1].inbound_gain, 
//								 	signalMap[2].valid * signalMap[2].nb, signalMap[2].inbound_gain, 
//								 	signalMap[3].valid * signalMap[3].nb, signalMap[3].inbound_gain * 1000);
}

// ---------------------------------------------------------------------
//	neighbor signal map table management
// ---------------------------------------------------------------------
// initialize in the very beginning
async command void SignalMap.getNbSignalMapSize(uint8_t *num, uint8_t *num0, uint8_t *num1, uint8_t *num2) {
	uint8_t i, j;
	nb_signal_map_entry_t *ne;
	uint8_t total, total0, total1, total2;
	
	total = 0;
	total0 = 0;
	total1 = 0;
	total2 = 0;
	for (i = 0; i < NB_SIGNAL_MAP_SIZE; i++) {
		ne = &nbSignalMap[i];
		if (!ne->valid)
			break;
		total++;
		for (j = 0; j < SM_SIZE; j++) {
			if (INVALID_ADDR == ne->signal_map[j].nb)
				break;
			if (0 == i)	total0++;
			if (1 == i)	total1++;
			if (2 == i)	total2++;
		}
	}
	*num = total;
	*num0 = total0;
	*num1 = total1;
	*num2 = total2;
}

void initNbSignalMap() {
	uint8_t i, j;
	nb_signal_map_entry_t *ne;
	
	for (i = 0; i < NB_SIGNAL_MAP_SIZE; i++) {
		ne = &nbSignalMap[i];
		ne->valid = FALSE;
		
		for (j = 0; j < SM_SIZE; j++) {
			ne->signal_map[j].nb = INVALID_ADDR;
		}
	}
}

// find the index to the entry of a neighbor
uint8_t findNbSignalMapIdx(am_addr_t nb) {
	uint8_t i;
	nb_signal_map_entry_t *ne;
	
	for (i = 0; i < NB_SIGNAL_MAP_SIZE; i++) {
		ne = &nbSignalMap[i];
		if (ne->valid && ne->nb == nb)
			break;
	}
	return i;
}

// find an empty slot in the table
uint8_t findEmptyNbSignalMapIdx() {
	uint8_t i;
	nb_signal_map_entry_t *ne;
	
	for (i = 0; i < NB_SIGNAL_MAP_SIZE; i++) {
		ne = &nbSignalMap[i];
		if (!ne->valid)
			return i;
	}
	return i;
}

// merge existing signal map w/ the new entries in footer
void mergeNbSignalMap(signal_map_entry_t *signal_map, sm_footer_t *footer, uint8_t size) {
	uint8_t i, j;
	bool found;
	sm_footer_t *footer_p;
	signal_map_entry_t *se = NULL;
	
	// FIFO	
	// if found
	//		update
	// else
	// 		if exists empty entry
	//			initialize
	//		else
	//			drop
	
	// each neighbor in footer
	for (i = 0; i < size; i++) {
		footer_p = &footer[i];
		
		found = FALSE;
		for (j = 0; j < SM_SIZE; j++) {
			se = &signal_map[j];
			
			if (INVALID_ADDR == se->nb)
				break;
			if (se->nb == footer_p->nb) {
				found = TRUE;
				se->inbound_gain = footer_p->inbound_gain;
				se->outbound_gain = footer_p->outbound_gain;
				break;
			}
		}
		if (!found && j < SM_SIZE) {
			se->nb = footer_p->nb;
			se->inbound_gain = footer_p->inbound_gain;
			se->outbound_gain = footer_p->outbound_gain;
		}
	}
}

void updateNbSignalMap(am_addr_t nb, sm_footer_t *footer, uint8_t size) {
	uint8_t idx;
	nb_signal_map_entry_t *ne;
	
	// only for active neighbor
	if (!call Util.isActiveLink(my_ll_addr, nb) && !call Util.isActiveLink(nb, my_ll_addr))
		return;
	
	// FIFO	
	// if found
	//		update
	// else
	// 		if exists empty entry
	//			initialize
	//		else
	//			drop
	idx = findNbSignalMapIdx(nb);
	if (idx < NB_SIGNAL_MAP_SIZE) {
		ne = &nbSignalMap[idx];
		mergeNbSignalMap(ne->signal_map, footer, size);
	} else {
		idx = findEmptyNbSignalMapIdx();
		if (idx < NB_SIGNAL_MAP_SIZE) {
			ne = &nbSignalMap[idx];
			ne->nb = nb;
			ne->valid = TRUE;
			mergeNbSignalMap(ne->signal_map, footer, size);
		}
	}
}

// get inbound & outbound gain from node to nb
// called from ControllerisContend
async command error_t SignalMap.getGain(am_addr_t nb, am_addr_t node, int16_t *inbound_gain, int16_t *outbound_gain) {
	uint8_t i, idx;
	signal_map_entry_t *signal_map, *se;
	
	idx = findNbSignalMapIdx(nb);
	if (idx >= NB_SIGNAL_MAP_SIZE)
		return FAIL;
	
	signal_map = nbSignalMap[idx].signal_map;
	for (i = 0; i < SM_SIZE; i++) {
		se = &signal_map[i];
		if (INVALID_ADDR == se->nb)
			break;
		if (se->nb == node) {
			*inbound_gain = se->inbound_gain;
			*outbound_gain = se->outbound_gain;
			return SUCCESS;
		}
	}
	return FAIL;
}

//-------------------------------------------------------------------------------------
// backup
//-------------------------------------------------------------------------------------
// query the gain from the neighbor
// return INVALID_GAIN if neighbor is not found or neighbor found but gain unknown
//command int16_t SignalMap.getInboundGain(am_addr_t nb) {
//	int16_t idx;
//	sm_entry_t *se;
//	
//	idx = findIdx(nb);
//	if (idx < SM_SIZE) {
//		se = &signalMap[idx];
//		return se->inbound_gain;
//	} else {
//		return INVALID_GAIN;
//	}
//}

//// query the gain to the neighbor
//command int16_t SignalMap.getOutboundGain(am_addr_t nb) {
//	int16_t idx;
//	sm_entry_t *se;
//	
//	idx = findIdx(nb);
//	if (idx < SM_SIZE) {
//		se = &signalMap[idx];
//		return se->outbound_gain;
//	} else {
//		return INVALID_GAIN;
//	}
//}


