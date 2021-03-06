/*
 * TODO: standardize
 * 	link_er_table_entry_t *le;
	local_link_er_table_entry_t *se;
 */
// ---------------------------------------------------------------------
// forward declarations
// ---------------------------------------------------------------------
// local link ER table
void initLocalLinkERTable();
void setLocalLinkERTable();
uint8_t findLocalLinkERTableIdx(am_addr_t nb, bool is_sender);

// neighbor link ER table: store all links
void initLinkERTable();
//uint8_t findLinkERTableIdx(am_addr_t sender, am_addr_t receiver);
uint8_t findEmptyLinkERTableIdx();
uint8_t findFarthestLinkERTableIdx(int16_t *link_in_gain);
void updateLinkPdr(am_addr_t nb, local_link_pdr_footer_t *footer, uint8_t size);
//void updateLinkERTable(link_er_footer_t *footer, uint8_t size, am_addr_t nb, uint16_t seqno);
//uint8_t getLinkERTableSize();
void sortLinkERTableIdx(uint8_t idx);
void sortLinkERTable();
void updateLinkERTableEntry(am_addr_t sender, am_addr_t receiver, am_addr_t from, int16_t rx_interference_threshold, uint8_t rx_er_version, bool is_local_er);
// update contention relationship upon newer ER arrival
//task void updateContentionTask();
void updateContentionTable();
void updateContention(uint8_t link_idx);
inline void initLinkERTableEntry(uint8_t idx, am_addr_t sender, am_addr_t receiver, int16_t rx_interference_threshold, uint8_t rx_er_version, bool is_local_er);
bool amInLinkER(uint8_t idx);
bool isContend(uint8_t local_idx, uint8_t link_idx);

// controller
error_t udpateER(bool is_sender, local_link_er_table_entry_t *le, int16_t gain, int16_t delta);
// wrapper
inline void updateBorder(bool is_tx, local_link_er_table_entry_t *le, int16_t i);
// based on deltaI, adjust ER
error_t adjustER(int16_t idx, bool is_tx, int32_t delta_i_dB);

// new pdr sample comes, execute the controller
error_t execController(am_addr_t nb, bool is_sender);


static long isqrt(long num);
// power arithmic
inline int32_t dbmSumU(int32_t x, int32_t y);
dbm_t dbmSumS(dbm_t x, dbm_t y);
dbm_t dbmDiffS(dbm_t x, dbm_t y);
dbm_t dbmWeightedSumS(dbm_t x, dbm_t y);


// ---------------------------------------------------------------------
//	local link table management
// ---------------------------------------------------------------------
// does not require packet exchange to populate the table
void initLocalLinkERTable() {
	uint8_t i, j;
	bool is_sender, is_receiver;
	am_addr_t nb;
	local_link_er_table_entry_t *se;
	
	j = 0;
	for (i = 0; i < active_link_size; i++) {
		is_sender = (activeLinks[i].sender == my_ll_addr);
		is_receiver = (activeLinks[i].receiver == my_ll_addr);
		if (!is_sender && !is_receiver)
			continue;
		// the other end of the link
		nb = is_sender ? activeLinks[i].receiver : activeLinks[i].sender;
		
		se = &localLinkERTable[j];
		se->nb = nb;
		se->valid = TRUE;
		se->link_idx = i;

		se->is_sender = is_sender;
		se->next_slot_by_tx = INVALID_SLOT;
		se->next_slot_by_rx = INVALID_SLOT;
		// jump start; if initialized as FALSE, receiver never switch to DATA channel if next_slot_by_tx delivered by ctrl pkt always late
		se->is_rx_pending = !is_sender;

//		// initialize ER w/ max, not empty, ER
//		// INVALID_DBM corresponds to empty ER since it's the upper bound of tx power and thus interference
//		se->rx_interference_threshold = INVALID_DBM;
//		se->rx_er_version = 0;
//		se->rx_er_border_idx = EMPTY_ER_IDX;
//		// only for receiver
//		if (is_receiver)
//			updateLinkERTableEntry(se->nb, my_ll_addr, my_ll_addr, se->rx_interference_threshold, se->rx_er_version, TRUE);
		
		// min var controller
		se->rx_nI.sign = 0;
		se->rx_nI.abs = INVALID_DBM;
		
	#ifdef FILTERED_PID
		se->sum_e = 0;
		se->last_e = 0;
	#endif	

		if (is_sender)
			atomic my_local_link_idx = j;
		if (j++ >= LOCAL_LINK_ER_TABLE_SIZE)
			break;
	}
}

// TODO: MUST be called after signal map boots up to use communication range (or whole signal map) to initialize ER
void setLocalLinkERTable() {
	uint8_t i;
	int16_t idx, in_gain;
	error_t ret;
	local_link_er_table_entry_t *se;
//#ifdef RIDB
	uint8_t j;
	int16_t tx_power, noise, interference, ni, sinr;
	sm_entry_t *sm;
//#endif
	
	for (i = 0; i < LOCAL_LINK_ER_TABLE_SIZE; i++) {
		se = &localLinkERTable[i];
		if (!se->valid)
			break;
		// rx ER only
		if (se->is_sender)
			continue;

		// w/ link range if valid
		ret = call SignalMap.findNbIdxGain(se->nb, &idx, &in_gain);
		if (SUCCESS == ret) {
//		#ifndef RIDB	
//			// for this link entry, can be only sender or receiver; this does not exlude bidirectional link
//			se->rx_interference_threshold = CC2420_DEF_RFPOWER_DBM - (in_gain >> SCALE_L_SHIFT_BIT);
//			se->rx_er_border_idx = idx;
//		#else
			sm = &signalMap[idx];
			tx_power = CC2420_DEF_RFPOWER_DBM_SCALED - sm->inbound_gain;
			noise = call SignalMap.getLocalNoise();
			// farthest interferer
			for (j = 0; j < SM_SIZE; j++) {
				// same link
				if (j == idx)
					continue;
				
				sm = &signalMap[j];
				if (!sm->valid)
					break;
				if (INVALID_GAIN == sm->inbound_gain)
					continue;
				interference = CC2420_DEF_RFPOWER_DBM_SCALED - sm->inbound_gain;
				ni = dbmSumU(interference, noise);
				sinr = tx_power - ni;
				// beyond ER
				if (sinr >= SINR_THRESHOLD)
					break;
			}
			if (j > 1) {
				sm = &signalMap[j - 1];
				se->rx_interference_threshold = CC2420_DEF_RFPOWER_DBM - (sm->inbound_gain >> SCALE_L_SHIFT_BIT);
				se->rx_er_border_idx = j - 1;
			} else {
				// empty ER; max threshold
				se->rx_interference_threshold = INVALID_DBM;
				se->rx_er_border_idx = EMPTY_ER_IDX;
			}
			//call UartLog.logTxRx(DBG_FLAG, DBG_CONTROLLER_FLAG, __LINE__, idx, j, -tx_power / 128, -noise / 128, -se->rx_interference_threshold, sinr);
//		#endif
			se->rx_er_version++;
			updateLinkERTableEntry(se->nb, my_ll_addr, my_ll_addr, se->rx_interference_threshold, se->rx_er_version, TRUE);
			updateContentionTable();
		}
//		se->rx_interference_threshold = MIN_DBM;
//		se->rx_er_version = 0;
//		se->rx_er_border_idx = call SignalMap.getSignalMapSize() - 1;
//		updateLinkERTableEntry(se->nb, my_ll_addr, my_ll_addr, se->rx_interference_threshold, se->rx_er_version, TRUE);
	}
}

// find the link
// @param is_sender: differentiate directed links
uint8_t findLocalLinkERTableIdx(am_addr_t nb, bool is_sender) {
	uint8_t i;
	local_link_er_table_entry_t *se;
	
	for (i = 0; i < LOCAL_LINK_ER_TABLE_SIZE; i++) {
		se = &localLinkERTable[i];
		if (se->valid && se->is_sender == is_sender && se->nb == nb) {
			break;
		}
	}
	return i;
}


// ---------------------------------------------------------------------
//	link table management
// ---------------------------------------------------------------------
// initialize in the very beginning
void initLinkERTable() {
	uint8_t i;
	link_er_table_entry_t *le;
	
	for (i = 0; i < LINK_ER_TABLE_SIZE; i++) {
		le = &linkERTable[i];
		//le->valid = FALSE;
		le->flags &= ~VALID_FLAG;
	}
}

// find the index to the entry of a link
async command uint8_t Controller.findLinkERTableIdx(am_addr_t sender, am_addr_t receiver) {
	uint8_t i;
	link_er_table_entry_t *le;
	
	for (i = 0; i < LINK_ER_TABLE_SIZE; i++) {
		le = &linkERTable[i];
		//if (le->valid && le->sender == sender && le->receiver == receiver) {
		if ((le->flags & VALID_FLAG) && le->sender == sender && le->receiver == receiver) {
			break;
		}
	}
	return i;
}

// find an empty slot in the table
uint8_t findEmptyLinkERTableIdx() {
	uint8_t i;
	link_er_table_entry_t *le;
	
	for (i = 0; i < LINK_ER_TABLE_SIZE; i++) {
		le = &linkERTable[i];
		//if (!le->valid) {
		if (!(le->flags & VALID_FLAG)) {
			return i;
		}
	}
	return i;
}

// "farthest" link, i.e., link whose's sender's inbound attenuation is largest
// @return: idx of the farthest link and its sender's inbound attenuation
// uint8_t findFarthestLinkERTableIdx(int16_t *link_in_gain) {
// 	uint8_t i, farthest_idx;
// 	int16_t in_gain, out_gain, max_in_gain;
// 	link_er_table_entry_t *le;
// 	
// 	max_in_gain = 0;
// 	farthest_idx = LINK_ER_TABLE_SIZE;
// 	for (i = 0; i < LINK_ER_TABLE_SIZE; i++) {
// 		le = &linkERTable[i];
// 		if (!le->valid)
// 			break;
// 		if (SUCCESS != call SignalMap.getLocalGain(le->sender, &in_gain, &out_gain))
// 			continue;
// 		if (max_in_gain < in_gain) {
// 			max_in_gain = in_gain;
// 			farthest_idx = i;
// 			*link_in_gain = in_gain;
// 		}
// 	}
// 	return farthest_idx;
// }

// improved version: optimize since signal map is sorted by inbound attenuation
// @return link_in_gain: used later
// pin 2 special types of links, i.e., incident links
// 1) do not kick out <my_ll_addr, my_receiver>; otherwise ER for local outgoing link is missing and thus contention relationship is missing; already guaranteed, although implicitly; bcoz signalMap does not contain my_ll_addr
// 2) incoming links
// TODO: verify latency
// should NOT be called bcoz MAX_ACTIVE_LINK_SIZE is enough, unless link id corrupted
uint8_t findFarthestLinkERTableIdx(int16_t *link_in_gain) {
	uint8_t i, j;
	link_er_table_entry_t *le;
	
	sm_entry_t *se;
	uint8_t sm_size = call SignalMap.getSignalMapSize();
	
	// from far to close
	for (i = sm_size; i > 0; i--) {
		se = &signalMap[i - 1];
		
		for (j = 0; j < LINK_ER_TABLE_SIZE; j++) {
			le = &linkERTable[j];
			//if (!le->valid)
			if (!(le->flags & VALID_FLAG))
				break;
			// do not evict incident links
			if (le->sender == my_ll_addr || le->receiver == my_ll_addr)
				continue;
			if (le->sender == se->nb) {
				*link_in_gain = se->inbound_gain;
				return j;
			}
		}
	}
	return LINK_ER_TABLE_SIZE;
}

// direct sort, not indirect sort
// @param idx: index of the link whose rank changes in linkERTable
void sortLinkERTableIdx(uint8_t idx) {
	uint8_t i, link_er_table_size_;
	uint8_t new_prio;
	link_er_table_entry_t tmp;
	link_er_table_entry_t *le, *copyLinkERTable;
	
	// runtime check
	if (idx >= LINK_ER_TABLE_SIZE) {
		assert(0);
		return;
	}
	start_time = call LocalTime.get();
	
	copyLinkERTable = linkERTable;
	tmp = copyLinkERTable[idx];
	new_prio = tmp.prio;
	
	if (idx > 0) {
		if (new_prio < copyLinkERTable[idx - 1].prio) {
			// moving forward
			for (i = idx; i > 0; i--) {
				le = &copyLinkERTable[i - 1];
				if (le->prio > new_prio) {
					copyLinkERTable[i] = copyLinkERTable[i - 1];
				} else {
					break;
				}
			}
			// found right location
			copyLinkERTable[i] = tmp;
		}
	}
	
	//link_er_table_size = getLinkERTableSize();
	atomic link_er_table_size_ = link_er_table_size;
	if ((idx + 1) < link_er_table_size_) {
		if (new_prio > copyLinkERTable[idx + 1].prio) {
			// moving backward
			for (i = idx; (i + 1) < link_er_table_size_; i++) {
				le = &copyLinkERTable[i + 1];
				if (le->prio < new_prio) {
					copyLinkERTable[i] = copyLinkERTable[i + 1];
				} else {
					break;
				}
			}
			// found right location
			copyLinkERTable[i] = tmp;
		}
	}
}

// sort the entire linkERTable
void sortLinkERTable() {
	uint8_t i;
	link_er_table_entry_t *le;
	
	for (i = 0; i < LINK_ER_TABLE_SIZE; i++) {
		le = &linkERTable[i];
		//if (!le->valid)
		if (!(le->flags & VALID_FLAG))
			break;
		if (!(le->flags & RANK_INCREMENT_FLAG))
			continue;
		// clear flag
		le->flags &= ~RANK_INCREMENT_FLAG;
	 	// dec priority after forwarding
	 	// special case: link ER whose receiver is me; always highest priority to let sender know ASAP, otherwise DATA cannot be used to sample PDR bcoz of ER version inconsistent
		// TODO: do this only after sent, e.g., sendDone(SUCCESS)
		if (le->receiver != my_ll_addr) {
			if (le->prio < 255) {
				le->prio++;
				sortLinkERTableIdx(i);
			}
		}
	}
}


// a new ER item arrives, either local or non-local
void updateLinkERTableEntry(am_addr_t sender, am_addr_t receiver, am_addr_t from, int16_t rx_interference_threshold, uint8_t rx_er_version, bool is_local_er) {
	uint8_t idx;
	link_er_table_entry_t *le;
	
	int16_t in_gain, out_gain, max_in_gain;
	
	// TODO: hack to prevent corrupted links from addmission, assume all links known; still necessary after additional CRC added?
	if (!call Util.isActiveLink(sender, receiver)) {
		return;
	}
	
	// FIFO	
	// if found
	//		update if newer version
	// else
	// 		if exists empty entry
	//			initialize
	//		else
	//			if new entry closer than farthest entry in table
	//				replace			
	//			else
	//				drop
	idx = call Controller.findLinkERTableIdx(sender, receiver);
	if (idx < LINK_ER_TABLE_SIZE) {
		le = &linkERTable[idx];
		
		// only update ER if valid & newer
		if (rx_interference_threshold != INVALID_DBM) {
			// ER
			if (((int8_t)(rx_er_version - le->rx_er_version)) > 0) {
				le->rx_interference_threshold = rx_interference_threshold;
				le->rx_er_version = rx_er_version;
				// update contention bitmap
				le->flags |= ER_CHANGED_FLAG;
				//post updateContentionTask();
				
				// update priority only after ER update
				le->prio = is_local_er ? LOCAL_INIT_PRIO : NON_LOCAL_INIT_PRIO;
				sortLinkERTableIdx(idx);
			}
		}
	} else {
		idx = findEmptyLinkERTableIdx();
		if (idx < LINK_ER_TABLE_SIZE) {
			// initialize
			initLinkERTableEntry(idx, sender, receiver, rx_interference_threshold, rx_er_version, is_local_er);
			le = &linkERTable[idx];
			le->flags |= ER_CHANGED_FLAG;
			//post updateContentionTask();
			
			// only new entry will change link_er_table_size
			atomic link_er_table_size++;
			sortLinkERTableIdx(idx);
		} else {
			idx = findFarthestLinkERTableIdx(&max_in_gain);
			// if reach here, means link id corrupted
			assert(0);
			//call UartLog.logTxRx(DBG_FLAG, DBG_CONTROLLER_FLAG, __LINE__, 0, LINK_ER_TABLE_SIZE, sender, receiver, from, idx);
			if (idx < LINK_ER_TABLE_SIZE) {
				if (SUCCESS == call SignalMap.getLocalGain(sender, &in_gain, &out_gain)) {
					// new link is "closer": add MIN_GAIN_GAP to prevent oscillation bcoz of gain estimation fluctuation
					if ((in_gain + MIN_GAIN_GAP) < max_in_gain) {
						// initialize
						initLinkERTableEntry(idx, sender, receiver, rx_interference_threshold, rx_er_version, is_local_er);
						le = &linkERTable[idx];
						le->flags |= ER_CHANGED_FLAG;
						//post updateContentionTask();
						
						sortLinkERTableIdx(idx);
					}
				}
			}
		}
	}
}

inline void initLinkERTableEntry(uint8_t idx, am_addr_t sender, am_addr_t receiver, int16_t rx_interference_threshold, uint8_t rx_er_version, bool is_local_er) {
	link_er_table_entry_t *le;
	// initialize
	le = &linkERTable[idx];
	le->sender = sender;
	le->receiver = receiver;
	le->flags = 0;
	//le->valid = TRUE;
	le->flags |= VALID_FLAG;
	// priority
	le->prio = is_local_er ? LOCAL_INIT_PRIO : NON_LOCAL_INIT_PRIO;
	// must be valid since isActiveLink() test passed earlier
	le->link_idx = call Util.findLinkIdx(sender, receiver);
	// no sanity check for the 1st time
	le->rx_interference_threshold = rx_interference_threshold;
	le->rx_er_version = rx_er_version;
	le->contend_flags = 0;
}

// whether node with inbound gain of @gain_scaled to me is in my ER denoted by @interference_threshold
// treat unknown conservatively
inline bool inER(int16_t gain_scaled, int16_t interference_threshold) {
	// TODO to be more accurate: (CC2420_DEF_RFPOWER_DBM << SCALE_L_SHIFT_BIT - gain_scaled) >= interference_threshold << SCALE_L_SHIFT_BIT
//	if (INVALID_GAIN == gain_scaled)
//		assert(0);
	return (INVALID_GAIN == gain_scaled) ? TRUE : ((CC2420_DEF_RFPOWER_DBM - (gain_scaled >> SCALE_L_SHIFT_BIT)) >= interference_threshold);
}



// called when a link's ER changes: either local link ER adapts or receive newer ER
// linkERTable[i].contend_flags j-th bit may change if ER of either changes:
// 1) link i
// 2) local link j: special case, i.e., linkERTable[j's corresponding idx].contend_flags is all 1's except j-th bit
// ASSUMPTION: contention relationship does not change due to signal map change
void updateContention(uint8_t link_idx) {
	uint8_t i, idx, local_link_idx;
	bool is_sender;
	am_addr_t nb;
	link_er_table_entry_t *le;
	local_link_er_table_entry_t *se;
	
	// runtime check
	if (link_idx >= LINK_ER_TABLE_SIZE) {
		assert(0);
		return;
	}
	le = &linkERTable[link_idx];
	// case 1): non-local link
	if (!(le->sender == my_ll_addr || le->receiver == my_ll_addr)) {
		// each local link
		for (i = 0; i < LOCAL_LINK_ER_TABLE_SIZE; i++) {
			se = &localLinkERTable[i];
			if (!se->valid)
				break;
			
			if (se->is_sender) {
				idx = call Controller.findLinkERTableIdx(my_ll_addr, se->nb);
			} else {
				idx = call Controller.findLinkERTableIdx(se->nb, my_ll_addr);
			}
			// this can happen when 1st link admitted into linkERTable is non-local
			if (idx >= LINK_ER_TABLE_SIZE)
				return;
			if (isContend(idx, link_idx)) {
				le->contend_flags |= (1 << i);
			} else {
				le->contend_flags &= ~(1 << i);		
			}
		}
	} else {
		// case 2): local link
		is_sender = (le->sender == my_ll_addr);
		nb = is_sender ? le->receiver : le->sender;
		local_link_idx = findLocalLinkERTableIdx(nb, is_sender);
		//call UartLog.logTxRx(DBG_FLAG, DBG_ER_FLAG, __LINE__, is_sender, nb, local_link_idx, link_idx, le->sender, le->receiver);
		// each link
		for (i = 0; i < LINK_ER_TABLE_SIZE; i++) {
			le = &linkERTable[i];
			//if (!le->valid)
			if (!(le->flags & VALID_FLAG))
				break;
			// special case
			if (i == link_idx)
				le->contend_flags = 0xFF;
			if (isContend(link_idx, i)) {
				le->contend_flags |= (1 << local_link_idx);
			} else {
				le->contend_flags &= ~(1 << local_link_idx);		
			}
		}
	}
}

// update contention relationship upon newer ER arrival, either local or non-local
//task void updateContentionTask() {
//	uint8_t i;
//	bool is_er_changed;
//	link_er_table_entry_t *le;
//	
//	is_er_changed = FALSE;
//	for (i = 0; i < LINK_ER_TABLE_SIZE; i++) {
//		le = &linkERTable[i];
//		//if (!le->valid)
//		if (!(le->flags & VALID_FLAG))
//			break;
//		if (le->flags & ER_CHANGED_FLAG) {
//			is_er_changed = TRUE;
//			le->flags &= ~ER_CHANGED_FLAG;
//			break;
//		}
//	}
//	// no pending
//	if (!is_er_changed)
//		return;
//	
//	// update is_in_er
//	if (amInLinkER(i)) {
//		le->flags |= IS_IN_ER_FLAG;
//	} else {
//		le->flags &= ~IS_IN_ER_FLAG;	
//	}
//	
//	// update contend_flags
//	updateContention(i);
//	// repost itself
//	post updateContentionTask();
//}
void updateContentionTable() {
	uint8_t i;
	link_er_table_entry_t *le;
	
	for (i = 0; i < LINK_ER_TABLE_SIZE; i++) {
		le = &linkERTable[i];
		//if (!le->valid)
		if (!(le->flags & VALID_FLAG))
			break;
		if (le->flags & ER_CHANGED_FLAG) {
			le->flags &= ~ER_CHANGED_FLAG;
			
			// update is_in_er
			if (amInLinkER(i)) {
				le->flags |= IS_IN_ER_FLAG;
			} else {
				le->flags &= ~IS_IN_ER_FLAG;	
			}
	
			// update contend_flags
			updateContention(i);
		}
	}
}

/*
 * two links contend iff
 * either share a node w/
 * or links in each other's rx ER (inbound)
 * @param i, j: index of 2 links in linkERTable, i MUST be a local link

 * be conservative and treat a link as contending if any info unknown, either ER or gain?
 * NO, otherwise a node is outside my largest tx range, it would always regard itself in my ER. Aggresive here bcoz
 * 1) assuming ER is no larger than max tx range, then this means some node outside ER overhears my ER through ER relay
 * 2) these occur rare if SM_BEACON_CNT is large enough and hopefully controller is robust
 */
//#warning conservative
bool isContend(uint8_t i, uint8_t j) {
	bool is_sender;
	error_t ret;
	am_addr_t s1, r1, s2, r2;
	int16_t in_gain, out_gain;
	link_er_table_entry_t *me, *le;

	// runtime check
	if (i >= LINK_ER_TABLE_SIZE || j >= LINK_ER_TABLE_SIZE) {
		assert(0);
		return TRUE;
	}
	
	me = &linkERTable[i];
	le = &linkERTable[j];
	s1 = me->sender;
	r1 = me->receiver;
	s2 = le->sender;
	r2 = le->receiver;
	
	// s1 -> r1
	// s2 -> r2
	if (s1 == s2 || s1 == r2 || r1 == s2 || r1 == r2) {
		// in case of the same link; link does not contend w/ itself
		return !(s1 == s2 && r1 == r2);
	}

	// I'm s1 or r1
	is_sender = (my_ll_addr == s1);
	// r1 interfered by s2?
	ret = is_sender ? call SignalMap.getGain(r1, s2, &in_gain, &out_gain) : call SignalMap.getLocalGain(s2, &in_gain, &out_gain);
	if (ret != SUCCESS)	{
		//call UartLog.logTxRx(DBG_FLAG, DBG_ERR_FLAG, __LINE__, s1, r1, s2, r2, is_sender, 0);
		return FALSE;	// return TRUE;
	}
	if (inER(in_gain, me->rx_interference_threshold))		return TRUE;

	// r2 interfered by s1?
	ret = is_sender ? call SignalMap.getLocalGain(r2, &in_gain, &out_gain) : call SignalMap.getGain(s1, r2, &in_gain, &out_gain);
	if (ret != SUCCESS)	{
		//call UartLog.logTxRx(DBG_FLAG, DBG_ERR_FLAG, __LINE__, s1, r1, s2, r2, is_sender, 0);
		return FALSE;
	}
	if (inER(out_gain, le->rx_interference_threshold))		return TRUE;

	// none interferes
	return FALSE;
}


// am I in link idx's latest ER; called in addLinkEstHeaderAndFooter() to limit relays in ER
bool amInLinkER(uint8_t idx) {
	error_t ret;
	int16_t in_gain, out_gain;
	link_er_table_entry_t *le;
	
	// runtime check
	if (idx >= LINK_ER_TABLE_SIZE) {
		assert(0);
		return TRUE;
	}

	le = &linkERTable[idx];
	// link end nodes are within the link's ER
	if (le->sender == my_ll_addr || le->receiver == my_ll_addr)		return TRUE;
	ret = call SignalMap.getLocalGain(le->receiver, &in_gain, &out_gain);
	if (ret != SUCCESS)	{
		//assert(idx);
		return FALSE;
	}
	if (inER(out_gain, le->rx_interference_threshold))				return TRUE;
	
	return FALSE;
}

// -----------------------------------------------------------------------------------------
// power arithmic based on dBm
// functions ending in U means only dealing w/ positive power, S w/ negative power as well
// -----------------------------------------------------------------------------------------
// pre-computed offset table; scaled; start from 1
int16_t diffDeltaTable[] = {879, 554, 387, 282, 211, 161, 124, 96, 75, 59, 46, 36, 29, 23, 18, 14, 11, 9, 7, 6, 4, 4, 3, 2, 2, 1, 1, 1, 1, 1};
// z = x - y; all scaled in dBm
// assert((x - y) >=  (0x1 << SCALE_L_SHIFT_BIT))
// make it command to be visible to SignalMapPUtil$calcInboundGain
async command int32_t Controller.dbmDiffU(int32_t x, int32_t y) {
	int32_t n, delta;
	
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
// z = x + y; all scaled in dBm; only positive power in mW
inline int32_t dbmSumU(int32_t x, int32_t y) {
	int32_t n, delta, tmp;
	
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
			z.abs = call Controller.dbmDiffU(x.abs, y.abs);
		} else if (y.abs >= (x.abs + (0x1 << SCALE_L_SHIFT_BIT))) {
			z.sign = y.sign;
			z.abs = call Controller.dbmDiffU(y.abs, x.abs);
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



static long isqrt(long num) {
    long op = num;
    long res = 0;
    long one = 1L<<30; // The second-to-top bit is set: 1 << 14 for short
    //uint32_t start_time = call LocalTimeMilli.get();
    
    // "one" starts at the highest power of four <= the argument.
    while (one > op) {
        dbg("LoopDetection", "%s \n", __FUNCTION__);
        one >>= 2;
    }

    while (one != 0) {
        dbg("LoopDetection", "%s \n", __FUNCTION__);
        if (op >= res + one) {
            op -= res + one;
            res = (res >> 1) + one;
        }
        else
          res >>= 1;
        one >>= 2;
    }
    return res;
}

// --------------------------------------------------------------------------------------------------
//// update contention relationships btw. local outgoing link and other links based on their latest ER
//async command void Controller.updateContention(uint8_t local_link_idx) {
//	uint8_t i, link_idx;
//	link_er_table_entry_t *le;
//	local_link_er_table_entry_t *se;
//	
//	// runtime check
//	if (local_link_idx >= LOCAL_LINK_ER_TABLE_SIZE) {
//		assert(0);
//		return;
//	}
//	
//	se = &localLinkERTable[local_link_idx];
//	if (se->is_sender) {
//		link_idx = call Controller.findLinkERTableIdx(my_ll_addr, se->nb);
//	} else {
//		link_idx = call Controller.findLinkERTableIdx(se->nb, my_ll_addr);
//	}

//	// each link
//	for (i = 0; i < LINK_ER_TABLE_SIZE; i++) {
//		le = &linkERTable[i];
//		if (!le->valid)
//			break;
//		start_time = call LocalTime.get();
//		if (isContend(link_idx, i)) {
//			le->contend_flags |= (1 << local_link_idx);
//		} else {
//			le->contend_flags &= ~(1 << local_link_idx);		
//		}
//		call UartLog.logEntry(DBG_FLAG, DBG_DELAY_FLAG, __LINE__, call LocalTime.get() - start_time);
//	}
//}

// --------------------------------------------------------------------------------------------------
//// re-sort link ER table by ascending priority after its update
//// indirect sorting to save expensive copying

//// index of VALID link ER entry ordered by ascending priority
//// linkERTable[sortedIndices[i]] stores the i-th smallest priority entry if valid
//uint8_t sortedIndices[LINK_ER_TABLE_SIZE];
//void initSortedIndices() {
//	uint8_t i;
//	
//	for (i = 0; i < LINK_ER_TABLE_SIZE; i++) {
//		sortedIndices[i] = i;
//	}
//}
//// convert index in linkERTable to index in sortedIndices
//inline uint8_t index2SortedIndex(uint8_t idx) {
//	uint8_t i;
//	
//	for (i = 0; i < LINK_ER_TABLE_SIZE; i++) {
//		if (sortedIndices[i] == idx)
//			return i;
//	}
//	return LINK_ER_TABLE_SIZE;
//}

//// @param idx: index of the link whose priority changes in linkERTable
//void sortLinkERTable(uint8_t idx) {
//	uint8_t i, sorted_idx;
//	uint8_t new_prio;
//	link_er_table_entry_t tmp;
//	link_er_table_entry_t *le, *copyLinkERTable;
//	uint8_t local_link_er_table_size, *copySortedIndices;
//	
//	// runtime check
//	if (idx >= LINK_ER_TABLE_SIZE) {
//		assert(0);
//		return;
//	}
//	
//	copyLinkERTable = linkERTable;
//	copySortedIndices = sortedIndices;
//	atomic local_link_er_table_size = link_er_table_size;
//	tmp = copyLinkERTable[idx];
//	new_prio = tmp.prio;
//	
//	// corresponding index in sortedIndices
//	sorted_idx = index2SortedIndex(idx);
//	if (sorted_idx >= LINK_ER_TABLE_SIZE) {
//		assert(0);
//		return;
//	}
//	
//	if (sorted_idx > 0) {
//		if (new_prio < copyLinkERTable[copySortedIndices[sorted_idx - 1]].prio) {
//			// moving forward
//			for (i = sorted_idx; i > 0; i--) {
//				le = &copyLinkERTable[copySortedIndices[i - 1]];
//				// no need to check le->valid bcoz it is valid
//				if (le->prio > new_prio) {
//					//copyLinkERTable[i] = copyLinkERTable[i - 1];
//					copySortedIndices[i] = copySortedIndices[i - 1];
//				} else {
//					break;
//				}
//			}
//			// found right location
//			//copyLinkERTable[i] = tmp;
//			copySortedIndices[i] = idx;
//		}
//	}
//	
////	link_er_table_size = getLinkERTableSize();
//	if ((sorted_idx + 1) < local_link_er_table_size) {
//		if (new_prio > copyLinkERTable[copySortedIndices[sorted_idx + 1]].prio) {
//			// moving backward
//			for (i = sorted_idx; (i + 1) < local_link_er_table_size; i++) {
//				le = &copyLinkERTable[copySortedIndices[i + 1]];
//				if (!le->valid)
//					break;
//				if (le->prio < new_prio) {
//					//copyLinkERTable[i] = copyLinkERTable[i + 1];
//					copySortedIndices[i] = copySortedIndices[i + 1];
//				} else {
//					// again, ASSUMPE consecutive linkERTable
//					break;
//				}
//			}
//			// found right location
//			//copyLinkERTable[i] = tmp;
//			copySortedIndices[i] = idx;
//		}
//	}
//}

//uint8_t getLinkERTableSize() {
//	uint8_t i, total;
//	link_er_table_entry_t *le;
//	
//	total = 0;
//	for (i = 0; i < LINK_ER_TABLE_SIZE; i++) {
//		le = &linkERTable[i];
//		// ASSUMPTION: entries are consecutive bcoz of FIFO
//		if (!le->valid)
//			break;
//		total++;
//	}
//	return total;
//}


// update priorities and sort accordingly after relay
// sort entry [0 .. const_max_item_cnt - 1] w/ [const_max_item_cnt .. LINK_ER_TABLE_SIZE - 1]
// observation: both are sorted, so just sort backwards to save processing
//void updateAndSortLinkERTable() {
//	uint8_t i;
//	link_er_table_entry_t *le;
//  	
//  	for (i = const_max_item_cnt; i > 0; i--) {
//  		le = &linkERTable[i - 1];
//  		// do not terminate here
//  		if (!le->valid)
//  			continue;
//		// dec priority after forwarding
//		if (le->prio < 255)
//			le->prio++;
//		
//		// sort
//		sortLinkERTable(i - 1);
//	}
//}

//// linkERTable[0 .. sorted_idx - 1] is sorted
//uint8_t sorted_idx;
//// insert sorted_idx's entry into the partially sorted table
//task void sortLinkERTableTask() {
//	uint8_t i;
//	uint8_t sorted_idx_;
//	link_er_table_entry_t *copyLinkERTable, tmp;
//	
//	atomic sorted_idx_ = sorted_idx;
//	atomic copyLinkERTable = linkERTable;
//    if (sorted_idx_ >= LINK_ER_TABLE_SIZE)
//    	return;
//    tmp = copyLinkERTable[sorted_idx_];
//    if (!tmp.valid)
//    	return;
//    
//    for (i = sorted_idx_; i > 0 && copyLinkERTable[i - 1].prio > tmp.prio; i--) {
//        copyLinkERTable[i] = copyLinkERTable[i - 1];
//    }
//    copyLinkERTable[i] = tmp;
//    
//    call UartLog.logEntry(DBG_FLAG, DBG_ER_FLAG, __LINE__, sorted_idx_);
//    atomic sorted_idx++;
//    post sortLinkERTableTask();
//}


//// @param bit_idx: link_idx in linkERTable
//inline bool testBit(uint8_t *bitmap, uint8_t bit_idx) {
//	uint8_t byte_offset, bit_offset;
//	bool is_set;
//	
//	//byte_offset = bit_idx / 8;
//	byte_offset = (bit_idx >> 3);
//	//bit_offset = i % 8;
//	bit_offset = bit_idx & 0x07;
//	
//	is_set = bitmap[byte_offset] & (1 << bit_offset);
//	if (is_set) {
//		// clear
//		bitmap[byte_offset] &= ~(1 << bit_offset);
//	}
//	return is_set;
//}

//inline void setBit(uint8_t *bitmap, uint8_t bit_idx) {
//	uint8_t byte_offset, bit_offset;
//	
//	byte_offset = (bit_idx >> 3);
//	bit_offset = bit_idx & 0x07;
//	bitmap[byte_offset] |= (1 << bit_offset);
//}

