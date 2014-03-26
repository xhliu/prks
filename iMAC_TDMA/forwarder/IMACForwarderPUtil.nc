// ---------------------------------------------------------------------
// forward declarations
// ---------------------------------------------------------------------
// do scheduling in a slot specified by g_slot_start_time
void scheduleSlot(uint32_t g_slot_start_time);
// tx/rx control packets
void txrxCtrl();

//uint32_t nextTxSlot(uint32_t current_slot);
//void nextRxSlot(uint32_t current_slot);
//inline bool winChannel(uint8_t local_link_idx, uint8_t link_idx, uint8_t t, bool *is_any_contend);
inline uint8_t getConflictSetSize();
inline uint8_t getLinkSetSize();
//inline bool isConservativeStay(uint32_t current_slot);
//inline uint32_t rand32(uint32_t seed);

//// @return: next slot for me to tx
//uint32_t nextTxSlot(uint32_t current_slot) {
//	bool is_any_contend;
//	uint8_t t;
//	uint8_t my_link_idx;
//	uint32_t slot;
//	local_link_er_table_entry_t *se;

//	// not a sender; or the corresponding receiver fails to be programmed
//	my_link_idx = call Controller.findLinkERTableIdx(my_ll_addr, call Util.getReceiver());
//	if (my_link_idx >= LINK_ER_TABLE_SIZE) {
//		return INVALID_SLOT;
//	}
//	// update contention relationship using latest ER
//	//call Controller.updateContention(my_local_link_idx);
//	
//	// look for next slot to tx
//	//for (slot = current_slot + 1; slot <= (current_slot + MAX_SLOT_FORWARD); slot++) {
//	for (slot = current_slot + 1; (int32_t)(slot - (current_slot + MAX_SLOT_FORWARD)) <= 0; slot++) {
//		// 	1) an alternative is to use high 16 bits of global time, which changes every 65536 / 320 slots; t does not change in this period, one link can consistently win the channel and starve others
//		//	2) use only lowest 7 bits for short periodicity while still 2^7 larger than # of links
//		t = slot & SLOT_MASK;
//		//#warning dedicate these slots to ctrl channel, more specifically for ftsp
//		// dedicate these slots to ctrl channel, more specifically for ftsp; choose (MAX_SLOT_FORWARD - active_link_size) bcoz it guarantees a link's priority can reach max (all 1's)
//		if (t < (MAX_SLOT_FORWARD - active_link_size))
//			continue;
//		
//		if (winChannel(my_local_link_idx, my_link_idx, t, &is_any_contend)) {
//			se = &localLinkERTable[my_local_link_idx];
//			/* TODO: inc force switch prob. if problematic
//			 * special handle for empty ER
//			 * force receiver to switch to CTRL from time to time, i.e., every 1 of out MAX_SLOT_FORWARD slots; otherwise receiver always stays in DATA channel and latest ER is never diffused
//			 */
//			if (!is_any_contend && 0 == t) {
//				//call UartLog.logTxRx(DBG_FLAG, DBG_TDMA_FLAG, __LINE__, 0, 0, is_any_contend, t, slot - current_slot, current_slot);
//				// just warning
//				assert(current_slot);
//				slot++;
//			}
//			se->next_slot_by_tx = slot;
//			return slot;
//		}
//	}
//	// should not reach here
//	assert(current_slot);
//	return INVALID_SLOT;
//}

//// i-th entry stores the index of i-th link in localLinkERTable in linkERTable
////uint8_t local2LinkERTableIdx[LOCAL_LINK_ER_TABLE_SIZE];
//void nextRxSlot(uint32_t current_slot) {
//	bool is_any_contend;
//	uint8_t i, link_idx;
//	local_link_er_table_entry_t *se;
//	
//	uint8_t t;
//	uint32_t slot;
//	
////	// each incoming link; pre-compute index in linkERTable once
////	for (i = 0; i < LOCAL_LINK_ER_TABLE_SIZE; i++) {
////		se = &localLinkERTable[i];
////		// only for incoming link
////		if (!se->valid || se->is_sender)
////			continue;
////		// some incoming links may not be in linkERTable, unlike outgoing link
////		// but that's fine since this is opportunistic anyway
////		local2LinkERTableIdx[i] = call Controller.findLinkERTableIdx(my_ll_addr, se->nb);
////	}
//	
//	// each incoming link
//	for (i = 0; i < LOCAL_LINK_ER_TABLE_SIZE; i++) {
//		se = &localLinkERTable[i];
//		// only for incoming link
//		if (!se->valid || se->is_sender)
//			continue;
//		// only when se->next_slot_by_t(r)x is valid
//		if (INVALID_SLOT == se->next_slot_by_tx || INVALID_SLOT == se->next_slot_by_rx)
//			continue;
//		link_idx = call Controller.findLinkERTableIdx(se->nb, my_ll_addr);
//		if (link_idx >= LINK_ER_TABLE_SIZE)
//			continue;
//		// update contention relationship using latest ER
//		//call Controller.updateContention(i);
//		
//		// look for next 0 to rx
//		// 1) slot < se->next_slot_by_tx: only prior to next_slot_by_tx can be opportunity
//		// 2) slot < se->next_slot_by_rx: multiple next_slot_by_rx can be found prior to next_slot_by_tx since it is computed every slot, always keep the earliest one; no need to buffer every next_slot_by_rx; no need to validate since INVALID_SLOT is max
//		//for (slot = current_slot + 1; slot < se->next_slot_by_tx && slot < se->next_slot_by_rx; slot++) {
//		for (slot = current_slot + 1; (int32_t)(slot - se->next_slot_by_tx) < 0 && (int32_t)(slot - se->next_slot_by_rx) < 0; slot++) {
//			t = slot & SLOT_MASK;
//			//#warning dedicate these slots to ctrl channel, more specifically for ftsp
//			if (t < (MAX_SLOT_FORWARD - active_link_size))
//				continue;
//			
//			if (winChannel(i, link_idx, t, &is_any_contend)) {
//				se->next_slot_by_rx = slot;
//				break;
//			}
//		}
//	}
//}

//// if a link has the highest priority in this slot t
//// @param local_link_idx: index in localLinkERTable
//// @param link_idx: index in linkERTable
//// @param is_any_contend: is any other link contending w/ this link, i.e., empty ER
//inline bool winChannel(uint8_t local_link_idx, uint8_t link_idx, uint8_t t, bool *is_any_contend) {
//	uint8_t i;
//	uint8_t prio, my_prio;
//	link_er_table_entry_t *le;
//	
//	le = &linkERTable[link_idx];
//	my_prio = le->link_idx ^ t;
//	*is_any_contend = FALSE;
//	// LAMA protocol
//	for (i = 0; i < LINK_ER_TABLE_SIZE; i++) {
//		le = &linkERTable[i];
//		//if (!le->valid)
//		if (!(le->flags & VALID_FLAG))
//			break;
//		//call UartLog.logTxRx(DBG_FLAG, DBG_TDMA_FLAG, __LINE__, i, le->sender, le->receiver, le->contend_flags & (1 << local_link_idx), local_link_idx, current_slot);
//		// not contend w/ this link
//		if (!(le->contend_flags & (1 << local_link_idx)))
//			continue;
//		*is_any_contend = TRUE;
//		// priority
//		//prio = link2Prio(ne->sender, ne->receiver, t, &link);
//		prio = le->link_idx ^ t;
//		if (prio > my_prio) {
//			return FALSE;
//		}
//	}
//	return TRUE;
//}

// get link ER table size of conflicting entries w/ local outgoing link
// @return: LINK_ER_TABLE_SIZE for node w/o outgoing link
inline uint8_t getConflictSetSize() {
	uint8_t i, total;
	link_er_table_entry_t *le;
	
	// not a sender
	if (my_local_link_idx >= LOCAL_LINK_ER_TABLE_SIZE)
		return LINK_ER_TABLE_SIZE;
	
	total = 0;
	for (i = 0; i < LINK_ER_TABLE_SIZE; i++) {
		le = &linkERTable[i];
		//if (!le->valid)
		if (!(le->flags & VALID_FLAG))
			break;
		// not contend w/ this link
		if (le->contend_flags & (1 << my_local_link_idx))
			total++;
	}
	return total;
}

// get link ER table size of active entries
inline uint8_t getLinkSetSize() {
	uint8_t i, total;
	link_er_table_entry_t *le;
	
	total = 0;
	for (i = 0; i < LINK_ER_TABLE_SIZE; i++) {
		le = &linkERTable[i];
		//if (!le->valid)
		if (!(le->flags & VALID_FLAG))
			break;
		total++;
	}
	return total;
}

//// conservatively stay in DATA channel? iff any pending incoming link wins
//inline bool isConservativeStay(uint32_t current_slot) {
//	uint8_t i, t, max_prio, prio;
//	bool is_rx_pending;
//	local_link_er_table_entry_t *se;
//	
//	// nodes not tx/rx DATA stay in CTRL channel
//	is_rx_pending = FALSE;
//	t = current_slot & SLOT_MASK;
//	max_prio = 0;
//	// each incoming link; pre-compute index in linkERTable once
//	for (i = 0; i < LOCAL_LINK_ER_TABLE_SIZE; i++) {
//		se = &localLinkERTable[i];
//		// also consider outgoing link, whose is_rx_pending is always FALSE
//		if (!se->valid)// || se->is_sender)
//			continue;
//		prio = se->link_idx ^ t;
//		if (max_prio < prio || 0 == max_prio) {
//			max_prio = prio;
//			is_rx_pending = se->is_rx_pending;
//		}
//	}
//	// highest priority link is expecting DATA
//	return is_rx_pending;
//}

//uint8_t activeLinkERSize() {
//	uint8_t i, total;
//	link_er_table_entry_t *le;
//	
//	total = 0;
//	for (i = 0; i < LINK_ER_TABLE_SIZE; i++) {
//		le = &linkERTable[i];
//		if (le->valid)
//			total++;
//	}
//	return total;
//}

