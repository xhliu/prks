// neighbor link ER table: store all links
void initLinkTable();
uint8_t findLinkTableIdx(am_addr_t sender, am_addr_t receiver);
uint8_t findEmptyLinkTableIdx();
void updateLinkTableEntry(am_addr_t sender, am_addr_t receiver, uint8_t pkt_block_idx, int16_t rx_signal, int16_t ni);
bool passConcurrencyCheck(am_addr_t addr);

// cache management function prototypes
void initCache();
uint8_t findCacheIdx(am_addr_t sender, am_addr_t receiver);
uint8_t findEmptyCacheIdx();
uint8_t findMinCacheIdx();
uint32_t maxCache();
void printCache();
void insertCache(am_addr_t sender, am_addr_t receiver, uint32_t link_nav);
uint32_t deleteCache(am_addr_t sender, am_addr_t receiver);
uint8_t cacheSize();

// ---------------------------------------------------------------------
// link table management functions
// ---------------------------------------------------------------------
// initialize in the very beginning
void initLinkTable() {
	uint8_t i;
	link_table_entry_t *le;
	
	for (i = 0; i < LINK_TABLE_SIZE; i++) {
		le = &linkTable[i];
		le->valid = FALSE;
	}
}

// find the index to the entry of a link
uint8_t findLinkTableIdx(am_addr_t sender, am_addr_t receiver) {
	uint8_t i;
	link_table_entry_t *le;
	
	for (i = 0; i < LINK_TABLE_SIZE; i++) {
		le = &linkTable[i];
		if (le->valid && le->sender == sender && le->receiver == receiver) {
			break;
		}
	}
	return i;
}

// find an empty slot in the table
uint8_t findEmptyLinkTableIdx() {
	uint8_t i;
	link_table_entry_t *le;
	
	for (i = 0; i < LINK_TABLE_SIZE; i++) {
		le = &linkTable[i];
		if (!le->valid) {
			return i;
		}
	}
	return i;
}

// a new item arrives
void updateLinkTableEntry(am_addr_t sender, am_addr_t receiver, uint8_t pkt_block_idx, int16_t rx_signal, int16_t ni) {
	uint8_t idx;
	link_table_entry_t *le;
	
	// FIFO	
	// if found
	//		update
	// else
	// 		if exists empty entry
	//			initialize
	//		else
	//			drop
	idx = findLinkTableIdx(sender, receiver);
	if (idx < LINK_TABLE_SIZE) {
		le = &linkTable[idx];
		le->rx_signal = rx_signal;
		le->ni = ni;
		//le->rx_timestamp = call LocalTime.get();
		le->complete_timestamp = call LocalTime.get() + (BLOCK_SIZE - 1 - pkt_block_idx) * PACKET_TIME;
	} else {
		idx = findEmptyLinkTableIdx();
		if (idx < LINK_TABLE_SIZE) {
			// initialize
			le = &linkTable[idx];
			le->valid = TRUE;
			le->sender = sender;
			le->receiver = receiver;
			le->rx_signal = rx_signal;
			le->ni = ni;
			//le->rx_timestamp = call LocalTime.get();
			le->complete_timestamp = call LocalTime.get() + (BLOCK_SIZE - 1 - pkt_block_idx) * PACKET_TIME;
		}
	}
}

// were I transmit, are active links significantly affected
//#warning not conservative when gain unknown
bool passConcurrencyCheck(am_addr_t addr) {
	uint8_t i;
	link_table_entry_t *le;
	int16_t out_gain, my_signal, my_interference, total_ni, sinr;
	uint32_t now;
	
	out_gain = call SignalMap.getOutboundGain(addr);
	if (INVALID_GAIN == out_gain) {
		assert(0);
		// conservative
		return FALSE;
	}
	if (INVALID_DBM == receiver_ni) {
		// bootstrap; otherwise deadlock, can never tx
		return TRUE;	//	FALSE;
	}
	my_signal = CC2420_DEF_RFPOWER_DBM_SCALED - out_gain;
	sinr = my_signal - receiver_ni;
	//call UartLog.logTxRx(DBG_FLAG, DBG_ER_FLAG, __LINE__, 0, 0, 0, -my_signal, -receiver_ni, sinr);
	if (sinr < SINR_THRESHOLD) {
		//call UartLog.logEntry(DBG_FLAG, DBG_TX_FLAG, __LINE__, sinr);
		//#warning disable my check
		return FALSE;
	}

	now = call LocalTime.get();
	for (i = 0; i < LINK_TABLE_SIZE; i++) {
		le = &linkTable[i];
		if (!le->valid)
			continue;
		// TODO: wrap around
		// expired, or not active; just a guess
		//if ((le->rx_timestamp + SNOOP_DURATION) < now)
		if (le->complete_timestamp < now)
			continue;
		// link here are considered active
		out_gain = call SignalMap.getOutboundGain(le->receiver);
		if (INVALID_GAIN == out_gain) {
			assert(le->receiver);
			// conservative
			//return FALSE;
			// node outside max tx range
			continue;
		}
		if (INVALID_DBM == le->ni || INVALID_DBM == le->rx_signal) {
			//call UartLog.logEntry(DBG_FLAG, DBG_TX_FLAG, __LINE__, 0);
			return FALSE;
		}
		my_interference = CC2420_DEF_RFPOWER_DBM_SCALED - out_gain;
		total_ni = call SignalMap.dbmSumU(le->ni, my_interference);
		sinr = le->rx_signal - total_ni;
		if (sinr < SINR_THRESHOLD) {
			//call UartLog.logEntry(DBG_FLAG, DBG_TX_FLAG, __LINE__, sinr);
			return FALSE;
		}
	}
	return TRUE;
}


// ---------------------------------------------------------------------
// cache management functions
// ---------------------------------------------------------------------
void initCache() {
	uint8_t i;
	link_nav_entry_t *le;
	
	for (i = 0; i < LINK_NAV_CACHE_SIZE; i++) {
		le = &linkNAVCache[i];
		le->valid = FALSE;
	}
}

uint8_t findCacheIdx(am_addr_t sender, am_addr_t receiver) {
	uint8_t i;
	link_nav_entry_t *le;
	
	for (i = 0; i < LINK_NAV_CACHE_SIZE; i++) {
		le = &linkNAVCache[i];
		if (le->valid) {
			if (le->sender == sender && le->receiver == receiver)
				return i;
		}
	}
	return i;
}

uint8_t findEmptyCacheIdx() {
	uint8_t i;
	link_nav_entry_t *le;
	
	for (i = 0; i < LINK_NAV_CACHE_SIZE; i++) {
		le = &linkNAVCache[i];
		if (!le->valid) {
			return i;
		}
	}
	return i;
}

uint8_t findMinCacheIdx() {
	uint8_t i;
	link_nav_entry_t *le;
	
	uint8_t min_nav_idx = 0;
	uint32_t min_nav = 0xFFFFFFFF;
	
	for (i = 0; i < LINK_NAV_CACHE_SIZE; i++) {
		le = &linkNAVCache[i];
		if (le->valid) {
			if (min_nav > le->link_nav) {
				min_nav = le->link_nav;
				min_nav_idx = i;
			}
		}
	}
	return min_nav_idx;
}

uint32_t maxCache() {
	uint8_t i;
	link_nav_entry_t *le;
	
	uint32_t max_nav = 0;
	
	for (i = 0; i < LINK_NAV_CACHE_SIZE; i++) {
		le = &linkNAVCache[i];
		if (le->valid) {
			if (max_nav < le->link_nav) {
				max_nav = le->link_nav;
			}
		}
	}
	return max_nav;
}

void printCache() {
	uint8_t i;
	link_nav_entry_t *le;
	
	for (i = 0; i < LINK_NAV_CACHE_SIZE; i++) {
		le = &linkNAVCache[i];
		if (le->valid) {
			dbg_clear("Cache", "<%u, %u>: %u\n", le->sender, le->receiver, le->link_nav);
		}
	}
	dbg_clear("Cache", "max: %u\n", maxCache());
}

void insertCache(am_addr_t sender, am_addr_t receiver, uint32_t link_nav) {
	uint8_t idx;
	link_nav_entry_t *le;
	
	// if found
	//		update
	// elif still empty slot
	//		initialize
	// elif earliest nav < link_nav
	//			replace earliest link
	idx = findCacheIdx(sender, receiver);
	if (idx < LINK_NAV_CACHE_SIZE) {
		le = &linkNAVCache[idx];
		le->link_nav = link_nav;
	} else {
		idx = findEmptyCacheIdx();
		if (idx < LINK_NAV_CACHE_SIZE) {
			le = &linkNAVCache[idx];
			le->valid = TRUE;
			le->sender = sender;
			le->receiver = receiver;
			le->link_nav = link_nav;
		} else {
			idx = findMinCacheIdx();
			le = &linkNAVCache[idx];
			if (le->link_nav < link_nav) {
				le->sender = sender;
				le->receiver = receiver;
				le->link_nav = link_nav;
			}
		}
	}
	//printCache();
}

// delete an entry
uint32_t deleteCache(am_addr_t sender, am_addr_t receiver) {
	uint8_t idx;
	link_nav_entry_t *le;

	idx = findCacheIdx(sender, receiver);
	if (idx < LINK_NAV_CACHE_SIZE) {
		le = &linkNAVCache[idx];
		le->valid = FALSE;
		return le->link_nav;
	}
	return 0;
	//printCache();
}

uint8_t cacheSize() {
	uint8_t i;
	link_nav_entry_t *le;
	uint8_t total = 0;
	
	for (i = 0; i < LINK_NAV_CACHE_SIZE; i++) {
		le = &linkNAVCache[i];
		if (le->valid) {
			total++;
		}
	}
	return total;
}

