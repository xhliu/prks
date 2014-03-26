// cache management functions
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

