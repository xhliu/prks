/*
 * Xiaohui Liu (whulxh@gmail.com)
 * 12/28/2011
 */
 
interface SignalMap {
	// query the gain from the neighbor
	// return is scaled by 128
	// return: INVALID_GAIN if gain to the neighbor is unknown
	command int16_t getInboundGain(am_addr_t nb);

	// query the gain to the neighbor
	command int16_t getOutboundGain(am_addr_t nb);
	
	// am I in the ER of the neighbor	
	command bool inExRegion(am_addr_t nb, int16_t intrf_thres);
	
	// get interference threshold
	command error_t getInterferenceThresholdPowerLevelNI(am_addr_t nb, bool is_rts, int16_t *inteference_threshold, int16_t *min_interference_threshold, uint8_t *power_level, int16_t *node_ni);
	
	// initialize ER of a link
	command void initLinkExRegion(am_addr_t nb, bool is_tx);
	
	// check whether a neighbor is in my outbound ER based on min_interference_threshold
	command void updateOutboundER(am_addr_t nb, int16_t min_interference_threshold, int16_t node_ni, uint8_t type, uint16_t seqno);
	
	// update NI for a node and every link when a new NI sample arrives
	command void updateNI(bool is_link_ni, am_addr_t nb, bool is_ack, int16_t post_rss);
	
	// for debug
	command void printSignalMap();
	
	command int16_t dbmSumU(int16_t x, int16_t y);
}
