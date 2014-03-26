/*
 * Xiaohui Liu (whulxh@gmail.com)
 * 12/28/2011
 */
 
interface SignalMap {
	//query the gain from the neighbor
	// return is scaled by 128
	// return: INVALID_GAIN if gain to the neighbor is unknown
	command uint16_t getInboundGain(am_addr_t nb);

	//query the gain to the neighbor
	command uint16_t getOutboundGain(am_addr_t nb);
}
