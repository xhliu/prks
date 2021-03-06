/*
 * @ author: Xiaohui Liu (whulxh@gmail.com)
 * @ updated: 10/02/2012 
 * @ description: to impose slot integrity, namely, a limit of rx, especially on CONTROL channel, in a slot to ensure everything finishes within it
 */
interface ForwarderInfo {
	// a new slot starts
	// @param is_data_channel: in data channel?
//	async event void slotStarted(bool is_data_channel);
	
	
	async command bool isForwarderEnabled();
	//async command bool isDataPending();
	//async command uint32_t getNextTxSlot();
}
