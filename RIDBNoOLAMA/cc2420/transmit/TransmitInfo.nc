/*
 * @ author: Xiaohui Liu (whulxh@gmail.com)
 * @ updated: 10/02/2012 
 * @ description: to impose slot integrity, namely, a limit of rx, especially on CONTROL channel, in a slot to ensure everything finishes within it
 */
interface TransmitInfo {
	// tx/rx on CONTROL channel
	async command bool isSender();
	async command uint32_t getTime32(uint16_t time);
}
