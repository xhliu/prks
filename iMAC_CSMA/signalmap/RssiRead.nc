/* *
 * @author: Xiaohui Liu (whulxh@gmail.com)
 * @created: 1/10/2012
 * @description: async reading of RSSI
 */

interface RssiRead {
	// return SUCCESS if RSSI can be read immediately, i.e., SPI bus is idle
	async command error_t read(uint16_t *buf);
}
