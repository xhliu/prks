/*
 * @author: Xiaohui Liu (whulxh@gmail.com)
 * @update: 4/6/2012
 */
 
interface IMac {
	command void enable();
	command void disable();
	command bool isEnabled();
	// virtual carrier sensing
	async command bool virtualCca();
}
