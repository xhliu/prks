/*
 * @ author: Xiaohui Liu (whulxh@gmail.com)
 * 
 * @ updated: 04/05/2012 08:56:00 PM 
 */
interface IMACController {
	// compute control output based on input
	command int32_t controller(uint8_t link_pdr, uint8_t link_pdr_sample, uint8_t reference_pdr);
}
