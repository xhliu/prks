/*
 * @ author: Xiaohui Liu (whulxh@gmail.com)
 * 
 * @ updated: 04/05/2012 08:56:00 PM 
 */
#include "IMACController.h"

interface IMACController {
	// compute control output based on input
//	async command int32_t controller(uint8_t link_pdr, uint8_t link_pdr_sample, uint8_t reference_pdr);
	async command local_link_er_table_entry_t *getLocalLinkERTable();
	async command link_er_table_entry_t *getLinkERTable();

	async command bool isTxSlot(uint32_t current_slot);
	async command bool isRxSlot(uint32_t current_slot);
	//async command uint32_t getNextTxSlot();
	async command void updateNextSlot(am_addr_t from, bool is_from_sender, uint32_t next_slot);
	//async command void updateContention(uint8_t);

	//async command uint8_t findMyLinkERTableIdx();
	async command uint8_t findMyLinkLocalIdx();
	async command uint8_t findLinkERTableIdx(am_addr_t sender, am_addr_t receiver);
	async command void clearDataPending(am_addr_t from);
	
	async command uint8_t getNbERVer(am_addr_t nb, bool is_sender);
	async command error_t sampleNI(message_t *msg);
	
	async command int16_t dbmDiffU(int16_t x, int16_t y);
	
	async command uint8_t loadLinkER(link_er_footer_t *er_footer);
	async command void updateLinkERTable(link_er_footer_t *footer, uint8_t size, am_addr_t from, uint16_t seqno_);
	
	async command uint8_t getLinkPdrReq();
	async command void setLinkPdrReq(uint8_t new_req);
	// get interference threshold
	// local ER: not necessary anymore since merged into linkERTable
//	async command error_t getLocalInterferenceThreshold(am_addr_t nb, bool is_sender, int16_t *interference_threshold);
	// get the ER of a link; not local
//	async command error_t getInterferenceThreshold(am_addr_t sender, am_addr_t receiver, int16_t *tx_interference_threshold, int16_t *rx_interference_threshold);
//	// is ER of any link incident w/ me changed; used to control ctrl pkt frequency
//	async command bool isERChangeSent();
//	// called after ctrl pkt sent
//	async command void setERChangeSent();
}
