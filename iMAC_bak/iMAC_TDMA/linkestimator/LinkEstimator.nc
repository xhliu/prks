/*
 * @ author: Xiaohui Liu (whulxh@gmail.com)
 * 
 * @ updated: 	04/07/2012 03:23:55 PM 
 *				add inbound quality to compute directional quality
 * @ acknowledgement: based on 4bitle
 * @ description: link estimator for iMAC
 */
#include "LinkEstimator.h"
interface LinkEstimator {
	async command neighbor_table_entry_t *getNeighborTable();
	// get inbound link pdr for data
	async command error_t getInDataPdr(am_addr_t neighbor, uint8_t *inquality, uint8_t *inquality_sample, uint8_t *inquality_version);
	// get inbound link quality for ACK
	async command error_t getInAckPdr(am_addr_t neighbor, uint8_t *ack_quality, uint8_t *ack_quality_sample);
	// get outbound link quality
	//command uint8_t getOutLinkQuality(am_addr_t neighbor);
	// update outbound link quality to nb
	async command void updateNeighborOutQuality(am_addr_t nb, uint8_t outquality, uint8_t outquality_version);
	
	//async command error_t updateNeighborTableEstD0(am_addr_t neighbor);
	/*
	 * signal when inbound data/ack pdr updated: ack updates either bidirectional pdr or outbound data pdr updates
	 * @param is_sender: ack pdr update?
	 */
	async event error_t inLinkPdrUpdated(am_addr_t neighbor, bool is_sender);

	// called when an acknowledgement is received; sign of a successful data transmission; to update forward link quality
	async command error_t txAck(am_addr_t neighbor);

	// called when an acknowledgement is not received; could be due to data pkt or acknowledgement loss; to update forward link quality
	async command error_t txNoAck(am_addr_t neighbor);
	
	// force a neighbor into the table
	async command error_t pinNeighbor(am_addr_t neighbor);
	
//	async command uint16_t getVerMatchCnt();
//	async command uint16_t getVerMismatchCnt();
}


