/*
 * @ author: Xiaohui Liu (whulxh@gmail.com)
 * 
 * @ updated: 	04/07/2012 03:23:55 PM 
 *				add inbound quality to compute directional quality
 * @ acknowledgement: based on 4bitle
 * @ description: link estimator for iMAC
 */
interface LinkEstimator {
	// get inbound link pdr for data
	command error_t getInDataPdr(am_addr_t neighbor, uint8_t *inquality, uint8_t *inquality_sample);
	// get inbound link quality for ACK
	command error_t getInAckPdr(am_addr_t neighbor, uint8_t *ack_quality, uint8_t *ack_quality_sample);
	// get outbound link quality
	//command uint8_t getOutLinkQuality(am_addr_t neighbor);
	/*
	 * signal when inbound data/ack pdr updated: ack updates either bidirectional pdr or outbound data pdr updates
	 * @param is_ack: ack pdr update?
	 */
	event error_t inLinkPdrUpdated(am_addr_t neighbor, bool is_ack);

	// called when an acknowledgement is received; sign of a successful data transmission; to update forward link quality
	command error_t txAck(am_addr_t neighbor);

	// called when an acknowledgement is not received; could be due to data pkt or acknowledgement loss; to update forward link quality
	command error_t txNoAck(am_addr_t neighbor);
	
	// force a neighbor into the table
	command error_t pinNeighbor(am_addr_t neighbor);
}


