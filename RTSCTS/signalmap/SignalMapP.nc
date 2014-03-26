/* *
 * @ author: 	Xiaohui Liu (whulxh@gmail.com) 
 * @ updated: 	12/28/2011
 * @ updated: 	04/07/2012 03:38:54 PM
 *				integrate into iMAC
 *				04/08/2012 03:00:55 PM 
 *				add current noise + interference; sort signal map
 *				04/23/2012 
 *				add bidirectional power ctrl
 * @ description: implement signal map, which offers signal gain from and to a neighbor
 */
#include "SignalMap.h"
#include "../beaconing/IMACBeacon.h"
#include "../IMac.h"

module SignalMapP {
	provides {
		interface AMSend as Send;
		interface Receive;
		interface Packet;
		
		interface Init;
		
		interface SignalMap;
	};
	
	uses {
		interface AMSend as SubSend;
		interface Receive as SubReceive;
		interface Packet as SubPacket;
		interface AMPacket as SubAMPacket;
	#ifndef TOSSIM
		interface CC2420Packet;
		//interface Read<uint16_t> as ReadRssi;
	#endif
		interface LinkEstimator;
		interface IMACController as Controller;
		interface IMac;
		interface Timer<TMilli> as OutboundERTimer;
		//interface Timer<TMilli> as NISampleTimer;
		interface LocalTime<TMilli>;
		interface UartLog;
	};
}

implementation {

am_addr_t my_ll_addr;

// signal map
sm_entry_t signalMap[SM_SIZE];

/*
 * mean interference + noise at a node
 * used when compute RTS/CTS tx power at others and to compute deltaI in dBm from dB locally
 * both beacon and RTS/CTS diffuse this information to maximize coverage and speed up
 */
dbm_t node_I;

// previous max ER border
int16_t prev_max_er_border_idx;
// how many times left to retain the power level corresponding to the previous maximal ER
int16_t power_level_reuse_cnt;

/*
 * 3 types of tx power: 1) data/ack; 2) signal map beacon; 3) RTS/CTS
 */
// default power of signal map beacon, in dBm; to compute link attenuation
int16_t BEACON_SM_POWER_DBM;
// default power of data, in dBm; to compute interference of nodes in ER
int16_t CC2420_DEF_RFPOWER_DBM;
int16_t CC2420_DEF_RFPOWER_DBM_SCALED;

// if there is not enough room in the packet to put all the signal map entries, in order to do round robin we need to remember which entry
// we sent in the last beacon
int16_t prevSentIdx = 0;
uint16_t seqno;

// ===========================================================
// forward declarations
// ===========================================================
// signal map table management functions
void initSignalMap();
int16_t findIdx(am_addr_t nb);
int16_t findEmptyIdx();
int16_t getSignalMapSize();
int16_t maxERBorderIdx();

inline int16_t level2Power(uint8_t power_level);
inline uint8_t power2Level(int16_t power);

inline int16_t dbmSumU(int16_t x, int16_t y);
dbm_t dbmSumS(dbm_t x, dbm_t y);
dbm_t dbmDiffS(dbm_t x, dbm_t y);
dbm_t dbmWeightedSumS(dbm_t x, dbm_t y);

uint16_t calcInboundGain(int16_t noise, int16_t rssi, uint8_t tx_power_level);


// get the link estimation header in the packet
sm_header_t* getHeader(message_t* m) {
	return (sm_header_t*)call SubPacket.getPayload(m, sizeof(sm_header_t));
}

// get the signal map footer (neighbor entries) in the packet
// @param len: payload length to upper layer
sm_footer_t* getFooter(message_t* m, uint8_t len) {
	// To get a footer at offset "len", the payload must be len + sizeof large.
	return (sm_footer_t*)(len + (uint8_t *)call Packet.getPayload(m, len + sizeof(sm_footer_t)));
}

// initialize the link estimator
command error_t Init.init() {
	initSignalMap();
	
	node_I.sign = 0;
	node_I.abs = INVALID_DBM;

	// precompute constants to save processing later
	BEACON_SM_POWER_DBM = level2Power(BEACON_SM_POWER_LEVEL);
	CC2420_DEF_RFPOWER_DBM = level2Power(CC2420_DEF_RFPOWER);
	CC2420_DEF_RFPOWER_DBM_SCALED = CC2420_DEF_RFPOWER_DBM << SCALE_L_SHIFT_BIT;
	prev_max_er_border_idx = -1;
	power_level_reuse_cnt = 0;
	
	my_ll_addr = call SubAMPacket.address();
	
	call OutboundERTimer.startPeriodic(AGING_PERIOD);
	//call NISampleTimer.startPeriodic(NI_SAMPLE_PERIOD);
	dbg("LE", "Link estimator init %d, %d, %d\n", BEACON_SM_POWER_DBM, CC2420_DEF_RFPOWER_DBM, CC2420_DEF_RFPOWER_DBM_SCALED);
	return SUCCESS;
}

/* *
 * * Interface SignalMap
 * */
// query the gain from the neighbor
// return INVALID_GAIN if neighbor is not found or neighbor found but gain unknown
command int16_t SignalMap.getInboundGain(am_addr_t nb) {
	int16_t idx;
	sm_entry_t *se;
	
	idx = findIdx(nb);
	if (idx < SM_SIZE) {
		se = &signalMap[idx];
		return se->inbound_gain;
	} else {
		return INVALID_GAIN;
	}
}

// query the gain to the neighbor
command int16_t SignalMap.getOutboundGain(am_addr_t nb) {
	int16_t idx;
	sm_entry_t *se;
	
	idx = findIdx(nb);
	if (idx < SM_SIZE) {
		se = &signalMap[idx];
		return se->outbound_gain;
	} else {
		return INVALID_GAIN;
	}
}

//--------------------------------------------------------------------------------------------------------------
//  am I in the ER of the neighbor
//  if ER property unknown, be conservative and assume it is in ER to ensure no unexpected interference from it
//  bidirectional: this can happen if either I can interfere w/ the neighbor or vice versa
//  sender and receiver of a link can be in its own ER using this command, but address recognition when setting
//	NAV makes this not mater
//---------------------------------------------------------------------------------------------------------------
command bool SignalMap.inExRegion(am_addr_t nb, int16_t interference_threshold) {
	int16_t i, idx;
	sm_entry_t *se;
	
	idx = findIdx(nb);
	if (idx >= SM_SIZE) {
		return TRUE;
	}
	
	se = &signalMap[idx];
	if (INVALID_GAIN == se->outbound_gain) {
		return TRUE;
	}
	
	// outbound
	// data is transmitted using power level CC2420_DEF_RFPOWER
	if ((CC2420_DEF_RFPOWER_DBM - (se->outbound_gain >> SCALE_L_SHIFT_BIT)) >= interference_threshold)
		return TRUE;
	
	// inbound
	// be conservative to ensure the neighbor is within none of my ERs
	for (i = 0; i < SM_SIZE; i++) {
		// exclude the neighbor itself
		if (i == idx)
			continue;
		
		se = &signalMap[i];
		if (se->valid) {
			// within tx or rx ER
			if (idx <= se->tx_er_border_idx || idx <= se->rx_er_border_idx)
				return TRUE;
		}
	}
	// only out of ER if passing all tests
	return FALSE;
}

// check whether a neighbor is in my outbound ER based on min_interference_threshold and update its NI
// called when RTS/CTS is received from IMACForwarderP$SubReceive$receive
command void SignalMap.updateOutboundER(am_addr_t nb, int16_t min_interference_threshold, int16_t node_i_, uint8_t type, uint16_t seqno_) {
	int16_t idx;
	sm_entry_t *se;
	
	idx = findIdx(nb);
	if (idx >= SM_SIZE)
		return;
	se = &signalMap[idx];
	se->is_in_outbound_er = ((CC2420_DEF_RFPOWER_DBM - (se->outbound_gain >> SCALE_L_SHIFT_BIT)) >= min_interference_threshold);
	// refresh age
	se->age = MAX_AGE;
	
	// pkt can be corrupted; filter some, if not all
	if (node_i_ > MIN_NODE_I || node_i_ < MAX_NODE_I) {
		se->node_i = node_i_;
		//call UartLog.logTxRx(DBG_FLAG, DBG_ER_FLAG, 8, MIN_NODE_i < 0, MIN_NODE_i, nb, type, seqno_, node_i_);
	}
}

// # of rounds to kick out a neighbor out of my outbound ER
event void OutboundERTimer.fired() {
	int16_t i;
	sm_entry_t *se;
	
	for (i = 0; i < SM_SIZE; i++) {
		se = &signalMap[i];
		if (se->valid) {
			if (se->age > 0) {
				se->age--;
			} else {
				se->is_in_outbound_er = FALSE;
			}
		}		
	}
}

/*
 * @param nb: to which neighbor's exclusion region
 * @param is_sender: is it RTS or CTS? 
 * @param interference_threshold: returned interference threshold, i.e., interference at the boundary of exclusion region, in dBm
 * @param power_level: the minimal tx power to reliably reach every node in the ER, i.e., SM[0..er_border_idx]
 * @return: SUCCESS only if both interference threshold and RTS/CTS tx power level are correctly obtained
 * runtime sanity check every time link gain is used; return FAIL if any is invalid
 * called from IMACControllerP$Controller$interferenceThreshold
 */
// wrapper
inline error_t assertGain(bool is_tx, sm_entry_t *nb_se, int16_t gain, int16_t idx, int16_t delta);
// wrapper
inline void updateBorder(bool is_tx, sm_entry_t *nb_se, int16_t i);
// based on deltaI, update ER
error_t updateER(int16_t idx, bool is_tx, dbm_t delta_I_d);

// look up ER and compute RTS/CTS signal power; piggyback node NI for speed up diffusion
command error_t SignalMap.getInterferenceThresholdPowerLevelNI(am_addr_t nb, bool is_sender, int16_t *interference_threshold, int16_t *min_interfererence_threshold, uint8_t *power_level, int16_t *node_i_) {
	int16_t i, idx, er_border_idx;
	uint8_t level;
	uint8_t link_pdr, link_pdr_sample;
	sm_entry_t *se;
	int16_t outbound_gain;
	int16_t nb_i, tx_signal, max_tx_signal, min_threshold, rx_i;
	
	idx = findIdx(nb);
	if (idx >= SM_SIZE) {
		//call SignalMap.printSignalMap();
		//call UartLog.logEntry(DBG_FLAG, DBG_ER_FLAG, 4, getSignalMapSize());
		return FAIL;
	}
	se = &signalMap[idx];
	rx_i = se->rx_I.abs / 128;
	// ER
	if (is_sender) {
		*interference_threshold = se->tx_interference_threshold;
		er_border_idx = se->tx_er_border_idx;
	} else {
		*interference_threshold = se->rx_interference_threshold;
		er_border_idx = se->rx_er_border_idx;
	}
	
	// node NI
	*node_i_ = (node_I.abs >> SCALE_L_SHIFT_BIT);
	
	// tx power
	//call UartLog.logTxRx(DBG_FLAG, DBG_BI_ER_FLAG, 0, 0, 0, 0, er_border_idx, power_level_reuse_cnt, prev_max_er_border_idx);
	// hysteresis
	if (power_level_reuse_cnt > 0) {
		// max ER can increase during the hysteresis
		if (er_border_idx < prev_max_er_border_idx)
			er_border_idx = prev_max_er_border_idx;
		power_level_reuse_cnt--;
	}
	// to reliably reach sender/receiver
	// initialize w/ any small enough power
	max_tx_signal = -10000;
	// special care for sender/receiver per se
	// i.e., RTS reaches receiver and CTS reaches sender
	if (se->node_i != INVALID_DBM && se->outbound_gain != INVALID_GAIN) {
		nb_i = se->node_i;
		outbound_gain = (se->outbound_gain >> SCALE_L_SHIFT_BIT);
		// min tx signal to ensure SINR at the sender/receiver is above the critical threshold
		max_tx_signal = outbound_gain + nb_i + RELIABLE_SNR_THRESHOLD;
	} else {
		;//call UartLog.logTxRx(DBG_FLAG, DBG_ER_FLAG, 0, 0, 0, idx, nb, se->node_i, se->outbound_gain);
	}

	// to reliably reach every node in the ER, i.e., SM[0..er_border_idx] or in my outbound ER
	min_threshold = 10000;
	for (i = 0; i < SM_SIZE; i++) {
		se = &signalMap[i];
		if (!se->valid)
			continue;
		// TODO: initialize threshold to be valid
		if (min_threshold > se->tx_interference_threshold)
			min_threshold = se->tx_interference_threshold;
		if (min_threshold > se->rx_interference_threshold)
			min_threshold = se->rx_interference_threshold;
		
		// in neither ER or outbound ER
		if (i > er_border_idx && !se->is_in_outbound_er)
			continue;
		
		if (INVALID_DBM == se->node_i || INVALID_GAIN == se->outbound_gain) {
			//call UartLog.logTxRx(DBG_FLAG, DBG_ER_FLAG, 1, 0, 0, idx, se->nb, se->node_i, se->outbound_gain);
			// TODO: just skip after making the occurrence low
			continue;
		}
		nb_i = se->node_i;
		outbound_gain = (se->outbound_gain >> SCALE_L_SHIFT_BIT);
		// min tx signal to ensure SINR at the neighbor is above the critical threshold
		tx_signal = outbound_gain + nb_i + RELIABLE_SNR_THRESHOLD;
		//if (tx_signal > 1000 || tx_signal < -100)
			//call UartLog.logTxRx(DBG_FLAG, DBG_ER_FLAG, 6, 0, 0, i, se->nb, outbound_gain, nb_i);
		if (max_tx_signal < tx_signal)
			max_tx_signal = tx_signal;
	}

	*min_interfererence_threshold = min_threshold;
	// convert power into power level
	level = power2Level(max_tx_signal);
	// use power level no smaller than data
	level = (level > CC2420_DEF_RFPOWER) ? level : CC2420_DEF_RFPOWER;
	// TODO: debug
	//level = 31;
	*power_level = level;
	call LinkEstimator.getInDataPdr(nb, &link_pdr, &link_pdr_sample);
	if (!is_sender)
		call UartLog.logTxRx(DBG_FLAG, DBG_ER_FLAG, 55, nb, -rx_i, level, -*node_i_, er_border_idx + 1, link_pdr);
	return SUCCESS;
}

// new pdr sample comes, execute the controller
// @param is_init: special care for initialization
error_t execController(am_addr_t nb, bool is_sender, bool is_init);

// new inbound data/ack pdr arrives
// ER is updated here, not on per pkt basis
// @param is_sender: inbound ACK reliability updated or outbound data
event error_t LinkEstimator.inLinkPdrUpdated(am_addr_t nb, bool is_sender) {
	return execController(nb, is_sender, FALSE);
}

error_t execController(am_addr_t nb, bool is_sender, bool is_init) {
	int16_t idx;
	error_t ret;
	sm_entry_t *se;

	uint8_t link_pdr, link_pdr_sample, reference_pdr;
	// scaled
	int32_t next_i, delta_i_d_dB;
	// encapsulation of dbms; using upper case I to differentiate
	dbm_t current_I, prev_I, delta_I, delta_I_d, delta_I_u, mean_delta_I_u, next_I;
	//uint32_t start_time = call LocalTime.get();
	
	idx = findIdx(nb);
	if (idx >= SM_SIZE) {
		return FAIL;
	}
	se = &signalMap[idx];
	
	// RTS uses inbound ACK pdr while CTS uses inbound data pdr
	if (is_sender) {
		if (!is_init && 0 == se->tx_I.sign)
			// no sample, wait till next update
			return FAIL;
		call LinkEstimator.getInAckPdr(nb, &link_pdr, &link_pdr_sample);
		reference_pdr = REFERENCE_ACK_PDR;
		prev_I = se->tx_prev_I;
		current_I = se->tx_I;
		delta_I_d = se->tx_delta_I_d;
		mean_delta_I_u = se->tx_mean_delta_I_u;
		// update I(t)
		se->tx_prev_I = se->tx_I;
		// reset for next interval; sign == 0 meaning no sample yet in the current interval
		se->tx_I.sign = 0;
		//i_cnt = se->tx_i_cnt;
		//se->tx_i_cnt = 0;
	} else {
		if (!is_init && 0 == se->rx_I.sign)
			// no sample, wait till next update
			return FAIL;
		call LinkEstimator.getInDataPdr(nb, &link_pdr, &link_pdr_sample);
		reference_pdr = REFERENCE_DATA_PDR;
		prev_I = se->rx_prev_I;
		current_I = se->rx_I;
		delta_I_d = se->rx_delta_I_d;
		mean_delta_I_u = se->rx_mean_delta_I_u;
		// update I(t)
		se->rx_prev_I = se->rx_I;
		// reset for next interval
		se->rx_I.sign = 0;
	}
	
	if (link_pdr != 0) {
		if (mean_delta_I_u.abs != INVALID_DBM) {
			delta_I = dbmDiffS(current_I, prev_I);
			delta_I_u = dbmDiffS(delta_I, delta_I_d);
			//mean_delta_i_u = (ALPHA * mean_delta_i_u + (10 - ALPHA) * delta_i_u) / 10;
			mean_delta_I_u = dbmWeightedSumS(mean_delta_I_u, delta_I_u);
			//call UartLog.logTxRx(DBG_FLAG, DBG_CALC_FLAG, nb, -current_i / 128, -prev_i / 128, -delta_I.abs / 128, -delta_I_d.abs / 128, -delta_I_u.abs / 128, -mean_delta_I_u.abs / 128);
			
			delta_i_d_dB = call Controller.controller(link_pdr, link_pdr_sample, reference_pdr);
			next_i = current_I.abs + delta_i_d_dB;
			next_I.sign = 1;
			next_I.abs = next_i;
			// delta_i_d = next_i - mean_delta_i_u - current_i
			delta_I_d = dbmDiffS(next_I, mean_delta_I_u);
			delta_I_d = dbmDiffS(delta_I_d, current_I);
			if (is_sender) {
				se->tx_delta_I_d = delta_I_d;
				se->tx_mean_delta_I_u = mean_delta_I_u;
			} else {
				se->rx_delta_I_d = delta_I_d;
				se->rx_mean_delta_I_u = mean_delta_I_u;
			}
			if (!is_sender)
				call UartLog.logTxRx(DBG_FLAG, DBG_CONTROLLER_FLAG, nb, is_sender, link_pdr, -current_I.abs / 128, delta_I.sign * delta_I.abs / 128, delta_I_d.sign * delta_I_d.abs / 128, delta_I_u.sign * delta_I_u.abs / 128);
		} else {
			// fix ER to initialize mean_delta_i_u, making (delta_i_d == 0)
			delta_I_d.sign = 0;
			delta_I_d.abs = 0;
			// delta_i_u == delta_i
			//if (current_I.abs != INVALID_DBM && prev_I.abs != INVALID_DBM) {
			if (current_I.sign != 0 && prev_I.sign != 0) {
				delta_I = dbmDiffS(current_I, prev_I);
				// (delta_i_d == 0) and initialize
				mean_delta_I_u = delta_I;
				if (is_sender) {
					se->tx_delta_I_d = delta_I_d;
					se->tx_mean_delta_I_u = mean_delta_I_u;
				} else {
					se->rx_delta_I_d = delta_I_d;
					se->rx_mean_delta_I_u = mean_delta_I_u;
				}
			}
		}
	} else {
		// TODO: this should only happen when pdr is uninitialized in iMAC simple test since no BSL issue here
		// if we make sure the other end of the active link is in my neighbor table: size larger than network size
		// fix ER to initialize link PDR
		delta_I_d.sign = 0;
		delta_I_d.abs = 0;
	}
	// initialize ER w/ communication range
	if (is_init)
		updateBorder(is_sender, se, idx);
	
	// is_sender == is_tx
	ret = updateER(idx, is_sender, delta_I_d);
	//call UartLog.logEntry(DBG_FLAG, DBG_EXEC_TIME_FLAG, 0, call LocalTime.get() - start_time);
	return ret;
}

// wrapper
inline error_t assertGain(bool is_tx, sm_entry_t *nb_se, int16_t gain, int16_t idx, int16_t delta) {
	if (INVALID_GAIN == gain) {
		return FAIL;
	} else {
		if (is_tx) {
			nb_se->tx_interference_threshold = CC2420_DEF_RFPOWER_DBM - (gain >> SCALE_L_SHIFT_BIT) + delta;
		} else {
			nb_se->rx_interference_threshold = CC2420_DEF_RFPOWER_DBM - (gain >> SCALE_L_SHIFT_BIT) + delta;
		}
		return SUCCESS;
	}
}

inline void updateBorder(bool is_tx, sm_entry_t *nb_se, int16_t i) {
	//call UartLog.logTxRx(DBG_FLAG, DBG_ER_FLAG, 2, 0, 0, 0, is_tx, nb_se->nb, i);
	if (is_tx)
		nb_se->tx_er_border_idx = i;
	else
		nb_se->rx_er_border_idx = i;
}

// based on delta_I_d, update ER
error_t updateER(int16_t idx, bool is_tx, dbm_t delta_I_d) {
	// signed to avoid inf loop due to i >= 0
	int16_t i, sm_size, er_border_idx;
	// point to the entry of the neighbor
	sm_entry_t *nb_se;
	sm_entry_t *se;
	int32_t delta_i_d;
	// scaled; otherwise dBm addition, substraction may cause great precision loss
	int32_t total_interference;
	
	nb_se = &signalMap[idx];
	// current ER border
	if (is_tx)
		er_border_idx = nb_se->tx_er_border_idx;
	else
		er_border_idx = nb_se->rx_er_border_idx;	
	
	sm_size = getSignalMapSize();
	
	delta_i_d = delta_I_d.abs;
	// enlarge exclusion region to reduce interference
	if (delta_I_d.sign < 0) {
		// cannot enlarge further; already largest
		if ((er_border_idx + 1) >= sm_size) {
			se = &signalMap[er_border_idx];
			return assertGain(is_tx, nb_se, se->inbound_gain, idx, 0);
		}
		
		total_interference = 0;
		for (i = er_border_idx + 1; i < sm_size; i++) {
			se = &signalMap[i];
			if (INVALID_GAIN == se->inbound_gain) {
				//call UartLog.logTxRx(DBG_FLAG, DBG_SM_FLAG, 3, i, er_border_idx, sm_size, se->nb, 0, 0);
				return FAIL;
			}
			total_interference = dbmSumU(total_interference, CC2420_DEF_RFPOWER_DBM - se->inbound_gain);
			if (total_interference >= delta_i_d)
				break;
		}
		// even the entire SM is not sufficient for the exclusion region; try the largest
		if (i >= sm_size)
			i--;
		// update boundary
		updateBorder(is_tx, nb_se, i);
		se = &signalMap[i];
		return assertGain(is_tx, nb_se, se->inbound_gain, idx, 0);
	} else if (delta_I_d.sign > 0) {
		// shrink exclusion region to increase interference

		// cannot shrink further; already empty
		if (-1 == er_border_idx) {
			se = &signalMap[0];
			// add DELTA so even "closest" neighbor is excluded
			return assertGain(is_tx, nb_se, se->inbound_gain, idx, DELTA);
		}
		// max ER before shrinking
		prev_max_er_border_idx = maxERBorderIdx();
		
		total_interference = 0;
		// assert(er_border_idx >= 0)
		// start from er_border_idx, not er_border_idx - 1, bcoz it is within ER
		for (i = er_border_idx; i >= 0; i--) {
			se = &signalMap[i];
			if (INVALID_GAIN == se->inbound_gain) {
				//call UartLog.logTxRx(DBG_FLAG, DBG_SM_FLAG, 4, i, er_border_idx, sm_size, se->nb, 0, 0);
				return FAIL;
			}
			total_interference = dbmSumU(total_interference, CC2420_DEF_RFPOWER_DBM - se->inbound_gain);
			if (total_interference >= delta_i_d)
				break;
		}
		
		// ER excludes the current neighbor
		// only if it exists
		if (i > -1)
			i--;
		
		// to make PDR not below requirement: maximally shrink the ER but still <= deltaI
		if (total_interference > delta_i_d)
			i++;
		
		// update boundary
		updateBorder(is_tx, nb_se, i);
		// max ER shrinks
		if (er_border_idx == prev_max_er_border_idx)
			if (maxERBorderIdx() < prev_max_er_border_idx)
				power_level_reuse_cnt = POWER_LEVEL_REUSE_CNT;
			
		if (i > -1) {
			se = &signalMap[i];
			return assertGain(is_tx, nb_se, se->inbound_gain, idx, 0);
		} else {
			// empty ER
			se = &signalMap[0];
			return assertGain(is_tx, nb_se, se->inbound_gain, idx, DELTA);
		}
	} else {
		// maintain current exclusion region
		if (er_border_idx > -1) {
			se = &signalMap[er_border_idx];
			return assertGain(is_tx, nb_se, se->inbound_gain, idx, 0);
		} else {
			// empty ER
			se = &signalMap[0];
			return assertGain(is_tx, nb_se, se->inbound_gain, idx, DELTA);
		}
	}
}

/* *
 * * Interface Send
 * */
// slap the header and footer before sending the message
// @param power_level: which power level is used to transmit the packet
// add the header footer in the packet. Call iust before sending the packet
uint8_t addLinkEstHeaderAndFooter(message_t *msg, uint8_t len);

command error_t Send.send(am_addr_t addr, message_t* msg, uint8_t len) {
	uint8_t newlen;
	newlen = addLinkEstHeaderAndFooter(msg, len);
	dbg("LI", "%s packet of length %hhu became %hhu\n", __FUNCTION__, len, newlen);
	return (call SubSend.send(addr, msg, newlen));
}

// add the header footer in the packet. Call iust before sending the packet
uint8_t addLinkEstHeaderAndFooter(message_t *msg, uint8_t len) {
	int16_t i, j, k;
	uint8_t maxEntries, newPrevSentIdx;
	uint8_t newlen;
	sm_header_t * ONE hdr;
	sm_footer_t * ONE footer;
	sm_entry_t *se;

	hdr = getHeader(msg);
	footer = getFooter(msg, len);	
	maxEntries = (call SubPacket.maxPayloadLength() - len - sizeof(sm_header_t)) / sizeof(sm_footer_t);
	dbg("LI", "Max payload is: %d, maxEntries is: %d\n", call SubPacket.maxPayloadLength(), maxEntries);

	// add footer
	j = 0;
	newPrevSentIdx = 0;
	for (i = 0; i < SM_SIZE && j < maxEntries; i++) {
	  	k = (prevSentIdx + i + 1) % SM_SIZE;
	  	se = &signalMap[k];
	  	
	  	if (se->valid) {
	  		footer[j].nb = se->nb;
	  		footer[j].inbound_gain = se->inbound_gain;
	  		j++;
	  		newPrevSentIdx = k;
	  	}
	}
	prevSentIdx = newPrevSentIdx;
	
	// add header
	// signal map is built using beacon, transmitted using maximal power
	// TODO: consider to convert to dBm to save processing
	hdr->power_level = BEACON_SM_POWER_LEVEL;
	hdr->node_i = (node_I.abs >> SCALE_L_SHIFT_BIT);
	hdr->footer_entry_cnts = j;
	hdr->seqno = seqno++;
	newlen = sizeof(sm_header_t) + len + j * sizeof(sm_footer_t);
/*
	if (TOS_NODE_ID == 0) {
		dbg("LITest", "packet %hu, len %hu, %hu entries: ", seqno, newlen, hdr->footer_entry_cnts);
		for (i = 0; i < j; i++)
			dbg_clear("LITest", "<%hu, %d> ", footer[i].nb, footer[i].inbound_gain / 128);
		dbg_clear("LITest", "\n");
	}
*/
	return newlen;
}


// done sending the message that originated by the user of this component
event void SubSend.sendDone(message_t* msg, error_t error ) {
	//if (SUCCESS == error)
		//call UartLog.logEntry(DBG_FLAG, DBG_SM_FLAG, 1, getHeader(msg)->footer_entry_cnts);
	signal Send.sendDone(msg, error);
}

// cascade the calls down
command uint8_t Send.cancel(message_t* msg) {
	return call SubSend.cancel(msg);
}

command uint8_t Send.maxPayloadLength() {
	return call Packet.maxPayloadLength();
}

command void* Send.getPayload(message_t* msg, uint8_t len) {
	return call Packet.getPayload(msg, len);
}


// new messages are received here
// update the signal map with the header and footer in the message, then signal the user of this component
void processReceivedMessage(message_t *msg, void *payload, uint8_t len);
void updateSignalMap(am_addr_t nb, int16_t in_gain, int16_t out_gain, int16_t node_i_);
void printSignalMap();
void sortSignalMap(int16_t idx);

event message_t* SubReceive.receive(message_t* msg, void* payload, uint8_t len) {
	//uint32_t start_time = call LocalTime.get();
	dbg("LI", "Received upper packet. Will signal up\n");
	processReceivedMessage(msg, payload, len);
	//call UartLog.logEntry(DBG_FLAG, DBG_EXEC_TIME_FLAG, 1, call LocalTime.get() - start_time);
	return signal Receive.receive(msg, call Packet.getPayload(msg, call Packet.payloadLength(msg)), call Packet.payloadLength(msg));
}

// called when signal map generated packet or packets from upper layer that are wired to pass through
// signal map is received
void processReceivedMessage(message_t *msg, void *payload, uint8_t len) {
	int16_t i;
	uint8_t tx_power_level;
	uint16_t val;
	// signal strength right b4 end of pkt reception
	int16_t pre_rss;
	// signal strength right after end of pkt reception	
	int16_t post_rss;
	int16_t in_gain = INVALID_GAIN;
	int16_t out_gain = INVALID_GAIN;
		
	sm_header_t* hdr = getHeader(msg);
	sm_footer_t *footer = getFooter(msg, call Packet.payloadLength(msg));
	
	uint8_t footer_entry_cnts = hdr->footer_entry_cnts;
	am_addr_t nb = call SubAMPacket.source(msg);
/*
	if (nb == 0 && TOS_NODE_ID == 1) {
		dbg("LITest", "packet %hu, len %hu, %hu entries: ", hdr->seqno, len, hdr->footer_entry_cnts);
		for (i = 0; i < hdr->footer_entry_cnts; i++)
			dbg_clear("LITest", "<%hu, %d> ", footer[i].nb, footer[i].inbound_gain / 128);
		dbg_clear("LITest", "\n");
	}
*/	
	// sample inbound gain from this packet
	tx_power_level = hdr->power_level;
#ifdef TOSSIM
	// simulate signal strengths
	pre_rss = -70 + nb % 10;
	val = 0;
	if (TRUE) {
#else
	// int16_t, not int8_t, to prevent underflow
	pre_rss = (int16_t)call CC2420Packet.getRssi(msg) - 45;
	val = call CC2420Packet.getRssiIdle(msg);
	// valid
	if (val != INVALID_RSSI) {
#endif
		// - 127 - 45
		post_rss = (int16_t)val - 172;
		call SignalMap.updateNI(FALSE, 0, FALSE, post_rss);
		
		if (pre_rss > post_rss)
			// compute inbound gain and sample local (interference+noise)
			in_gain = calcInboundGain(pre_rss, post_rss, tx_power_level);
	}
	
	// see if the packet contains outbound gain for me
	for (i = 0; i < footer_entry_cnts; i++) {
		//contains my outbound gain
		if (footer[i].nb == my_ll_addr) {
			out_gain = footer[i].inbound_gain;
		}
	}
	//call UartLog.logTxRx(DBG_FLAG, DBG_ER_FLAG, 3, 0, 0, 0, nb, in_gain, out_gain);
	// update signal map
	updateSignalMap(nb, in_gain, out_gain, hdr->node_i);
}

void updateSignalMap(am_addr_t nb, int16_t in_gain, int16_t out_gain, int16_t node_i_) {
	int16_t idx;
	sm_entry_t *se;

	// update signal map: FIFO
	// if found
	//		update
	// else
	// 		if exists empty entry
	//			initialize
	//		else
	//			drop
	idx = findIdx(nb);
	if (idx < SM_SIZE) {
		se = &signalMap[idx];
		// update (interference+noise)
		se->node_i = node_i_;
		// a new inbound gain sample; sample on if it is valid
		if (in_gain != INVALID_GAIN) {
			if (se->inbound_gain != INVALID_GAIN) {
				se->inbound_gain = se->inbound_gain - (se->inbound_gain >> EWMA_R_SHIFT_BIT) + (in_gain >> EWMA_R_SHIFT_BIT);
			} else {
				se->inbound_gain = in_gain;
			}
		}
		// update outbound gain only if it is valid
		if (out_gain != INVALID_GAIN)
			se->outbound_gain = out_gain;

		sortSignalMap(idx);
	} else {
		/* 
		 * only admit neighbors whose inbound gain is known bcoz 
		 * 1) otherwise corrupted neighbors, neighbor w/ id not even in testbed, can crop in, whose inbound gain remains unknown and cause codes requiring inbound gain (e.g., ER update) to halt
		 * 2) anyway, admit such neighbors is useless since inbound gain unknown; the only issue is we may miss some initial outbound gain updates and hopefully can catch up w/ more coming later!!
		 */
		if (in_gain != INVALID_GAIN) {
			idx = findEmptyIdx();
			if (idx < SM_SIZE) {
				se = &signalMap[idx];
				// initialize
				se->nb = nb;
				se->valid = TRUE;
				se->node_i = node_i_;
				// no need for validity check for 1st time
				se->inbound_gain = in_gain;
				se->outbound_gain = out_gain;
				// will be initialized later before sending any data
				se->tx_interference_threshold = 0;
				se->rx_interference_threshold = 0;
				// empty ER
				se->tx_er_border_idx = -1;
				se->rx_er_border_idx = -1;
				se->is_in_outbound_er = FALSE;
				se->age = 0;
				// min var controller
				se->tx_I.sign = 0;
				se->tx_I.abs = INVALID_DBM;
				se->rx_I.sign = 0;
				se->rx_I.abs = INVALID_DBM;
				se->tx_prev_I.sign = 0;
				se->tx_prev_I.abs = INVALID_DBM;
				se->rx_prev_I.sign = 0;
				se->rx_prev_I.abs = INVALID_DBM;
				se->tx_delta_I_d.sign = 0;
				se->tx_delta_I_d.abs = INVALID_DBM;
				se->rx_delta_I_d.sign = 0;
				se->rx_delta_I_d.abs = INVALID_DBM;
				se->tx_mean_delta_I_u.sign = 0;
				se->tx_mean_delta_I_u.abs = INVALID_DBM;
				se->rx_mean_delta_I_u.sign = 0;
				se->rx_mean_delta_I_u.abs = INVALID_DBM;

				sortSignalMap(idx);
			}
		}
	}
}

// re-sort signal map by ascending inbound gain after its update
// @param idx: index of the neighbor whose inbound gain changes
void sortSignalMap(int16_t idx) {
	int16_t i, sm_size;
	int16_t new_in_gain;
	sm_entry_t tmp;
	sm_entry_t *se;
	
	tmp = signalMap[idx];
	new_in_gain = tmp.inbound_gain;
	
	if (idx > 0) {
		if (new_in_gain < signalMap[idx - 1].inbound_gain) {
			// moving forward
			for (i = idx; i > 0; i--) {
				se = &signalMap[i - 1];
				if (se->inbound_gain > new_in_gain) {
					signalMap[i] = signalMap[i - 1];
				} else {
					break;
				}
			}
			// found right location
			signalMap[i] = tmp;
		}
	}
	
	sm_size = getSignalMapSize();
	if ((idx + 1) < sm_size) {
		if (new_in_gain > signalMap[idx + 1].inbound_gain) {
			// moving backward
			for (i = idx; (i + 1) < sm_size; i++) {
				se = &signalMap[i + 1];
				if (se->inbound_gain < new_in_gain) {
					signalMap[i] = signalMap[i + 1];
				} else {
					break;
				}
			}
			// found right location
			signalMap[i] = tmp;
		}
	}
	
	//dbg("SM", "%s after sorting %u\n", __FUNCTION__, idx);
	//printSignalMap();
}

// ===========================================================================================================
// update NI for a node and every link when a new NI sample arrives
// either a new beacon is received (processReceivedMessage) or in data plane IMACForwarderP$Receive$receive
// for sender, sample NI when CTS received
// for receiver, sample NI when CTS sent or DATA received
// @param ni: new NI sample
// change I(t) estimation from arithmetic mean to moving average bcoz of ease of computation using dbmWeightedSumS
// ===========================================================================================================
command void SignalMap.updateNI(bool is_link_ni, am_addr_t nb, bool is_sender, int16_t ni) {
	uint8_t idx;
	sm_entry_t *se;
	dbm_t nI;
	
	// TODO: filter outliers, mostly, if not all, are 83 (255 - 172)
	if (ni >= 0)
		return;
	//call UartLog.logEntry(DBG_FLAG, DBG_CALC_FLAG, nb, -ni);
	nI.sign = 1;
	nI.abs = (ni << SCALE_L_SHIFT_BIT);
	// node NI
	//if (node_I.abs != INVALID_DBM) {
	if (node_I.sign != 0) {
		//node_i = node_i - (node_i >> 4) + (ni >> 4);
		node_I = dbmWeightedSumS(node_I, nI);
	} else {
		node_I = nI;
	}
	
	if (!is_link_ni)
		return;

	idx = findIdx(nb);
	if (idx < SM_SIZE) {
		se = &signalMap[idx];
		call UartLog.logTxRx(DBG_FLAG, DBG_ER_FLAG, 9, 0, idx, -se->rx_I.abs / 128, nb, is_sender, -ni);
		if (is_sender) {
			if (se->tx_I.sign != 0)
				se->tx_I = dbmWeightedSumS(se->tx_I, nI);
			else
				se->tx_I = nI;
		} else {
			if (se->rx_I.sign != 0)
				se->rx_I = dbmWeightedSumS(se->rx_I, nI);
			else
				se->rx_I = nI;
		}
	}
	
//	// link NI for each neighbor
//	for (i = 0; i < SM_SIZE; i++) {
//		se = &signalMap[i];
//		if (se->valid) {
//			// update mean noise+interference in this interval
//			//i = (i * i_cnt + ni) / (i_cnt + 1);
//			//i_cnt++;
//			// only for active links to save processing
//			if (se->tx_interference_threshold != 0) {
//				//se->tx_i = ((int32_t)se->tx_i * se->tx_i_cnt + ni) / (se->tx_i_cnt + 1);
//				if (se->tx_I.sign != 0)
//					se->tx_I = dbmWeightedSumS(se->tx_I, nI);
//				else
//					se->tx_I = nI;
//				//se->tx_i_cnt++;
//				//if (se->tx_i > 0 || se->tx_i < -100)
//					//call UartLog.logTxRx(DBG_FLAG, DBG_ER_FLAG, 9, 0, idx, se->nb, se->tx_i, se->tx_i_cnt, ni);
//			}
//			if (se->rx_interference_threshold != 0) {
//				//se->rx_i = ((int32_t)se->rx_i * se->rx_i_cnt + ni) / (se->rx_i_cnt + 1);
//				if (se->rx_I.sign != 0)
//					se->rx_I = dbmWeightedSumS(se->rx_I, nI);
//				else
//					se->rx_I = nI;
//				//se->rx_i_cnt++;
//				//if (se->rx_i > 0 || se->rx_i < -100)
//					//call UartLog.logTxRx(DBG_FLAG, DBG_ER_FLAG, 9, 1, idx, se->nb, se->rx_i, se->rx_i_cnt, ni);
//			}
//		}
//	}
}


command void SignalMap.printSignalMap() {
	int16_t i;
	sm_entry_t *se;
	
	dbg("SM", "%s\n", __FUNCTION__);
	for (i = 0; i < SM_SIZE; i++) {
		se = &signalMap[i];
		if (se->valid) {
			dbg_clear("SM", "%u: <%u, %d>\n", i, se->nb, se->inbound_gain);
			//if (INVALID_GAIN == se->inbound_gain || INVALID_GAIN == se->outbound_gain)
				//call UartLog.logTxRx(DBG_FLAG, DBG_SM_FLAG, 0, i, se->rx_er_border_idx, se->inbound_gain, se->outbound_gain, se->rx_interference_threshold, se->nb);
		}
	}
}

command void SignalMap.initLinkExRegion(am_addr_t nb, bool is_tx) {
	execController(nb, is_tx, TRUE);
}

//#ifndef TOSSIM
//// sample noise + interfenrece periodically
//event void ReadRssi.readDone(error_t result, uint16_t val) {
//	int16_t ni;
//	
//	// sample noise plus interference when it is valid
//	if (SUCCESS == result) {
//		ni = (int16_t)val - 172;
//		call SignalMap.updateNI(FALSE, 0, FALSE, ni);
//	}
//}
//#endif

//event void NISampleTimer.fired() {

//#ifndef TOSSIM
//	call ReadRssi.read();
//#endif
//}

/* *
 * * Interface Packet
 * */
command void Packet.clear(message_t* msg) {
	call SubPacket.clear(msg);
}

// subtract the space occupied by the signal map header and footer from the incoming payload size
command uint8_t Packet.payloadLength(message_t* msg) {
	sm_header_t *hdr = getHeader(msg);
	return (call SubPacket.payloadLength(msg) - sizeof(sm_header_t) - sizeof(sm_footer_t) * hdr->footer_entry_cnts);
}

// account for the space used by header and footer while setting the payload length
command void Packet.setPayloadLength(message_t* msg, uint8_t len) {
	sm_header_t *hdr = getHeader(msg);
	call SubPacket.setPayloadLength(msg, len + sizeof(sm_header_t) + sizeof(sm_footer_t) * hdr->footer_entry_cnts);
}

command uint8_t Packet.maxPayloadLength() {
	return (call SubPacket.maxPayloadLength() - sizeof(sm_header_t));
}

// application payload pointer is just past the link estimation header
command void* Packet.getPayload(message_t* msg, uint8_t len) {
	void* payload = call SubPacket.getPayload(msg, len + sizeof(sm_header_t));
	if (payload != NULL) {
		payload += sizeof(sm_header_t);
	}
	return payload;
}

// divide into 2 seperate files for readability
#include "SignalMapPUtils.nc"

}
