/*
 * @ author: Xiaohui Liu (whulxh@gmail.com)
 * 
 * @ created: 12/28/2011
 */
 
#ifndef SIGNAL_MAP_H
#define SIGNAL_MAP_H

enum {
	INVALID_GAIN = 0x7FFF,
	OFFSET = 1,
	EWMA_R_SHIFT_BIT = 4,	// right shift to scale down
	// scale gain for precision
	//SCALE_L_SHIFT_BIT = 7,	//left shift to scale up
	
	// Eq. (11) in paper "convergence theorem for a general class of power-control algorithms"
	// \gamma: min SNR to ensure reliability 98% based on "$Dropbox/DNC/Students/Xiaohui/pdr_vs_sinr_curve.fig"
	GAMMA = 10,
	DELTA = 2,
	INVALID_POWER_LEVEL = 255,
	
	// initial (interference+noise)
	INVALID_DBM = 0x7FFF,
	// min signed 16 bits
	MIN_DBM = 0x8000,
	
	// hysteresis on tx power after maximal ER shrinks
	POWER_LEVEL_REUSE_CNT = 2,
	MAX_AGE = 4,
	AGING_PERIOD = 30000,
	
	// sample noise + interference
	NI_SAMPLE_PERIOD = 2000,
	MIN_NODE_I = -100,
	MAX_NODE_I = 0,
};

// default signal map size
// size is int16_t to accommodate network consisting of more than 256 nodes
#ifndef SM_SIZE
#define SM_SIZE 110		// accommodate some illegitimate nodes of corrupted node id
#endif

//signal map header
typedef nx_struct {
	nx_uint8_t power_level;
	// node's (interference+noise)
	nx_int16_t node_i;
	// number of entries in the footer
	nx_uint8_t footer_entry_cnts;
	// TODO
	nx_uint16_t seqno;
} sm_header_t;

//signal map footer containing neighbor and inbound gain, used to compute outbound gain
typedef nx_struct {
	nx_am_addr_t nb;
	//power gain to a neighbor
	nx_int16_t inbound_gain;
} sm_footer_t;

// represent \delta_Is
// 0 is represented as {0, 0}
typedef struct {
	// can only be 1, 0 or -1
	int8_t sign;
	// absolute value in dBm; thus can be negative
	int16_t abs;
} dbm_t;

//signal map entry
typedef struct {
	am_addr_t nb;
	bool valid;
	// interference_noise at the neighbor; useful when deciding tx power level for RTS/CTS
	int16_t node_i;
	
	// scaled
	// to compute ER
	int16_t inbound_gain;
	// to compute RTS/CTS tx power & determine if I'm in the ER of the neighbor
	int16_t outbound_gain;
	
	// for power control
	int16_t prev_tx_power_level;
	// ER
	int16_t tx_interference_threshold;
	int16_t rx_interference_threshold;	
/* boundary of the current exclusion region for the link to this neighbor (inbound), 
 * i.e., outmost node index within the ER in SM
 * ER is the set of neighbors from [0 .. ex_region_border_idx] in SM
 * -1 means empty ER
 */
	// when I transmit DATA to the neighbor
	int16_t tx_er_border_idx;
	// when I receive DATA from the neighbor
	int16_t rx_er_border_idx;
	
	// A is in the outbound ER of B if the maximal ER of B includes A
	// is this neighbor in my outbound ER?
	bool is_in_outbound_er;
	// TTL to kick the neighbor out of my outbound ER in case I haven't heard its hysteresis signal
	uint8_t age;
	
	// min variance controller related info
	// capital I means in capsulation
	// mean noise+interference for a link during interval (t, t+1] so far
	dbm_t tx_I;
	dbm_t rx_I;
	// # of samples in interval (t, t+1] so far
	//int16_t tx_i_cnt;
	//int16_t rx_i_cnt;
	// I(t - 1): mean interference+noise for a link during interval (t - 1, t]
	dbm_t tx_prev_I;
	dbm_t rx_prev_I;
	//int16_t tx_prev_i;
	//int16_t rx_prev_i;
	// \Delta I_d(t): previous desired NI change; scaled
	dbm_t tx_delta_I_d;
	dbm_t rx_delta_I_d;
	// \bar(\Delta I_u(t)) mean delta unknown NI change; scaled
	dbm_t tx_mean_delta_I_u;
	dbm_t rx_mean_delta_I_u;
} sm_entry_t;

#endif
