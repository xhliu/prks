#ifndef IMAC_H
#define IMAC_H

enum {
	// EWMA, w/ a denominator of 10
	ALPHA = 9,
	SCALE_L_SHIFT_BIT = 7,
	
	// PDR requirement; differentiate data/ack reliability for now
	// C-MAC: req not set here but in IMACForwarder.h
	REFERENCE_DATA_PDR = 95,
//	REFERENCE_ACK_PDR = 80,
};
#endif
