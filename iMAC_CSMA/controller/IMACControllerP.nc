/*
 * @ author: Xiaohui Liu (whulxh@gmail.com)
 * @ updated: 04/05/2012 08:56:00 PM 
 * @ description:
 				input: current link pdr
 				output: deltaI
 */
#include "IMACController.h"
//#include "../signalmap/SignalMap.h"
#include "../IMac.h"

module IMACControllerP {
	provides {
		interface IMACController;
	}
}
implementation {

// mapping PDR to (1 / a)
uint16_t pdr_slope_table[] = {841, 19, 11, 9, 7, 6, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 9, 10, 11, 12, 14, 16, 20, 24, 32, 620, 1233, 3430};

// compute control output based on input
// scaled by 2^SCALE_L_SHIFT_BIT
#ifdef P_CONTROLLER
command int32_t IMACController.controller(uint8_t link_pdr, uint8_t link_pdr_sample, uint8_t reference_pdr) {
	uint8_t table_size;
	uint16_t slope;
	int32_t deltaI_dB;
	
	// just to be cautious and prevent arrayoutofbound
	table_size = sizeof(pdr_slope_table) / sizeof(pdr_slope_table[0]);
	if (link_pdr >= table_size)
		link_pdr = table_size - 1;
	// 1 / a
	slope = pdr_slope_table[link_pdr];
	// 1 / a_r: special care when PDR is far away from reference
	if (((link_pdr > reference_pdr) && (link_pdr - reference_pdr) > E0) || 
		((reference_pdr > link_pdr) && (reference_pdr - link_pdr) > E0)) {
		// get min bcoz this is reciprocal
		if (slope > pdr_slope_table[reference_pdr])
			slope = pdr_slope_table[reference_pdr];
	}
	// TODO: scaling 128 times enough?
	deltaI_dB = slope * (((int32_t)link_pdr - (int32_t)reference_pdr) << SCALE_L_SHIFT_BIT) / 100;
	return deltaI_dB;
}

#elif defined(MIN_VAR_CONTROLLER)

command int32_t IMACController.controller(uint8_t link_pdr, uint8_t link_pdr_sample, uint8_t reference_pdr) {
	uint8_t table_size;
	uint16_t slope;
	int32_t deltaI_dB, nominator;
	
	// just to be cautious and prevent arrayoutofbound
	table_size = sizeof(pdr_slope_table) / sizeof(pdr_slope_table[0]);
	if (link_pdr >= table_size)
		link_pdr = table_size - 1;
	// 1 / a
	slope = pdr_slope_table[link_pdr];
	// 1 / a_r: special care when PDR is far away from reference
	if (((link_pdr > reference_pdr) && (link_pdr - reference_pdr) > E0) || 
		((reference_pdr > link_pdr) && (reference_pdr - link_pdr) > E0)) {
		// get min bcoz this is reciprocal
		if (slope > pdr_slope_table[reference_pdr])
			slope = pdr_slope_table[reference_pdr];
	}
	nominator = (ALPHA * (int32_t)link_pdr + (10 - ALPHA) * (int32_t)link_pdr_sample) / 10 - (int32_t)reference_pdr - DELTA_Y;
	// change scale
	nominator = (nominator << SCALE_L_SHIFT_BIT) / 100;
	// 1 / (1 - c)
	deltaI_dB = slope * nominator * 10 / (10 - ALPHA);
	return deltaI_dB;
}


#endif
}

