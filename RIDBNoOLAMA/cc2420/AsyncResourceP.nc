#include "AsyncResource.h"

module AsyncResourceP {
	provides {
		// client is binary only
		interface AsyncResource as Resource[uint8_t client];
	}
}
implementation {

bool spi_locked = FALSE;

bool tx_pending = FALSE;
bool rx_pending = FALSE;

async command error_t Resource.request[uint8_t client]() {
	atomic {
		if (spi_locked) {
			if (TX_CLIENT == client) {
				tx_pending = TRUE;
			} else {
				rx_pending = TRUE;
			}
			return FAIL;
		}
		spi_locked = TRUE;
		return SUCCESS;
	}
}

async command error_t Resource.immediateRequest[uint8_t client]() {
	return call Resource.request[client]();
}

async command error_t Resource.release[uint8_t client]() {
	bool pending;

	// let the other know
	if (TX_CLIENT == client) {
		atomic pending = rx_pending;
		if (pending) {
			signal Resource.granted[RX_CLIENT]();
			atomic rx_pending = FALSE;
		}
	} else if (RX_CLIENT == client) {
		atomic pending = tx_pending;
		if (pending) {
			signal Resource.granted[TX_CLIENT]();
			atomic tx_pending = FALSE;
		}
	}
	atomic spi_locked = FALSE;
	return SUCCESS;
}

async command bool Resource.isOwner[uint8_t client]() {
	return FALSE;
}

default async event void Resource.granted[uint8_t client]() {}
}
