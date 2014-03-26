module CC2420ShareSpiP {
	provides {
		interface Resource;
		interface ChipSpiResource;
	}
	uses {
		interface Resource as SpiResource;
		
		interface UartLog;
	}
}
implementation {

bool is_owner = FALSE;

// -----------------------------------------------------------------
//	Resource interface
// -----------------------------------------------------------------
async command error_t Resource.request() {
//	error_t error = call SpiResource.request();
//	if (error != SUCCESS)
//	return error;
	return SUCCESS;
}

async command error_t Resource.immediateRequest() {
	return SUCCESS;
//	error_t error;
//atomic {
//	if (is_owner)
//		return SUCCESS;
//}	
//	error = call SpiResource.immediateRequest();
//	if (SUCCESS == error) {
//		atomic is_owner = TRUE;
//		return SUCCESS;
//	} else {
//		error = call SpiResource.request();
//	}
//	if (error != SUCCESS)
//		call UartLog.logEntry(DBG_FLAG, DBG_SPI_FLAG, error, 1);
//	return error;
}

async command bool Resource.isOwner() {
	return TRUE;
//	return call SpiResource.isOwner();
}

async command error_t Resource.release() {
	// never release after obtained
	return SUCCESS;
	//return call SpiResource.release();
}

event void SpiResource.granted() {
//	atomic is_owner = TRUE;
//	signal Resource.granted();
}

// -----------------------------------------------------------------
//	Resource interface
// -----------------------------------------------------------------
async command void ChipSpiResource.abortRelease() {
}
async command error_t ChipSpiResource.attemptRelease() {
	return SUCCESS;
}
default async event void ChipSpiResource.releasing() {
}

}
