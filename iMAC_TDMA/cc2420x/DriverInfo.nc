#include <CC2420XDriverLayer.h>

interface DriverInfo {
	async command uint16_t getStatus();
	async command uint16_t getSfdStatus();
	
	async command void enableSfd();
	async command void disableSfd();
}
