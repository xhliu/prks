/* *
 * @ author:    Xiaohui Liu (whulxh@gmail.com) 
 * @ updated:   08/31/2012 02:55:03 PM 
 * @ description: async receive
 */

#include <TinyError.h>
#include <message.h>

interface AsyncIntercept {
	// mainly to log timestamp
	// @param is_incoming: packet arrival or departure
#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
async 
#endif
	event bool forward(bool is_incoming, message_t* msg, void* payload, uint8_t len);
}
