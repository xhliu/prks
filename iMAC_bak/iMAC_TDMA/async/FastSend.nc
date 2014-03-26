/* *
 * @ author:    Xiaohui Liu (whulxh@gmail.com) 
 * @ updated:   08/31/2012 02:55:03 PM 
 * @ description: async sender
 */

#include <TinyError.h>
#include <message.h>

interface FastSend {
	async command error_t send(am_addr_t dest, message_t* msg, uint8_t len);

	async event void sendDone(message_t* msg, error_t error);
	
	async command error_t cancel(message_t* msg);
	async command uint8_t maxPayloadLength();
	async command void* getPayload(message_t* msg, uint8_t len);
}
