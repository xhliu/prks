/* *
 * @ author:    Xiaohui Liu (whulxh@gmail.com) 
 * @ updated:   08/31/2012 02:55:03 PM 
 * @ description: async receive
 */

#include <TinyError.h>
#include <message.h>

interface AsyncReceive {
	async event message_t* receive(message_t* msg, void* payload, uint8_t len);
}
