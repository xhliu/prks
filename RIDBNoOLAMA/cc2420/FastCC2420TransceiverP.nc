/* *
 * @ author:    Xiaohui Liu (whulxh@gmail.com) 
 * @ updated:   08/31/2012 02:55:03 PM 
 * @ description: a sender that skips all layers from CC2420CsmaC to AMSenderC to speed up tx/rx
 * 0) packet layout:
 	 CC2420 header | fast payload | fast CRC footer | cc2420 footer
 */
#include "FastCC2420Transceiver.h"

module FastCC2420TransceiverP {
	provides {
		interface Init;
		interface FastSend as Send[uint8_t client];
		interface FastReceive as Receive[uint8_t client];
		interface FastAMPacket as AMPacket;
		interface FastPacket as Packet;
//		interface PacketTimeSyncOffset;
	}
	
	uses {
		interface CC2420Transmit as SubSend;
		interface FastReceive as SubReceive;
		
		interface ActiveMessageAddress;
		interface CC2420PacketBody;
		interface CC2420Config;
		
		interface Random;
		interface Crc;
		
		interface UartLog;
	}
}

implementation {

uint8_t localSendId;
am_addr_t my_ll_addr;

command error_t Init.init() {
	localSendId = call Random.rand16();
	my_ll_addr = call AMPacket.address();
	return SUCCESS;
}

async event void CC2420Config.syncDone(error_t error) {}


// --------------------------------------------------------------
// Send interface
// --------------------------------------------------------------
async command error_t Send.send[uint8_t client](am_addr_t dest, message_t* msg, uint8_t len) {
    // regard beginning of cc2420 header as base
    uint8_t offset;
    // crc excluding timesync_radio_t
    uint16_t crc;
    fast_footer_t *fast_footer;

	cc2420_header_t* header = call CC2420PacketBody.getHeader(msg);
    cc2420_metadata_t* metadata = call CC2420PacketBody.getMetadata(msg);
    
    // fill in header, excluding length field per se
    header->length = CC2420_SIZE + len + sizeof(fast_footer_t);
#ifdef CC2420_HW_SECURITY
    header->fcf &= ((1 << IEEE154_FCF_ACK_REQ)|(1 << IEEE154_FCF_SECURITY_ENABLED));
#else
    header->fcf &= 1 << IEEE154_FCF_ACK_REQ;
#endif
    header->fcf |= ( ( IEEE154_TYPE_DATA << IEEE154_FCF_FRAME_TYPE ) |
		     ( 1 << IEEE154_FCF_INTRAPAN ) |
		     ( IEEE154_ADDR_SHORT << IEEE154_FCF_DEST_ADDR_MODE ) |
		     ( IEEE154_ADDR_SHORT << IEEE154_FCF_SRC_ADDR_MODE ) );
	
	atomic header->dsn = localSendId++;
	header->destpan = call CC2420Config.getPanAddr();

    header->dest = dest;
    header->src = my_ll_addr;
	header->network = TINYOS_6LOWPAN_NETWORK_ID;
	header->type = client;
	
	// append CRC footer including everything, even the length filed, except itself and the FCS
	offset = sizeof(cc2420_header_t) + len;
	crc = call Crc.crc16((void *)header, offset - sizeof(timesync_radio_t));
	fast_footer = (fast_footer_t *)((uint8_t *)header + offset);
	// convert length of bytes into uint16_t
	*fast_footer = call Crc.seededCrc16(crc, (void *)((uint8_t *)header + offset - sizeof(timesync_radio_t)), sizeof(timesync_radio_t));
	
	// fill in metadata
    metadata->ack = FALSE;
    metadata->rssi = 0;
    metadata->lqi = 0;
    //metadata->timesync = FALSE;
    metadata->timestamp = CC2420_INVALID_TIMESTAMP;
	// to compute final CRC including timesync footer for ftsp beacons
	metadata->partial_crc = crc;
		
	// diable cca for all packets, including iMAC and ftsp
	return (call SubSend.send(msg, FALSE));
}

async event void SubSend.sendDone(message_t* msg, error_t error) {
	cc2420_header_t* header = call CC2420PacketBody.getHeader(msg);
	uint8_t type = header->type;
	
	signal Send.sendDone[type](msg, error);
}

async command error_t Send.cancel[uint8_t client](message_t* msg) {
	return call SubSend.cancel();
}

async command uint8_t Send.maxPayloadLength[uint8_t client]() {
	return call Packet.maxPayloadLength();
}

async command void* Send.getPayload[uint8_t client](message_t* m, uint8_t len) {
	return call Packet.getPayload(m, len);
}


// --------------------------------------------------------------
// Receive interface
// -------------------------------------------------------------- 
async event message_t *SubReceive.receive(message_t *msg, void *payload, uint8_t len) {
	uint8_t offset;
	uint16_t crc;
	fast_footer_t *fast_footer;

	cc2420_header_t* header = call CC2420PacketBody.getHeader(msg);
	uint8_t type = header->type;
	
	// CRC check
	offset = sizeof(cc2420_header_t) + len - sizeof(fast_footer_t);
	fast_footer = (fast_footer_t *)((uint8_t *)header + offset);
	crc = call Crc.crc16((void *)header, offset);
	
	if ((*fast_footer) == crc) {
		// pass
//		call UartLog.logTxRx(DBG_FLAG, DBG_DELAY_FLAG, 0, offset, crc, header->src, header->dest, header->type, __LINE__);
		return signal Receive.receive[type](msg, call Packet.getPayload(msg, call Packet.payloadLength(msg)), call Packet.payloadLength(msg));
	} else {
		// fail
//		call UartLog.logTxRx(DBG_FLAG, DBG_DELAY_FLAG, 1, offset, crc, header->src, header->dest, header->type, __LINE__);
		return msg;
	}
}


// --------------------------------------------------------------
// AMPacket interface
// -------------------------------------------------------------- 
async command am_addr_t AMPacket.address() {
	return call ActiveMessageAddress.amAddress();
}

async command am_addr_t AMPacket.destination(message_t* amsg) {
	cc2420_header_t* header = call CC2420PacketBody.getHeader(amsg);
	return header->dest;
}

async command am_addr_t AMPacket.source(message_t* amsg) {
	cc2420_header_t* header = call CC2420PacketBody.getHeader(amsg);
	return header->src;
}

async command void AMPacket.setDestination(message_t* amsg, am_addr_t addr) {
	cc2420_header_t* header = call CC2420PacketBody.getHeader(amsg);
	header->dest = addr;
}

async command void AMPacket.setSource(message_t* amsg, am_addr_t addr) {
	cc2420_header_t* header = call CC2420PacketBody.getHeader(amsg);
	header->src = addr;
}

async command bool AMPacket.isForMe(message_t* amsg) {
	return (call AMPacket.destination(amsg) == call AMPacket.address() ||
    	call AMPacket.destination(amsg) == AM_BROADCAST_ADDR);
}

async command am_id_t AMPacket.type(message_t* amsg) {
	cc2420_header_t* header = call CC2420PacketBody.getHeader(amsg);
	return header->type;
}

async command void AMPacket.setType(message_t* amsg, am_id_t type) {
	cc2420_header_t* header = call CC2420PacketBody.getHeader(amsg);
	header->type = type;
}

async command am_group_t AMPacket.group(message_t* amsg) {
	return (call CC2420PacketBody.getHeader(amsg))->destpan;
}

async command void AMPacket.setGroup(message_t* amsg, am_group_t grp) {
	// Overridden intentionally when we send()
	(call CC2420PacketBody.getHeader(amsg))->destpan = grp;
}

async command am_group_t AMPacket.localGroup() {
	return call CC2420Config.getPanAddr();
}

// --------------------------------------------------------------
// Packet interface
// --------------------------------------------------------------
async command void Packet.clear(message_t* msg) {
	memset(call CC2420PacketBody.getHeader(msg), 0x0, sizeof(cc2420_header_t));
	memset(call CC2420PacketBody.getMetadata(msg), 0x0, sizeof(cc2420_metadata_t));
}

async command uint8_t Packet.payloadLength(message_t* msg) {
	cc2420_header_t *header = call CC2420PacketBody.getHeader(msg);
	uint8_t len = header->length - CC2420_SIZE - sizeof(fast_footer_t);
	return len;
}

async command void Packet.setPayloadLength(message_t* msg, uint8_t len) {
	cc2420_header_t *header = call CC2420PacketBody.getHeader(msg);
	header->length = len + CC2420_SIZE + sizeof(fast_footer_t);
}

async command uint8_t Packet.maxPayloadLength() {
	// exclude the padding byte as well
	return (TOSH_DATA_LENGTH - sizeof(fast_footer_t));
}

async command void* Packet.getPayload(message_t* msg, uint8_t len) {
	if (len <= call Packet.maxPayloadLength()) {
    	return (void *)msg->data;
    } else {
	    return NULL;
    }
}

//// --------------------------------------------------------------
//// PacketTimeSyncOffset interface
//// --------------------------------------------------------------
//async command bool PacketTimeSyncOffset.isSet(message_t* msg) {
//	return ((call CC2420PacketBody.getMetadata(msg))->timesync);
//}

////returns offset of timestamp from the beginning of cc2420 header which is
////          sizeof(cc2420_header_t)+datalen-sizeof(timesync_radio_t)
////uses packet length of the message which is
////          MAC_HEADER_SIZE+MAC_FOOTER_SIZE+datalen
//// XL: timestamp is not the end of a packet anymore, CRC added
//async command uint8_t PacketTimeSyncOffset.get(message_t* msg) {
//	// XL: essentially return (call CC2420PacketBody.getHeader(msg))->length + 1 - MAC_FOOTER_SIZE - sizeof(timesync_radio_t)
//	return (call CC2420PacketBody.getHeader(msg))->length
//		    + (sizeof(cc2420_header_t) - MAC_HEADER_SIZE)
//		    - MAC_FOOTER_SIZE
//		    - sizeof(timesync_radio_t)
//		    - sizeof(fast_footer_t);
//}

//async command void PacketTimeSyncOffset.set(message_t* msg) {
//	(call CC2420PacketBody.getMetadata(msg))->timesync = TRUE;
//}

//async command void PacketTimeSyncOffset.cancel(message_t* msg) {
//	(call CC2420PacketBody.getMetadata(msg))->timesync = FALSE;
//}

  
async event void ActiveMessageAddress.changed() {}
default async event void Send.sendDone[uint8_t client](message_t* msg, error_t error) {}
default async event message_t *Receive.receive[uint8_t client](message_t *msg, void *payload, uint8_t len) {
	return msg;
}


//async command error_t Send.send[uint8_t client](am_addr_t dest, message_t* msg, uint8_t len) {
//    // AMQueueEntryP
//    call AMPacket.setDestination(msg, dest);
//    call AMPacket.setType(msg, client);
//	
//	// AMQueueImplP
//	call Packet.setPayloadLength(msg, len);
//	
//	// CC2420TinyosNetworkP
//	call CC2420Packet.setNetwork(msg, TINYOS_6LOWPAN_NETWORK_ID);
//	
//	// UniqueSendP
//	localSendId = call Random.rand16();
//	call CC2420PacketBody.getHeader(msg))->dsn = localSendId++;
//	
//	// CC2420CsmaP
//	cc2420_header_t* header = call CC2420PacketBody.getHeader( p_msg );
//    cc2420_metadata_t* metadata = call CC2420PacketBody.getMetadata( p_msg );
//    header->length = len + CC2420_SIZE;
//    header->fcf |= ( ( IEEE154_TYPE_DATA << IEEE154_FCF_FRAME_TYPE ) |
//		     ( 1 << IEEE154_FCF_INTRAPAN ) |
//		     ( IEEE154_ADDR_SHORT << IEEE154_FCF_DEST_ADDR_MODE ) |
//		     ( IEEE154_ADDR_SHORT << IEEE154_FCF_SRC_ADDR_MODE ) );

//    metadata->ack = FALSE;
//    metadata->rssi = 0;
//    metadata->lqi = 0;
//    //metadata->timesync = FALSE;
//    metadata->timestamp = CC2420_INVALID_TIMESTAMP;
//}

}
