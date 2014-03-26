/*
 * Copyright (c) 2005-2006 Arch Rock Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the Arch Rock Corporation nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * ARCHED ROCK OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE
 */

/**
 * @author Jonathan Hui <jhui@archrock.com>
 * @author David Moss
 * @author Jung Il Choi
 * @author JeongGil Ko
 * @author Razvan Musaloiu-E
 * @version $Revision: 1.22 $ $Date: 2009/10/28 21:09:52 $
 */

#include "IEEE802154.h"
#include "message.h"
#include "AM.h"

module CC2420ReceiveP @safe() {

  provides interface Init;
  provides interface StdControl;
  provides interface CC2420Receive;
  provides interface Receive;
  // XL
  provides interface FastReceive;
  provides interface ReceiveIndicator as PacketIndicator;
  uses interface GeneralIO as CSN;
  uses interface GeneralIO as FIFO;
  uses interface GeneralIO as FIFOP;
  uses interface GpioInterrupt as InterruptFIFOP;
#ifdef TIMESYNC_FIX
	uses interface GpioCapture as CaptureSFD;
#endif
  uses interface Resource as SpiResource;
  uses interface CC2420Fifo as RXFIFO;
  uses interface CC2420Strobe as SACK;
  uses interface CC2420Strobe as SFLUSHRX;
  uses interface CC2420Packet;
  uses interface CC2420PacketBody;
  uses interface CC2420Config;
  uses interface PacketTimeStamp<T32khz,uint32_t>;

  uses interface CC2420Strobe as SRXDEC;
  uses interface CC2420Register as SECCTRL0;
  uses interface CC2420Register as SECCTRL1;
  uses interface CC2420Ram as KEY0;
  uses interface CC2420Ram as KEY1;
  uses interface CC2420Ram as RXNONCE;
  uses interface CC2420Ram as RXFIFO_RAM;
  uses interface CC2420Strobe as SNOP;

  uses interface Leds;
#ifdef SIGNAL_MAP
	uses interface RssiRead;
#endif
#if defined(SLOT_INTEGRITY)
	uses interface SlotInfo;
#endif	
	uses interface TransmitInfo;

	uses interface LocalTime<T32khz>;
	uses interface UartLog;
}

implementation {

  typedef enum {
    S_STOPPED,
    S_STARTED,
    S_RX_LENGTH,
    S_RX_DEC,
    S_RX_DEC_WAIT,
    S_RX_FCF,
    S_RX_PAYLOAD,
  } cc2420_receive_state_t;

  enum {
    RXFIFO_SIZE = 128,
    TIMESTAMP_QUEUE_SIZE = 8,
    SACK_HEADER_LENGTH = 7,
  };

  uint32_t m_timestamp_queue[ TIMESTAMP_QUEUE_SIZE ];

  uint8_t m_timestamp_head;
  
  uint8_t m_timestamp_size;
  
  /** Number of packets we missed because we were doing something else */
  uint8_t m_missed_packets;

  /** TRUE if we are receiving a valid packet into the stack */
  bool receivingPacket;
  
  /** The length of the frame we're currently receiving */
  norace uint8_t rxFrameLength;
  
  norace uint8_t m_bytes_left;
  
  norace message_t* ONE_NOK m_p_rx_buf;

  message_t m_rx_buf;

	message_t m_process_buf;
	message_t *m_p_process_buf;
	bool processing;

  cc2420_receive_state_t m_state;

#ifdef SIGNAL_MAP
	uint16_t rssi_val;
#endif
  /***************** Prototypes ****************/
  void reset_state();
  void beginReceive();
  void receive();
  void waitForNextPacket();
  void flush();
  bool passesAddressCheck(message_t * ONE msg);

task void receiveDone_task();
void receiveDone();

#if defined(SLOT_INTEGRITY)
// at most 1 rx in this slot
bool rx_enabled;
bool is_data_channel;
async event void SlotInfo.slotStarted(bool is_data_channel_) {
	is_data_channel = is_data_channel_;
	rx_enabled = TRUE;
}
#endif
  /***************** Init Commands ****************/
  command error_t Init.init() {
    m_p_rx_buf = &m_rx_buf;
    
    m_p_process_buf = &m_process_buf;
    processing = FALSE;
    return SUCCESS;
  }

  /***************** StdControl ****************/
  command error_t StdControl.start() {
    atomic {
      reset_state();
      m_state = S_STARTED;
      atomic receivingPacket = FALSE;
      /* Note:
         We use the falling edge because the FIFOP polarity is reversed. 
         This is done in CC2420Power.startOscillator from CC2420ControlP.nc.
       */
      call InterruptFIFOP.enableFallingEdge();
    }
    return SUCCESS;
  }
  
  command error_t StdControl.stop() {
    atomic {
      m_state = S_STOPPED;
      reset_state();
      call CSN.set();
      call InterruptFIFOP.disable();
    }
    return SUCCESS;
  }

  /***************** CC2420Receive Commands ****************/
  /**
   * Start frame delimiter signifies the beginning/end of a packet
   * See the CC2420 datasheet for details.
   */
  async command void CC2420Receive.sfd( uint32_t time ) {
    if ( m_timestamp_size < TIMESTAMP_QUEUE_SIZE ) {
      uint8_t tail =  ( ( m_timestamp_head + m_timestamp_size ) % 
                        TIMESTAMP_QUEUE_SIZE );
      m_timestamp_queue[ tail ] = time;
      m_timestamp_size++;
    }
  }

  async command void CC2420Receive.sfd_dropped() {
    if ( m_timestamp_size ) {
      m_timestamp_size--;
    }
  }

  /***************** PacketIndicator Commands ****************/
  command bool PacketIndicator.isReceiving() {
    bool receiving;
    atomic {
      receiving = receivingPacket;
    }
    return receiving;
  }
  
// XL
uint16_t rx_cnt = 0;
uint32_t start_time;

#ifdef TIMESYNC_FIX
async event void CaptureSFD.captured( uint16_t time ) {}
#endif

/***************** InterruptFIFOP Events ****************/
async event void InterruptFIFOP.fired() {
	#ifdef SIGNAL_MAP
	atomic rssi_val = INVALID_RSSI;
	call RssiRead.read(&rssi_val);
	#endif
	rx_cnt++;
	start_time = call LocalTime.get();
//	call UartLog.logEntry(DBG_FLAG, DBG_SPI_FLAG, 0, rx_cnt);
	if ( m_state == S_STARTED ) {
		m_state = S_RX_LENGTH;
		beginReceive();
	} else {
		m_missed_packets++;
	}
}

  /***************** SpiResource Events ****************/
event void SpiResource.granted() {
	receive();
}

  /***************** RXFIFO Events ****************/
  /**
   * We received some bytes from the SPI bus.  Process them in the context
   * of the state we're in.  Remember the length byte is not part of the length
   */
#ifdef DUMMY_SPI
async event void RXFIFO.readDone( uint8_t* rx_buf, uint8_t rx_len, error_t error ) {}
void RXFIFOreadDone(uint8_t* rx_buf) {
#else
void RXFIFOreadDone(uint8_t* rx_buf) {}
async event void RXFIFO.readDone( uint8_t* rx_buf, uint8_t rx_len, error_t error ) {
#endif

//	cc2420_status_t status;
    cc2420_header_t* header = call CC2420PacketBody.getHeader( m_p_rx_buf );
    uint8_t tmpLen __DEPUTY_UNUSED__ = sizeof(message_t) - (offsetof(message_t, data) - sizeof(cc2420_header_t));
    uint8_t* COUNT(tmpLen) buf = TCAST(uint8_t* COUNT(tmpLen), header);
    rxFrameLength = buf[ 0 ];
	
    switch (m_state) {

    case S_RX_LENGTH:
#warning disable integrity check
	#if defined(SLOT_INTEGRITY)
    	// XL: slot integrity
    	// only for CTRL channel
    	if (!is_data_channel) {
    		if (call TransmitInfo.isSender()) {
    			// CTRL sender does not rx
    			flush();
    			break;
    		} else {
    			// CTRL receiver rx at most 1
    			if (rx_enabled) {
    				rx_enabled = FALSE;
    			} else {
    				flush();
    				break;
    			}
    		}
    	}
	#endif
    	
		m_state = S_RX_FCF;
		if (rxFrameLength + 1 > m_bytes_left) {
			// Length of this packet is bigger than the RXFIFO, flush it out.
			flush();
		} else {
			// XL: overflow, FIFOP's polarity is reversed
			if (!call FIFO.get() && !call FIFOP.get()) {
				m_bytes_left -= rxFrameLength + 1;
			}
			if (rxFrameLength <= MAC_PACKET_SIZE) {
				if (rxFrameLength > 0) {
					if (rxFrameLength > SACK_HEADER_LENGTH) {
						// This packet has an FCF byte plus at least one more byte to read
						call RXFIFO.continueRead(buf + 1, SACK_HEADER_LENGTH);
						RXFIFOreadDone((uint8_t*)(call CC2420PacketBody.getHeader( m_p_rx_buf )));
//						call UartLog.logTxRx(DBG_FLAG, DBG_SPI_FLAG, 4, status, 0, call CSN.get(), 0, 0, rx_cnt);
					} else {
						// XL: a short pkt, likely an ack, but non-ack pkt passing CRC also possible
						// This is really a bad packet, skip FCF and get it out of here.
						m_state = S_RX_PAYLOAD;
						call RXFIFO.continueRead(buf + 1, rxFrameLength);
						RXFIFOreadDone((uint8_t*)(call CC2420PacketBody.getHeader( m_p_rx_buf )));
					}
				} else {
					// Length == 0; start reading the next packet
					atomic receivingPacket = FALSE;
					call CSN.set();
					call SpiResource.release();
					waitForNextPacket();
				}
			} else {
				// Length is too large; we have to flush the entire Rx FIFO
				flush();
			}
		}
      	break;
      
	case S_RX_FCF:
		m_state = S_RX_PAYLOAD;
		/*
		* The destination address check here is not completely optimized. If you 
		* are seeing issues with dropped acknowledgements, try removing
		* the address check and decreasing SACK_HEADER_LENGTH to 2.
		* The length byte and the FCF byte are the only two bytes required
		* to know that the packet is valid and requested an ack.  The destination
		* address is useful when we want to sniff packets from other transmitters
		* while acknowledging packets that were destined for our local address.
		*/
		if (call CC2420Config.isAutoAckEnabled() && !call CC2420Config.isHwAutoAckDefault()) {
			if (((( header->fcf >> IEEE154_FCF_ACK_REQ ) & 0x01) == 1)
				&& ((header->dest == call CC2420Config.getShortAddr()) || (header->dest == AM_BROADCAST_ADDR))
				&& ((( header->fcf >> IEEE154_FCF_FRAME_TYPE ) & 7) == IEEE154_TYPE_DATA)) {
				// CSn flippage cuts off our FIFO; SACK and begin reading again
				call CSN.set();
				call CSN.clr();
				call SACK.strobe();
				call CSN.set();

				call CSN.clr();
				call RXFIFO.beginRead(buf + 1 + SACK_HEADER_LENGTH, rxFrameLength - SACK_HEADER_LENGTH);
				RXFIFOreadDone((uint8_t*)(call CC2420PacketBody.getHeader( m_p_rx_buf )));
				//call UartLog.logTxRx(DBG_FLAG, DBG_SPI_FLAG, 5, status, 0, call CSN.get(), 0, 0, rx_cnt);
				return;
			}
		}
		// Didn't flip CSn, we're ok to continue reading.
		call RXFIFO.continueRead(buf + 1 + SACK_HEADER_LENGTH, rxFrameLength - SACK_HEADER_LENGTH);
		RXFIFOreadDone((uint8_t*)(call CC2420PacketBody.getHeader( m_p_rx_buf )));
		break;

    case S_RX_PAYLOAD:
		call CSN.set();
		if (!m_missed_packets) {
			// Release the SPI only if there are no more frames to download
			call SpiResource.release();
		}
	#ifndef TIMESYNC_FIX
		//new packet is buffered up, or we don't have timestamp in fifo, or ack
		if ((m_missed_packets && call FIFO.get() ) || !call FIFOP.get() || !m_timestamp_size || rxFrameLength <= 10) {
			call PacketTimeStamp.clear(m_p_rx_buf);
		} else {
			if (m_timestamp_size == 1)
				call PacketTimeStamp.set(m_p_rx_buf, m_timestamp_queue[m_timestamp_head]);
			m_timestamp_head = (m_timestamp_head + 1) % TIMESTAMP_QUEUE_SIZE;
			m_timestamp_size--;

			if (m_timestamp_size > 0) {
				call PacketTimeStamp.clear(m_p_rx_buf);
				m_timestamp_head = 0;
				m_timestamp_size = 0;
			}
		}
	#endif
		// We may have received an ack that should be processed by Transmit
		// buf[rxFrameLength] >> 7 checks the CRC
//		call UartLog.logEntry(DBG_FLAG, DBG_SPI_FLAG, 6, (call CC2420PacketBody.getHeader(m_p_rx_buf))->type);
		// ?????????? rx_buf
		if ((buf[rxFrameLength] >> 7) && rx_buf) {
			uint8_t type = (header->fcf >> IEEE154_FCF_FRAME_TYPE) & 7;
			signal CC2420Receive.receive(type, m_p_rx_buf);
			if (type == IEEE154_TYPE_DATA) {
//				atomic call UartLog.logTxRx(DBG_FLAG, DBG_DELAY_FLAG, 201, (call CC2420PacketBody.getHeader(m_p_rx_buf))->type, (call CC2420PacketBody.getHeader(m_p_rx_buf))->length, 0, 0, m_missed_packets, call LocalTime.get() - start_time);
//				start_time = call LocalTime.get();
				receiveDone();
//				atomic call UartLog.logTxRx(DBG_FLAG, DBG_DELAY_FLAG, 200, (call CC2420PacketBody.getHeader(m_p_rx_buf))->type, (call CC2420PacketBody.getHeader(m_p_rx_buf))->length, 0, 0, m_missed_packets, call LocalTime.get() - start_time);
				return;
			}
		}
		waitForNextPacket();
		break;

    default:
		atomic receivingPacket = FALSE;
		call CSN.set();
		call SpiResource.release();
		break;
    }
}

async event void RXFIFO.writeDone( uint8_t* tx_buf, uint8_t tx_len, error_t error ) {}
  
  /***************** Tasks *****************/
  /**
   * Fill in metadata details, pass the packet up the stack, and
   * get the next packet.
   */
task void receiveDone_task() {
	message_t *tmp;
	
	atomic tmp = m_p_process_buf;
	#warning debug ftsp
	if ((call CC2420PacketBody.getHeader(tmp))->type != TYPE_SYNC) {
		tmp = signal FastReceive.receive(tmp, tmp->data, (call CC2420PacketBody.getHeader(tmp))->length - CC2420_SIZE);
	} else {
		tmp = signal Receive.receive(tmp, tmp->data, (call CC2420PacketBody.getHeader(tmp))->length - CC2420_SIZE);
	}
	
	atomic {
		processing = FALSE;
		m_p_process_buf = tmp;
	}
}

void receiveDone() {
    cc2420_metadata_t* metadata = call CC2420PacketBody.getMetadata( m_p_rx_buf );
    cc2420_header_t* header = call CC2420PacketBody.getHeader( m_p_rx_buf);
    uint8_t length = header->length;
    uint8_t tmpLen __DEPUTY_UNUSED__ = sizeof(message_t) - (offsetof(message_t, data) - sizeof(cc2420_header_t));
    uint8_t* COUNT(tmpLen) buf = TCAST(uint8_t* COUNT(tmpLen), header);

    metadata->crc = buf[ length ] >> 7;
    metadata->lqi = buf[ length ] & 0x7f;
    metadata->rssi = buf[ length - 1 ];
#if defined(SIGNAL_MAP)
	atomic metadata->rssi_idle = rssi_val;
#endif

	if (passesAddressCheck(m_p_rx_buf) && length >= CC2420_SIZE) {
	#ifdef TIMESYNC_FIX
//			if ((call FIFO.get() || !call FIFOP.get()) || ((TBCCTL1 & CM_3) >> 14) != 1) {
			if (call FIFO.get() || !call FIFOP.get()) {
				// RXFIFO not empty; later packet arrives
				call PacketTimeStamp.clear(m_p_rx_buf);
			} else {
				call UartLog.logEntry(DBG_FLAG, DBG_FTSP_FLAG, ((TBCCTL1 & CM_3) >> 14) == 1, call CC2420Config.isAddressRecognitionEnabled() + 10 * call CC2420Config.isHwAddressRecognitionDefault());
				// RXFIFO empty; no other packet arrived yet, I'm the only one
				call PacketTimeStamp.set(m_p_rx_buf, call TransmitInfo.getTime32(TBCCR1));
			}
	#endif
//#warning disable taskize
		// speed up critical path in data plane
		if (AM_IMAC_LE == header->type) {
	  		m_p_rx_buf = signal FastReceive.receive(m_p_rx_buf, m_p_rx_buf->data, length - CC2420_SIZE);
//	  		if (header->type != 243 || header->src != 123)
//	  			call UartLog.logTxRx(DBG_FLAG, DBG_DELAY_FLAG, 210, 0, 0, length, header->src, header->dest, header->type);
	  	} else {
	  		// defer intensive computation of control packet
			if (!processing) {
		  		message_t *tmp;
		  		
		  		processing = TRUE;
		  		// swap buffer
		  		tmp = m_p_rx_buf;
		  		m_p_rx_buf = m_p_process_buf;
		  		m_p_process_buf = tmp;
		  		
		  		post receiveDone_task();
		  	} // else: miss the packet since the previous one is not processed yet
		}
    }
    atomic receivingPacket = FALSE;
    waitForNextPacket();
}

default async event message_t *FastReceive.receive(message_t *msg, void *payload, uint8_t len) {
	return msg;
}

  /****************** CC2420Config Events ****************/
  async event void CC2420Config.syncDone( error_t error ) {
  }
  
  /****************** Functions ****************/
  /**
   * Attempt to acquire the SPI bus to receive a packet.
   */
  void beginReceive() { 
//	start_time = call LocalTime.get();
    m_state = S_RX_LENGTH;
    atomic receivingPacket = TRUE;
	#ifdef TIMESYNC_FIX
		call CaptureSFD.disable();
	#endif

    if(call SpiResource.isOwner()) {
      receive();
      
    } else if (call SpiResource.immediateRequest() == SUCCESS) {
      receive();
      
    } else {
      call SpiResource.request();
    }
  }
  
/**
* Flush out the Rx FIFO
*/
void flush() {
	reset_state();
	call CSN.set();
	call CSN.clr();
	call SFLUSHRX.strobe();
	call SFLUSHRX.strobe();
	call CSN.set();
	call SpiResource.release();
	waitForNextPacket();
}

  /**
   * The first byte of each packet is the length byte.  Read in that single
   * byte, and then read in the rest of the packet.  The CC2420 could contain
   * multiple packets that have been buffered up, so if something goes wrong, 
   * we necessarily want to flush out the FIFO unless we have to.
   */
  void receive() {
//  	cc2420_status_t status;
    call CSN.clr();
    call RXFIFO.beginRead( (uint8_t*)(call CC2420PacketBody.getHeader( m_p_rx_buf )), 1 );
    atomic RXFIFOreadDone((uint8_t*)(call CC2420PacketBody.getHeader( m_p_rx_buf )));
//	call UartLog.logTxRx(DBG_FLAG, DBG_SPI_FLAG, 2, status, call TxStatus.getStatus(), call CSN.get(), 0, 0, rx_cnt);
  }


  /**
   * Determine if there's a packet ready to go, or if we should do nothing
   * until the next packet arrives
   */
  void waitForNextPacket() {
    atomic {
      if ( m_state == S_STOPPED ) {
        call SpiResource.release();
        return;
      }
      
      atomic receivingPacket = FALSE;
	#ifdef TIMESYNC_FIX
		call CaptureSFD.captureRisingEdge();
	#endif
      /*
       * The FIFOP pin here is high when there are 0 bytes in the RX FIFO
       * and goes low as soon as there are bytes in the RX FIFO.  The pin
       * is inverted from what the datasheet says, and its threshold is 127.
       * Whenever the FIFOP line goes low, as you can see from the interrupt
       * handler elsewhere in this module, it means we received a new packet.
       * If the line stays low without generating an interrupt, that means
       * there's still more data to be received.
       */
		// XL: check FIFO besides FIFOP???
      if ( ( m_missed_packets && call FIFO.get() ) || !call FIFOP.get() ) {
        // A new packet is buffered up and ready to go
        if ( m_missed_packets ) {
          m_missed_packets--;
        }
		beginReceive();
      } else {
        // Wait for the next packet to arrive
        m_state = S_STARTED;
        m_missed_packets = 0;
        call SpiResource.release();
      }
    }
  }
  
  /**
   * Reset this component
   */
  void reset_state() {
    m_bytes_left = RXFIFO_SIZE;
    atomic receivingPacket = FALSE;
    m_timestamp_head = 0;
    m_timestamp_size = 0;
    m_missed_packets = 0;
  }

  /**
   * @return TRUE if the given message passes address recognition
   */
  bool passesAddressCheck(message_t *msg) {
    cc2420_header_t *header = call CC2420PacketBody.getHeader( msg );
    
    if(!(call CC2420Config.isAddressRecognitionEnabled())) {
      return TRUE;
    }
    
    return (header->dest == call CC2420Config.getShortAddr()
        || header->dest == AM_BROADCAST_ADDR);
  }
}
