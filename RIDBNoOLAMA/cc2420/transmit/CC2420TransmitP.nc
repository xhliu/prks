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
 * @author Jung Il Choi Initial SACK implementation
 * @author JeongGil Ko
 * @author Razvan Musaloiu-E
 * @version $Revision: 1.16 $ $Date: 2009/10/28 21:09:52 $
 */

#include "CC2420.h"
#include "CC2420TimeSyncMessage.h"
#include "crc.h"
#include "message.h"

module CC2420TransmitP @safe() {

  provides interface Init;
  provides interface StdControl;
  provides interface CC2420Transmit as Send;
  provides interface RadioBackoff;
  provides interface ReceiveIndicator as EnergyIndicator;
  provides interface ReceiveIndicator as ByteIndicator;
	// XL
  provides interface TransmitInfo;
  
  uses interface Alarm<T32khz,uint32_t> as BackoffTimer;
  uses interface CC2420Packet;
  uses interface CC2420PacketBody;
  uses interface PacketTimeStamp<T32khz,uint32_t>;
  uses interface PacketTimeSyncOffset;
  uses interface GpioCapture as CaptureSFD;
  uses interface GeneralIO as CCA;
  uses interface GeneralIO as CSN;
  uses interface GeneralIO as SFD;
#ifdef TIMESYNC_FIX
  uses interface GeneralIO as FIFO;
  uses interface GeneralIO as FIFOP;
#endif
  uses interface Resource as SpiResource;
  uses interface ChipSpiResource;
  uses interface CC2420Fifo as TXFIFO;
  uses interface CC2420Ram as TXFIFO_RAM;
  uses interface CC2420Register as TXCTRL;
  uses interface CC2420Strobe as SNOP;
  uses interface CC2420Strobe as STXON;
  uses interface CC2420Strobe as STXONCCA;
  uses interface CC2420Strobe as SFLUSHTX;
  uses interface CC2420Register as MDMCTRL1;

  uses interface CC2420Strobe as STXENC;
  uses interface CC2420Register as SECCTRL0;
  uses interface CC2420Register as SECCTRL1;
  uses interface CC2420Ram as KEY0;
  uses interface CC2420Ram as KEY1;
  uses interface CC2420Ram as TXNONCE;

  uses interface CC2420Receive;
  uses interface Leds;
	uses interface Crc;
	uses interface UartLog;
	uses interface Util;
	uses interface LocalTime<T32khz>;
}

implementation {

  typedef enum {
    S_STOPPED,
    S_STARTED,
    S_LOAD,
    S_SAMPLE_CCA,
    S_BEGIN_TRANSMIT,
    S_SFD,
    S_EFD,
    S_ACK_WAIT,
    S_CANCEL,
  } cc2420_transmit_state_t;

  // This specifies how many jiffies the stack should wait after a
  // TXACTIVE to receive an SFD interrupt before assuming something is
  // wrong and aborting the send. There seems to be a condition
  // on the micaZ where the SFD interrupt is never handled.
  enum {
    CC2420_ABORT_PERIOD = 320
  };

  norace message_t * ONE_NOK m_msg;
  
  norace bool m_cca;
  
  norace uint8_t m_tx_power;
  
  cc2420_transmit_state_t m_state;

  bool m_receiving = FALSE;
  
  uint16_t m_prev_time;
  
  /** Byte reception/transmission indicator */
  bool sfdHigh;
  
  /** Let the CC2420 driver keep a lock on the SPI while waiting for an ack */
  // XL: so CC2420ReceiveP can request and acquire SPI immediately when ack is received
  bool abortSpiRelease;
  
  /** Total CCA checks that showed no activity before the NoAck LPL send */
  norace int8_t totalCcaChecks;
  
  /** The initial backoff period */
  norace uint16_t myInitialBackoff;
  
  /** The congestion backoff period */
  norace uint16_t myCongestionBackoff;
  
  // XL
  uint32_t start_time;

  /***************** Prototypes ****************/
  error_t send( message_t * ONE p_msg, bool cca );
  error_t resend( bool cca );
  error_t loadTXFIFO();
  error_t TXFIFOWriteDone();
  error_t attemptSend();
  void congestionBackoff();
  error_t acquireSpiResource();
  error_t releaseSpiResource();
  void signalDone( error_t err, uint8_t id);
uint8_t pkt_type;
  
  /***************** Init Commands *****************/
  command error_t Init.init() {
    // XL
    atomic m_state = S_STOPPED;
    call CCA.makeInput();
    call CSN.makeOutput();
    call SFD.makeInput();
    return SUCCESS;
  }

  /***************** StdControl Commands ****************/
  command error_t StdControl.start() {
    atomic {
      call CaptureSFD.captureRisingEdge();
      m_state = S_STARTED;
      m_receiving = FALSE;
      abortSpiRelease = FALSE;
      m_tx_power = 0;
    }
    return SUCCESS;
  }

  command error_t StdControl.stop() {
    atomic {
      m_state = S_STOPPED;
      call BackoffTimer.stop();
      call CaptureSFD.disable();
      call SpiResource.release();  // REMOVE
      call CSN.set();
    }
    return SUCCESS;
  }


/**************** Send Commands ****************/
async command error_t Send.send(message_t* ONE p_msg, bool useCca) {
	return send(p_msg, useCca);
}

  async command error_t Send.resend(bool useCca) {
    return resend( useCca );
  }

  async command error_t Send.cancel() {
    atomic {
      switch( m_state ) {
      case S_LOAD:
      case S_SAMPLE_CCA:
      case S_BEGIN_TRANSMIT:
        m_state = S_CANCEL;
        break;
        
      default:
        // cancel not allowed while radio is busy transmitting
        return FAIL;
      }
    }

    return SUCCESS;
  }

  async command error_t Send.modify( uint8_t offset, uint8_t* buf, uint8_t len ) {
    call CSN.clr();
    call TXFIFO_RAM.write( offset, buf, len );
    call CSN.set();
    return SUCCESS;
  }
  
  /***************** Indicator Commands ****************/
  command bool EnergyIndicator.isReceiving() {
    return !(call CCA.get());
  }
  
  command bool ByteIndicator.isReceiving() {
    bool high;
    atomic high = sfdHigh;
    return high;
  }
  

  /***************** RadioBackoff Commands ****************/
  /**
   * Must be called within a requestInitialBackoff event
   * @param backoffTime the amount of time in some unspecified units to backoff
   */
  async command void RadioBackoff.setInitialBackoff(uint16_t backoffTime) {
    myInitialBackoff = backoffTime + 1;
  }
  
  /**
   * Must be called within a requestCongestionBackoff event
   * @param backoffTime the amount of time in some unspecified units to backoff
   */
  async command void RadioBackoff.setCongestionBackoff(uint16_t backoffTime) {
    myCongestionBackoff = backoffTime + 1;
  }
  
  async command void RadioBackoff.setCca(bool useCca) {
  }
  
  
  async command uint32_t TransmitInfo.getTime32(uint16_t time)
  {
    uint32_t recent_time=call BackoffTimer.getNow();
    return recent_time + (int16_t)(time - recent_time);
  }

  /**
   * The CaptureSFD event is actually an interrupt from the capture pin
   * which is connected to timing circuitry and timer modules.  This
   * type of interrupt allows us to see what time (being some relative value)
   * the event occurred, and lets us accurately timestamp our packets.  This
   * allows higher levels in our system to synchronize with other nodes.
   *
   * Because the SFD events can occur so quickly, and the interrupts go
   * in both directions, we set up the interrupt but check the SFD pin to
   * determine if that interrupt condition has already been met - meaning,
   * we should fall through and continue executing code where that interrupt
   * would have picked up and executed had our microcontroller been fast enough.
   */


async event void CaptureSFD.captured( uint16_t time ) {
	uint32_t time32;
#ifndef TIMESYNC_FIX
	uint8_t sfd_state = 0;
#endif
atomic {
#ifndef TIMESYNC_FIX
	time32 = call TransmitInfo.getTime32(time);
#else
	time32 = call TransmitInfo.getTime32(TBCCR1);
#endif
	
	switch( m_state ) {
		case S_SFD:
			m_state = S_EFD;
			sfdHigh = TRUE;
			// in case we got stuck in the receive SFD interrupts, we can reset
			// the state here since we know that we are not receiving anymore
			m_receiving = FALSE;
			call CaptureSFD.captureFallingEdge();
		#ifndef TIMESYNC_FIX
			call PacketTimeStamp.set(m_msg, time32);
		#else
			if (call FIFO.get() || !call FIFOP.get()) {
				// RXFIFO not empty; later packet arrives
				call PacketTimeStamp.clear(m_msg);
			} else {
				// RXFIFO empty; no other packet arrived yet, I'm the only one
				call PacketTimeStamp.set(m_msg, time32);
			}
		#endif
			if (call PacketTimeSyncOffset.isSet(m_msg)) {
				uint8_t absOffset = sizeof(message_header_t)-sizeof(cc2420_header_t)+call PacketTimeSyncOffset.get(m_msg);
			#ifndef TIMESYNC_FIX
				#warning CRC not applied!
				timesync_radio_t *timesync = (timesync_radio_t *)((nx_uint8_t*)m_msg+absOffset);
				// set timesync event time as the offset between the event time and the SFD interrupt time (TEP  133)
				*timesync  -= time32;
				call CSN.clr();
				call TXFIFO_RAM.write(absOffset, (uint8_t*)timesync, sizeof(timesync_radio_t) );
				call CSN.set();
				//restoring the event time to the original value
				*timesync  += time32;
			#else
				nx_uint32_t event_time;
				timesync_radio_t timesync;
				nx_uint16_t crc;
				event_time = (call CC2420PacketBody.getMetadata(m_msg))->event_time;
				event_time -= time32;
//		#warning override event_time to debug ftsp
//				event_time = time32;
		
				timesync.timestamp = event_time;
				timesync.timestamp_overriden = TIMESTAMP_OVERRIDEN;
				// update CRC accordingly
				crc = call Crc.seededCrc16((call CC2420PacketBody.getMetadata(m_msg))->partial_crc, (void *)&timesync, sizeof(timesync_radio_t));
				
				call CSN.clr();
				call TXFIFO_RAM.write(absOffset, (uint8_t*)&timesync, sizeof(timesync_radio_t));
				call CSN.set();
				call CSN.clr();
				call TXFIFO_RAM.write(absOffset + sizeof(timesync_radio_t), (uint8_t*)&crc, sizeof(crc));
				call CSN.set();
			#endif
			}

			if ( (call CC2420PacketBody.getHeader( m_msg ))->fcf & ( 1 << IEEE154_FCF_ACK_REQ ) ) {
				// This is an ack packet, don't release the chip's SPI bus lock.
				abortSpiRelease = TRUE;
			}
			releaseSpiResource();
			call BackoffTimer.stop();

			if (call SFD.get()) {
				break;
			}
		/** Fall Through because the next interrupt was already received */

		case S_EFD:
			sfdHigh = FALSE;
			call CaptureSFD.captureRisingEdge();
//			atomic call UartLog.logTxRx(DBG_FLAG, DBG_DELAY_FLAG, 101, (call CC2420PacketBody.getHeader(m_msg))->type, (call CC2420PacketBody.getHeader(m_msg))->length, 0, 0, 0, call LocalTime.get() - start_time);
//			atomic start_time = call LocalTime.get();
			
			if ((call CC2420PacketBody.getHeader(m_msg))->fcf & (1 << IEEE154_FCF_ACK_REQ)) {
//				atomic call UartLog.logTxRx(DBG_FLAG, DBG_DELAY_FLAG, 110, (call CC2420PacketBody.getHeader(m_msg))->type, (call CC2420PacketBody.getHeader(m_msg))->dsn, 0, 0, 0, call LocalTime.get() - start_time);
				m_state = S_ACK_WAIT;
				call BackoffTimer.start(CC2420_ACK_WAIT_DELAY);
			} else {
				signalDone(SUCCESS, 3);
			}

			if (!call SFD.get()) {
				break;
			}
		/** Fall Through because the next interrupt was already received */

		default:
		#ifndef TIMESYNC_FIX
			/* this is the SFD for received messages */
			if ( !m_receiving && sfdHigh == FALSE ) {
				sfdHigh = TRUE;
				call CaptureSFD.captureFallingEdge();
				// safe the SFD pin status for later use
				sfd_state = call SFD.get();
				call CC2420Receive.sfd( time32 );
				m_receiving = TRUE;
				m_prev_time = time;
				if (call SFD.get()) {
					// wait for the next interrupt before moving on
					return;
				}
				// if SFD.get() = 0, then an other interrupt happened since we
				// reconfigured CaptureSFD! Fall through
			}

			if ( sfdHigh == TRUE ) {
				sfdHigh = FALSE;
				call CaptureSFD.captureRisingEdge();
				m_receiving = FALSE;
				/* if sfd_state is 1, then we fell through, but at the time of
				* saving the time stamp the SFD was still high. Thus, the timestamp
				* is valid.
				* if the sfd_state is 0, then either we fell through and SFD
				* was low while we safed the time stamp, or we didn't fall through.
				* Thus, we check for the time between the two interrupts.
				* FIXME: Why 10 tics? Seems like some magic number...
				*/
				if ((sfd_state == 0) && (time - m_prev_time < 10) ) {
					call CC2420Receive.sfd_dropped();
					if (m_msg)
						call PacketTimeStamp.clear(m_msg);
				}
				break;
			}
		#endif
	}
}
}

  /***************** ChipSpiResource Events ****************/
  async event void ChipSpiResource.releasing() {
    if(abortSpiRelease) {
      call ChipSpiResource.abortRelease();
    }
  }
  
  
  /***************** CC2420Receive Events ****************/
  /**
   * If the packet we just received was an ack that we were expecting,
   * our send is complete.
   */
  async event void CC2420Receive.receive( uint8_t type, message_t* ack_msg ) {
    cc2420_header_t* ack_header;
    cc2420_header_t* msg_header;
    cc2420_metadata_t* msg_metadata;
    uint8_t* ack_buf;
    uint8_t length;

    if ( type == IEEE154_TYPE_ACK && m_msg) {
      ack_header = call CC2420PacketBody.getHeader( ack_msg );
      msg_header = call CC2420PacketBody.getHeader( m_msg );
      
      if ( m_state == S_ACK_WAIT && msg_header->dsn == ack_header->dsn ) {
        call BackoffTimer.stop();
        
        msg_metadata = call CC2420PacketBody.getMetadata( m_msg );
        ack_buf = (uint8_t *) ack_header;
        length = ack_header->length;
        
        msg_metadata->ack = TRUE;
        msg_metadata->rssi = ack_buf[ length - 1 ];
        msg_metadata->lqi = ack_buf[ length ] & 0x7f;
        signalDone(SUCCESS, 0);
      }
    }
  }

  /***************** SpiResource Events ****************/
   event void SpiResource.granted() {
    uint8_t cur_state;

    atomic {
      cur_state = m_state;
    }

    switch( cur_state ) {
    case S_LOAD:
      loadTXFIFO();
      break;
      
    case S_BEGIN_TRANSMIT:
      attemptSend();
      break;
      
    case S_CANCEL:
      call CSN.clr();
      call SFLUSHTX.strobe();
      call CSN.set();
      releaseSpiResource();
      atomic {
        m_state = S_STARTED;
      }
      signal Send.sendDone( m_msg, ECANCEL );
      break;
      
    default:
      releaseSpiResource();
      break;
    }
  }
  
  /***************** TXFIFO Events ****************/
  /**
   * The TXFIFO is used to load packets into the transmit buffer on the
   * chip
   */
async event void TXFIFO.readDone( uint8_t* tx_buf, uint8_t tx_len, error_t error ) {}
  
  
  /***************** Timer Events ****************/
  /**
   * The backoff timer is mainly used to wait for a moment before trying
   * to send a packet again. But we also use it to timeout the wait for
   * an acknowledgement, and timeout the wait for an SFD interrupt when
   * we should have gotten one.
   */
  async event void BackoffTimer.fired() {
    atomic {
      switch( m_state ) {
        
      case S_SAMPLE_CCA : 
        // sample CCA and wait a little longer if free, just in case we
        // sampled during the ack turn-around window
        if ( call CCA.get() ) {
          m_state = S_BEGIN_TRANSMIT;
          call BackoffTimer.start( CC2420_TIME_ACK_TURNAROUND );
          
        } else {
          congestionBackoff();
        }
        break;
        
      case S_BEGIN_TRANSMIT:
      case S_CANCEL:
        if ( acquireSpiResource() == SUCCESS ) {
          attemptSend();
        }
        break;
        
      case S_ACK_WAIT:
        signalDone(SUCCESS, 1);
        break;

      case S_SFD:
        // We didn't receive an SFD interrupt within CC2420_ABORT_PERIOD
        // jiffies. Assume something is wrong.
        call SFLUSHTX.strobe();
        call CaptureSFD.captureRisingEdge();
        releaseSpiResource();
        signalDone(ERETRY, 2);
        break;

      default:
        break;
      }
    }
  }
      
/***************** Functions ****************/
/**
* Set up a message to be sent. First load it into the outbound tx buffer
* on the chip, then attempt to send it.
* @param *p_msg Pointer to the message that needs to be sent
* @param cca TRUE if this transmit should use clear channel assessment
* XL 
* @return: SUCCESS only if packet is sent; 
* include, but are not limited to, ECANCEL if dropped after CCA; FAIL if previous packet not finished tx yet
*/
error_t send( message_t* ONE p_msg, bool cca ) {
	//    atomic start_time = call LocalTime.get();
	atomic {
		if (m_state == S_CANCEL) {
			return ECANCEL;
		}
		// XL: this prevents overriding the previous pending packet
		if ( m_state != S_STARTED ) {
			return FAIL;
		}
	#ifdef TIMESYNC_FIX
		// an SFD is pending, i.e., a packet is received and to be read
		if (call FIFO.get() || !call FIFOP.get()) {
			return FAIL;
		}
		call CaptureSFD.disable();
	#endif
		m_state = S_LOAD;
		m_cca = cca;
		m_msg = p_msg;
		totalCcaChecks = 0;
	}
	
	if (acquireSpiResource() == SUCCESS) {
		return loadTXFIFO();
	}
	return SUCCESS;
}

  /** 
   * Setup the packet transmission power and load the tx fifo buffer on
   * the chip with our outbound packet.  
   *
   * Warning: the tx_power metadata might not be initialized and
   * could be a value other than 0 on boot.  Verification is needed here
   * to make sure the value won't overstep its bounds in the TXCTRL register
   * and is transmitting at max power by default.
   *
   * It should be possible to manually calculate the packet's CRC here and
   * tack it onto the end of the header + payload when loading into the TXFIFO,
   * so the continuous modulation low power listening strategy will continually
   * deliver valid packets.  This would increase receive reliability for
   * mobile nodes and lossy connections.  The crcByte() function should use
   * the same CRC polynomial as the CC2420's AUTOCRC functionality.
   */
//void loadTXFIFO() {
error_t loadTXFIFO() {
	cc2420_header_t* header = call CC2420PacketBody.getHeader( m_msg );
	uint8_t tx_power = (call CC2420PacketBody.getMetadata( m_msg ))->tx_power;
	
	if ( !tx_power ) {
		tx_power = CC2420_DEF_RFPOWER;
	}
	
	call CSN.clr();
	
	if ( m_tx_power != tx_power ) {
		call TXCTRL.write( ( 2 << CC2420_TXCTRL_TXMIXBUF_CUR ) |
						 ( 3 << CC2420_TXCTRL_PA_CURRENT ) |
						 ( 1 << CC2420_TXCTRL_RESERVED ) |
						 ( (tx_power & 0x1F) << CC2420_TXCTRL_PA_LEVEL ) );
	}
	
	m_tx_power = tx_power;
	
	{
		uint8_t tmpLen __DEPUTY_UNUSED__ = header->length - 1;
		// XL: non split-phase
		call TXFIFO.write(TCAST(uint8_t * COUNT(tmpLen), header), header->length - 1);
		return TXFIFOWriteDone();
	}
}

#ifdef DUMMY_SPI
async event void TXFIFO.writeDone( uint8_t* tx_buf, uint8_t tx_len, error_t error ) {}
error_t TXFIFOWriteDone() {
	call CSN.set();
	atomic {
		m_state = S_BEGIN_TRANSMIT;
	}
	return attemptSend();
}
#else
error_t TXFIFOWriteDone() {
	return SUCCESS;
}
async event void TXFIFO.writeDone( uint8_t* tx_buf, uint8_t tx_len, error_t error ) {
	call CSN.set();
	atomic {
		m_state = S_BEGIN_TRANSMIT;
	}
	attemptSend();
}
#endif
/**
* Attempt to send the packet we have loaded into the tx buffer on 
* the radio chip.  The STXONCCA will send the packet immediately if
* the channel is clear.  If we're not concerned about whether or not
* the channel is clear (i.e. m_cca == FALSE), then STXON will send the
* packet without checking for a clear channel.
*
* If the packet didn't get sent, then congestion == TRUE.  In that case,
* we reset the backoff timer and try again in a moment.
*
* If the packet got sent, we should expect an SFD interrupt to take
* over, signifying the packet is getting sent.
* 
* If security is enabled, STXONCCA or STXON will perform inline security
* options before transmitting the packet.
*/
// tx on CONTROL channel or rx
bool is_ctrl_sender;
error_t attemptSend() {
	uint8_t status;
	bool congestion = TRUE;
	uint8_t type = (call CC2420PacketBody.getHeader(m_msg))->type;
	
	atomic {
		call CSN.clr();
		// DATA pkt not CCA
		status = (type != AM_IMAC_LE) ? call STXONCCA.strobe() : call STXON.strobe();
	#ifdef TIMESYNC_FIX
		call CaptureSFD.captureRisingEdge();
	#endif
		if (!(status & CC2420_STATUS_TX_ACTIVE)) {
			status = call SNOP.strobe();
			if (status & CC2420_STATUS_TX_ACTIVE) {
				congestion = FALSE;
			}
		}
		// not sent
		if (congestion) {
			// just try once; otherwise may cross slot boundary
			call SFLUSHTX.strobe();
			releaseSpiResource();
			m_state = S_STARTED;
		#ifndef DUMMY_SPI
			// send() does not reach here; returns before TXFIFO loading finishes
			signal Send.sendDone(m_msg, ECANCEL);
		#endif
		} else {
			// sent
			m_state = S_SFD;
			call BackoffTimer.start(CC2420_ABORT_PERIOD);
//			atomic call UartLog.logTxRx(DBG_FLAG, DBG_DELAY_FLAG, 100, (call CC2420PacketBody.getHeader(m_msg))->type, (call CC2420PacketBody.getHeader(m_msg))->dsn, 0, 0, 0, call LocalTime.get() - start_time);
//			atomic start_time = call LocalTime.get();
		}
		call CSN.set();
		// iff not DATA channel and sent
		is_ctrl_sender = ((type != AM_IMAC_LE) && !congestion);
	}
	return (congestion ? ECANCEL : SUCCESS);
}

// tx on CONTROL channel or rx depending on CCA; only applies to CTRL channel
async command bool TransmitInfo.isSender() {
	return is_ctrl_sender;
}
// void TXFIFOWriteDone() {
// 	cc2420_transmit_state_t m_state_;
// 	atomic m_state_ = m_state;
// 	
// 	call CSN.set();
// 	//if ( m_state == S_CANCEL ) {
// 	if ( m_state_ == S_CANCEL ) {
// 		atomic {
// 			call CSN.clr();
// 			call SFLUSHTX.strobe();
// 			call CSN.set();
// 		}
// 		releaseSpiResource();
// 		atomic m_state = S_STARTED;
// 		signal Send.sendDone( m_msg, ECANCEL );
// 	
// 	} else if ( !m_cca ) {
// 		atomic {
// 			m_state = S_BEGIN_TRANSMIT;
// 		}
// 		attemptSend();
// 	} else {
// 		releaseSpiResource();
// 		atomic {
// 			m_state = S_SAMPLE_CCA;
// 		}
// 		
// 		signal RadioBackoff.requestInitialBackoff(m_msg);
// 		call BackoffTimer.start(myInitialBackoff);
// 	}
// }

  

  /**
   * Resend a packet that already exists in the outbound tx buffer on the
   * chip
   * @param cca TRUE if this transmit should use clear channel assessment
   */
  error_t resend( bool cca ) {

    atomic {
      if (m_state == S_CANCEL) {
        return ECANCEL;
      }
      
      if ( m_state != S_STARTED ) {
        return FAIL;
      }
      
      m_cca = cca;
      m_state = cca ? S_SAMPLE_CCA : S_BEGIN_TRANSMIT;
      totalCcaChecks = 0;
    }
    
    if(m_cca) {
      signal RadioBackoff.requestInitialBackoff(m_msg);
      call BackoffTimer.start( myInitialBackoff );
      
    } else if ( acquireSpiResource() == SUCCESS ) {
      attemptSend();
    }
    
    return SUCCESS;
  }



//void attemptSend() {
//	uint8_t status;
//	bool congestion = TRUE;

//atomic {
////#warning diable ftsp channel check
////#ifndef TIMESYNC_FIX
//	if (m_state == S_CANCEL) {
////#else
////	if (m_state == S_CANCEL || 
////	// XL: ftsp beacon is only sent in control channel
////	(AM_TIMESYNCMSG == (call CC2420PacketBody.getHeader(m_msg))->type && call CC2420Config.getChannel() != CC2420_CONTROL_CHANNEL)) {
////#endif
//		call SFLUSHTX.strobe();
//		releaseSpiResource();
//		call CSN.set();
//		m_state = S_STARTED;
//		signal Send.sendDone( m_msg, ECANCEL );
//		return;
//	}
//	call CSN.clr();
//	status = m_cca ? call STXONCCA.strobe() : call STXON.strobe();
//	if ( !( status & CC2420_STATUS_TX_ACTIVE ) ) {
//		status = call SNOP.strobe();
//		if ( status & CC2420_STATUS_TX_ACTIVE ) {
//			congestion = FALSE;
//		}
//	}

//	m_state = congestion ? S_SAMPLE_CCA : S_SFD;
//	call CSN.set();
//}

//	if ( congestion ) {
//		// XL: sanity check
////		call UartLog.logEntry(DBG_FLAG, DBG_DELAY_FLAG, 100, 0);
//		totalCcaChecks = 0;
//		releaseSpiResource();
//		congestionBackoff();
//	} else {
//		call BackoffTimer.start(CC2420_ABORT_PERIOD);
////		atomic call UartLog.logTxRx(DBG_FLAG, DBG_DELAY_FLAG, 100, (call CC2420PacketBody.getHeader(m_msg))->type, (call CC2420PacketBody.getHeader(m_msg))->dsn, 0, 0, 0, call LocalTime.get() - start_time);
//		atomic start_time = call LocalTime.get();
//	}
//}
  
  
  /**  
   * Congestion Backoff
   */
  void congestionBackoff() {
    atomic {
      signal RadioBackoff.requestCongestionBackoff(m_msg);
      call BackoffTimer.start(myCongestionBackoff);
    }
  }
  
  error_t acquireSpiResource() {
    error_t error = call SpiResource.immediateRequest();
    if ( error != SUCCESS ) {
      call SpiResource.request();
    }
    return error;
  }

  error_t releaseSpiResource() {
    call SpiResource.release();
    return SUCCESS;
  }


void signalDone(error_t err, uint8_t id) {
//	cc2420_header_t *header = call CC2420PacketBody.getHeader(m_msg);
//	atomic call UartLog.logTxRx(DBG_FLAG, DBG_DELAY_FLAG, 103, (call CC2420PacketBody.getHeader(m_msg))->type, id, 0, 0, 0, call LocalTime.get() - start_time);
	atomic m_state = S_STARTED;
	abortSpiRelease = FALSE;
	call ChipSpiResource.attemptRelease();
//	call UartLog.logTxRx(DBG_FLAG, DBG_DELAY_FLAG, 110, 0, 0, 0, header->src, header->dest, header->type);
	signal Send.sendDone( m_msg, err );
}

}

