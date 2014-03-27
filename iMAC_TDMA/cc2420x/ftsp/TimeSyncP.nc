/*
 * Copyright (c) 2002, Vanderbilt University
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement is
 * hereby granted, provided that the above copyright notice, the following
 * two paragraphs and the author appear in all copies of this software.
 *
 * IN NO EVENT SHALL THE VANDERBILT UNIVERSITY BE LIABLE TO ANY PARTY FOR
 * DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT
 * OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE VANDERBILT
 * UNIVERSITY HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * THE VANDERBILT UNIVERSITY SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND THE VANDERBILT UNIVERSITY HAS NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 *
 * @author: Miklos Maroti, Brano Kusy (kusy@isis.vanderbilt.edu), Janos Sallai
 * Ported to T2: 3/17/08 by Brano Kusy (branislav.kusy@gmail.com)
 */
 
/*
 * Xiaohui's ftsp FTSP_FIX
 * 1) everyone assumes itself to be root unless hear from otherwise from smaller id node
 * 2) overrides previous root's global time
 * 3) assume lowest id node never leaves
 */

#include "TimeSyncMsg.h"
#include <Tasklet.h>

generic module TimeSyncP(typedef precision_tag)
{
    provides
    {
        interface Init;
        interface StdControl;
        interface GlobalTime<precision_tag>;

        //interfaces for extra functionality: need not to be wired
//        interface TimeSyncInfo;
        interface TimeSyncMode;
    }
    uses
    {
        interface Boot;
        interface SplitControl as RadioControl;
        interface AsyncTimeSyncAMSend<precision_tag,uint32_t> as Send;
        interface AsyncReceive as Receive;
        interface AsyncTimeSyncPacket<precision_tag,uint32_t> as TimeSyncPacket;
//        interface Timer<TMilli>;
        interface Random;
        interface Leds;
        interface LocalTime<precision_tag> as LocalTime;

#ifdef LOW_POWER_LISTENING
        interface LowPowerListening;
#endif
		// XL
//		interface CC2420Config;
		interface RadioState;
		interface PacketField<uint8_t> as PacketTransmitPower;
//		interface CC2420PacketBody;
		interface ForwarderInfo;
		interface UartLog;
    }
}
implementation
{
#ifndef TIMESYNC_RATE
#define TIMESYNC_RATE   10
#endif
uint32_t start_time;
    enum {
    	// XL: non-default parameters
        MAX_ENTRIES           = 64, //8,              // number of entries in the table
        ENTRY_VALID_LIMIT     = MAX_ENTRIES, //4,              // number of entries to become synchronized
        ENTRY_SEND_LIMIT      = MAX_ENTRIES,  //3,              // number of entries to send sync messages
        BEACON_RATE           = TIMESYNC_RATE,  // how often send the beacon msg (in seconds)
        ROOT_TIMEOUT          = 5,              //time to declare itself the root if no msg was received (in sync periods)
        IGNORE_ROOT_MSG       = 4,              // after becoming the root ignore other roots messages (in send period)
    	// enlarge bcoz of us, not 32khz or ms
        ENTRY_THROWOUT_LIMIT  = 500 * 32,    // if time sync error is bigger than this clear the table
    };
//#warning ENTRY_THROWOUT_LIMIT
//// not in enum bcoz of signed integer issue, e.g., 1 is less than -ENTRY_THROWOUT_LIMIT
//const int32_t ENTRY_THROWOUT_LIMIT = 100000; //60000L;

    typedef struct TableItem
    {
        uint8_t     state;
        uint32_t    localTime;
        int32_t     timeOffset; // globalTime - localTime
    } TableItem;

    enum {
        ENTRY_EMPTY = 0,
        ENTRY_FULL = 1,
    };

    TableItem   table[MAX_ENTRIES];
    uint8_t tableEntries;

    enum {
    	// not used
        //STATE_IDLE = 0x00,
        STATE_PROCESSING = 0x01,
        STATE_SENDING = 0x02,
        STATE_INIT = 0x04,
    };

    uint8_t state, mode;

/*
    We do linear regression from localTime to timeOffset (globalTime - localTime).
    This way we can keep the slope close to zero (ideally) and represent it
    as a float with high precision.

        timeOffset - offsetAverage = skew * (localTime - localAverage)
        timeOffset = offsetAverage + skew * (localTime - localAverage)
        globalTime = localTime + offsetAverage + skew * (localTime - localAverage)
*/

    float       skew;
    uint32_t    localAverage;
    int32_t     offsetAverage;
    uint8_t     numEntries; // the number of full entries in the table

    message_t processedMsgBuffer;
    message_t* processedMsg;

    message_t outgoingMsgBuffer;
    TimeSyncMsg* outgoingMsg;

	// XL: # of beacons sent since the last reference point inserted into regression table, except before it finds a root, where heartBeats denotes beacon periods elapsed
    uint8_t heartBeats; // the number of sucessfully sent messages
                        // since adding a new entry with lower beacon id than ours
	
	// XL:
	uint16_t seqno = 0;
	//uint32_t local_time;
	
    async command uint32_t GlobalTime.getLocalTime()
    {
        return call LocalTime.get();
    }

    async command error_t GlobalTime.getGlobalTime(uint32_t *time)
    {
        *time = call GlobalTime.getLocalTime();
        return call GlobalTime.local2Global(time);
    }

	async command uint16_t GlobalTime.getRoot()
    {
    	return outgoingMsg->rootID;
    }

    error_t is_synced()
    {
      if (numEntries>=ENTRY_VALID_LIMIT || outgoingMsg->rootID==TOS_NODE_ID)
        return SUCCESS;
      else
        return FAIL;
    }


    async command error_t GlobalTime.local2Global(uint32_t *time)
    {
    //XL: special case for root
#ifdef FTSP_FIX
    if (outgoingMsg->rootID != TOS_NODE_ID)
#endif
        atomic *time += offsetAverage + (int32_t)(skew * (int32_t)(*time - localAverage));
        return is_synced();
    }

    async command error_t GlobalTime.global2Local(uint32_t *time)
    {
        uint32_t approxLocalTime = *time - offsetAverage;
    //XL: special case for root
#ifdef FTSP_FIX
    if (outgoingMsg->rootID != TOS_NODE_ID)
#endif
        *time = approxLocalTime - (int32_t)(skew * (int32_t)(approxLocalTime - localAverage));
        return is_synced();
    }
	
	// XL
	async command error_t GlobalTime.global2LocalInterval(uint32_t *interval) {
		*interval = (float) *interval / (1 + skew);
		return is_synced();
	}
	async command error_t GlobalTime.getSkew(float *skew_p) {
		*skew_p = skew;
		return is_synced();
	}
	
 void printTable();
    void calculateConversion()
    {
        float newSkew = skew;
        uint32_t newLocalAverage;
        int32_t newOffsetAverage;
        int32_t localAverageRest;
        int32_t offsetAverageRest;

        int64_t localSum;
        int64_t offsetSum;

        int8_t i;

        for(i = 0; i < MAX_ENTRIES && table[i].state != ENTRY_FULL; ++i)
            ;

        if( i >= MAX_ENTRIES )  // table is empty
            return;
/*
        We use a rough approximation first to avoid time overflow errors. The idea
        is that all times in the table should be relatively close to each other.
*/
        newLocalAverage = table[i].localTime;
        newOffsetAverage = table[i].timeOffset;

        localSum = 0;
        localAverageRest = 0;
        offsetSum = 0;
        offsetAverageRest = 0;

        while( ++i < MAX_ENTRIES )
            if( table[i].state == ENTRY_FULL ) {
                /*
                   This only works because C ISO 1999 defines the signe for modulo the same as for the Dividend!
                */ 
                localSum += (int32_t)(table[i].localTime - newLocalAverage) / tableEntries;
                localAverageRest += (table[i].localTime - newLocalAverage) % tableEntries;
                offsetSum += (int32_t)(table[i].timeOffset - newOffsetAverage) / tableEntries;
                offsetAverageRest += (table[i].timeOffset - newOffsetAverage) % tableEntries;
            }

        newLocalAverage += localSum + localAverageRest / tableEntries;
        newOffsetAverage += offsetSum + offsetAverageRest / tableEntries;

        localSum = offsetSum = 0;
        for(i = 0; i < MAX_ENTRIES; ++i)
            if( table[i].state == ENTRY_FULL ) {
                int32_t a = table[i].localTime - newLocalAverage;
                int32_t b = table[i].timeOffset - newOffsetAverage;

                localSum += (int64_t)a * a;
                offsetSum += (int64_t)a * b;
            }

        if( localSum != 0 )
            newSkew = (float)offsetSum / (float)localSum;

        atomic
        {
            skew = newSkew;
            offsetAverage = newOffsetAverage;
            localAverage = newLocalAverage;
            numEntries = tableEntries;
        }
		// XL: abnormal skew after sync
//		if (((SUCCESS == is_synced()) && (newSkew > 0.0001 || newSkew < -0.0001)))
//			printTable();
    }

// XL: debug large skew
uint16_t send_cnt = 0;
uint16_t fire_cnt = 0;
void printTable() {
	uint8_t i;
	uint32_t timeOffset_abs;
	TableItem *te;
	float skew_abs = (skew > 0) ? skew : -skew;
	uint32_t offsetAverage_abs;
	
	atomic offsetAverage_abs = (offsetAverage > 0) ? offsetAverage : -offsetAverage;
	
	atomic call UartLog.logTxRx(DBG_FLAG, DBG_FTSP_FLAG, __LINE__, skew > 0, offsetAverage > 0, localAverage, offsetAverage_abs >> 16, offsetAverage_abs, skew_abs * 1000000UL);
	for (i = 0; i < MAX_ENTRIES; i++) {
		te = &table[i];
		timeOffset_abs = (te->timeOffset > 0) ? te->timeOffset : -te->timeOffset;
		if (te->state == ENTRY_FULL) {
//    	call UartLog.logTxRx(DBG_FLAG, DBG_FTSP_FLAG, i, outgoingMsg->rootID, te->timeOffset > 0, timeOffset_abs >> 16, timeOffset_abs, skew_abs * 1000000UL, te->localTime);
    		call UartLog.logTxRx(DBG_FLAG, DBG_FTSP_FLAG, __LINE__, i, outgoingMsg->rootID, te->timeOffset > 0, timeOffset_abs >> 16, timeOffset_abs, te->localTime);
    	}
	}
}

    void clearTable()
    {
        int8_t i;
        for(i = 0; i < MAX_ENTRIES; ++i)
            table[i].state = ENTRY_EMPTY;

        atomic numEntries = 0;
    }

    uint8_t numErrors=0;
    void addNewEntry(TimeSyncMsg *msg)
    {
        int8_t i, freeItem = -1, oldestItem = 0;
        uint32_t age, oldestTime = 0;
        int32_t timeError;

        // clear table if the received entry's been inconsistent for some time
        timeError = msg->localTime;
        call GlobalTime.local2Global((uint32_t*)(&timeError));
        timeError -= msg->globalTime;
//        if (is_synced() == SUCCESS)
//        	call UartLog.logEntry(DBG_FLAG, DBG_FTSP_FLAG, __LINE__, timeError);
        if( (is_synced() == SUCCESS) &&
            (timeError > ENTRY_THROWOUT_LIMIT || timeError < -ENTRY_THROWOUT_LIMIT))
        {
//        	uint32_t now;
//        	call GlobalTime.getGlobalTime(&now);
//           	call UartLog.logTxRx(DBG_FLAG, DBG_FTSP_FLAG, __LINE__, 0, 0, outgoingMsg->rootID, numErrors, timeError > 0, (timeError > 0) ? timeError : - timeError);

            if (++numErrors > 3) {
                clearTable();
                call UartLog.logEntry(DBG_FLAG, DBG_FTSP_FLAG, __LINE__, msg->globalTime);
            }
            return; // don't incorporate a bad reading
        }

        tableEntries = 0; // don't reset table size unless you're recounting
        numErrors = 0;

        for(i = 0; i < MAX_ENTRIES; ++i) {
            age = msg->localTime - table[i].localTime;

            //logical time error compensation
            if( age >= 0x7FFFFFFFL )
                table[i].state = ENTRY_EMPTY;

            if( table[i].state == ENTRY_EMPTY )
                freeItem = i;
            else
                ++tableEntries;

            if( age >= oldestTime ) {
                oldestTime = age;
                oldestItem = i;
            }
        }
		// XL: FIFO cache
		// 	if empty found
		//		replace
		//	else
		//		replace oldest entry
        if( freeItem < 0 )
            freeItem = oldestItem;
        else
            ++tableEntries;

        table[freeItem].state = ENTRY_FULL;

        table[freeItem].localTime = msg->localTime;
        table[freeItem].timeOffset = msg->globalTime - msg->localTime;
    }

    void task processMsg()
    {
        TimeSyncMsg* msg;
        atomic msg = (TimeSyncMsg*)(call Send.getPayload(processedMsg, sizeof(TimeSyncMsg)));
		
        if( msg->rootID < outgoingMsg->rootID
        #ifndef FTSP_FIX
            // jw: after becoming the root ignore other roots messages (in send period)
            && !(heartBeats < IGNORE_ROOT_MSG && outgoingMsg->rootID == TOS_NODE_ID)
        #endif
        ) {
//            call UartLog.logEntry(DBG_FLAG, DBG_FTSP_FLAG, msg->rootID, outgoingMsg->rootID);
            outgoingMsg->rootID = msg->rootID;
            outgoingMsg->seqNum = msg->seqNum;
        #ifdef FTSP_FIX
        	// override previous root's time
        	clearTable();
        #endif
        }
        else if( outgoingMsg->rootID == msg->rootID && (int8_t)(msg->seqNum - outgoingMsg->seqNum) > 0 ) {
        	// XL: only beacon based on newer global time is admitted
            outgoingMsg->seqNum = msg->seqNum;
        }
        else
            goto exit;

        call Leds.led0Toggle();
        if( outgoingMsg->rootID < TOS_NODE_ID )
            atomic heartBeats = 0;

        addNewEntry(msg);
        calculateConversion();

    exit:
        atomic state &= ~STATE_PROCESSING;
//		call UartLog.logTxRx(DBG_FLAG, DBG_FTSP_FLAG, __LINE__, call ForwarderInfo.isForwarderEnabled(), 0, msg->rootID, msg->nodeID, msg->seqNum, outgoingMsg->seqNum);
    }

    async event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len)
    {
#ifdef TIMESYNC_DEBUG   // this code can be used to simulate multiple hopsf
        uint8_t incomingID = (uint8_t)((TimeSyncMsg*)payload)->nodeID;
        int8_t diff = (incomingID & 0x0F) - (TOS_NODE_ID & 0x0F);
        if( diff < -1 || diff > 1 )
            return msg;
        diff = (incomingID & 0xF0) - (TOS_NODE_ID & 0xF0);
        if( diff < -16 || diff > 16 )
            return msg;
#endif
		uint8_t state_;
		atomic state_ = state;
		// XL: STATE_PROCESSING prevents overriding
        if( (state_ & STATE_PROCESSING) == 0 && call TimeSyncPacket.isValid(msg)) {
            message_t *old;
            atomic {
		        old = processedMsg;
		        processedMsg = msg;
            }
            // XL: convert sender's localTime into my localTime, now TimeSyncMsg.localTime store reference point's local time in my clock
            ((TimeSyncMsg*)(payload))->localTime = call TimeSyncPacket.eventTime(msg);

            atomic state |= STATE_PROCESSING;
            post processMsg();

            return old;
        }

        return msg;
    }

    error_t sendMsg()
    {
        uint32_t localTime, globalTime;

        globalTime = localTime = call GlobalTime.getLocalTime();
        call GlobalTime.local2Global(&globalTime);

        // we need to periodically update the reference point for the root
        // to avoid wrapping the 32-bit (localTime - localAverage) value
        if( outgoingMsg->rootID == TOS_NODE_ID ) {
            if( (int32_t)(localTime - localAverage) >= 0x20000000 )
            {
                atomic
                {
                    localAverage = localTime;
                    offsetAverage = globalTime - localTime;
                }
            }
        }
      #ifndef FTSP_FIX
        else if( heartBeats >= ROOT_TIMEOUT ) {
            heartBeats = 0; //to allow ROOT_SWITCH_IGNORE to work
            outgoingMsg->rootID = TOS_NODE_ID;
            ++(outgoingMsg->seqNum); // maybe set it to zero?
        }
      #endif

		// XL
		outgoingMsg->seqno = seqno++;
//#warning fill global time w/ local to verify pkt-level timesync
//        outgoingMsg->globalTime = localTime;
        outgoingMsg->globalTime = globalTime;
        // we don't send time sync msg, if we don't have enough data
        if( numEntries < ENTRY_SEND_LIMIT && outgoingMsg->rootID != TOS_NODE_ID ) {
            atomic {
            	++heartBeats;
            	state &= ~STATE_SENDING;
            }
        } else {
			// XL: packet-level sync of localTime
			error_t ret;
			//#warning ftsp also uses power 31
			call PacketTransmitPower.set(&outgoingMsgBuffer, CONTROL_POWER_LEVEL);
			ret = call Send.send(AM_BROADCAST_ADDR, &outgoingMsgBuffer, TIMESYNCMSG_LEN, localTime);
//			call UartLog.logTxRx(DBG_FLAG, DBG_FTSP_FLAG, __LINE__, call ForwarderInfo.isForwarderEnabled(), ret, outgoingMsg->rootID, outgoingMsg->nodeID, outgoingMsg->seqNum, outgoingMsg->globalTime);
			if (ret != SUCCESS) {
				atomic state &= ~STATE_SENDING;
			} else {
				// XL
				//local_time = localTime;
				return SUCCESS;
			}
       	}
        return FAIL;
	}

    async event void Send.sendDone(message_t* ptr, error_t error)
    {
    	// switch back b4 slot ends; doesn't hurt to switch even forwarder not started
    	//call CC2420Config.switchChannel(CC2420_DEF_CHANNEL);
    	//call RadioState.setChannel(CC2420X_DEF_CHANNEL);
    	
		if (ptr != &outgoingMsgBuffer) {
			assert(0);
			return;
		}
		
        if (error == SUCCESS)
        {
        	// do not log here bcoz outgoingMsg->seqNum may have been overwritten btw. send() and sendDone()
//        	call UartLog.logTxRx(DBG_FLAG, DBG_FTSP_FLAG, __LINE__, call ForwarderInfo.isForwarderEnabled(), outgoingMsg->seqno, outgoingMsg->rootID, outgoingMsg->nodeID, outgoingMsg->seqNum, outgoingMsg->globalTime);
        	//uint32_t event_time = (call CC2420PacketBody.getMetadata(ptr))->event_time;
        	//call UartLog.logTxRx(DBG_FLAG, DBG_FTSP_FLAG, __LINE__, event_time == local_time, outgoingMsg->globalTime == local_time, outgoingMsg->seqno, event_time >> 16, event_time, local_time);
            atomic ++heartBeats;
            call Leds.led1Toggle();

            if( outgoingMsg->rootID == TOS_NODE_ID )
                ++(outgoingMsg->seqNum);
        }

        atomic state &= ~STATE_SENDING;
		//atomic call UartLog.logEntry(DBG_FLAG, DBG_FTSP_FLAG, __LINE__, call LocalTime.get() - local_time);
    }
    
    error_t timeSyncMsgSend() {}
    
    // return SUCCESS means beacon submitted to lower layer, expecting sendDone signalled later
	async command error_t GlobalTime.sendFtspBeacon() {
		error_t ret;
	#ifndef FTSP_FIX
        if( outgoingMsg->rootID == 0xFFFF && ++heartBeats >= ROOT_TIMEOUT ) {
            outgoingMsg->seqNum = 0;
            outgoingMsg->rootID = TOS_NODE_ID;
        }
	#endif
		uint8_t state_;
		atomic state_ = state;
		//local_time = call LocalTime.get();
//        call UartLog.logTxRx(DBG_FLAG, DBG_FTSP_FLAG, __LINE__, call ForwarderInfo.isForwarderEnabled(), outgoingMsg->seqno, outgoingMsg->rootID, numEntries, state_, ENTRY_THROWOUT_LIMIT);
        if( outgoingMsg->rootID != 0xFFFF && (state_ & STATE_SENDING) == 0 ) {
	       atomic state |= STATE_SENDING;
	       //post sendMsg();
	       ret = sendMsg();
	       return ret;
        }
        return FAIL;
    }

// XL
//async event void CC2420Config.syncDone(error_t error) {}
tasklet_async event void RadioState.done() {}

//    event void Timer.fired()
//    {
//    	fire_cnt++;
//      if (mode == TS_TIMER_MODE) {
//        timeSyncMsgSend();
//      }
//      else
//        call Timer.stop();
//    }

    command error_t TimeSyncMode.setMode(uint8_t mode_){
//        if (mode_ == TS_TIMER_MODE){
//            call Timer.startPeriodic((uint32_t)(896U+(call Random.rand16()&0xFF)) * BEACON_RATE);
//        }
//        else
//            call Timer.stop();

        mode = mode_;
        return SUCCESS;
    }

    command uint8_t TimeSyncMode.getMode(){
        return mode;
    }

    command error_t TimeSyncMode.send(){
        if (mode == TS_USER_MODE){
            timeSyncMsgSend();
            return SUCCESS;
        }
        return FAIL;
    }

    command error_t Init.init()
    {
        atomic{
            skew = 0.0;
            localAverage = 0;
            offsetAverage = 0;
        };

        clearTable();

        atomic outgoingMsg = (TimeSyncMsg*)call Send.getPayload(&outgoingMsgBuffer, sizeof(TimeSyncMsg));
	#ifndef FTSP_FIX
        outgoingMsg->rootID = 0xFFFF;
	#else
		// assume the smallest id node does not depart online
        outgoingMsg->rootID = TOS_NODE_ID;
        outgoingMsg->seqNum = 0;
	#endif
        processedMsg = &processedMsgBuffer;
        atomic state = STATE_INIT;

        return SUCCESS;
    }

    event void Boot.booted()
    {
      call RadioControl.start();
      call StdControl.start();
    }

    command error_t StdControl.start()
    {
        atomic heartBeats = 0;
        outgoingMsg->nodeID = TOS_NODE_ID;
        call TimeSyncMode.setMode(TS_TIMER_MODE);

        return SUCCESS;
    }

    command error_t StdControl.stop()
    {
//        call Timer.stop();
        return SUCCESS;
    }

//    async command float     TimeSyncInfo.getSkew() { return skew; }
//    async command uint32_t  TimeSyncInfo.getOffset() { return offsetAverage; }
//    async command uint32_t  TimeSyncInfo.getSyncPoint() { return localAverage; }
//    async command uint16_t  TimeSyncInfo.getRootID() { return outgoingMsg->rootID; }
//    async command uint8_t   TimeSyncInfo.getSeqNum() { return outgoingMsg->seqNum; }
//    async command uint8_t   TimeSyncInfo.getNumEntries() { return numEntries; }
//    async command uint8_t   TimeSyncInfo.getHeartBeats() { return heartBeats; }

    event void RadioControl.startDone(error_t error){}
    event void RadioControl.stopDone(error_t error){}
}
