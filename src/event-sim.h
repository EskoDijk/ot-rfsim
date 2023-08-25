/*
*  Copyright (c) 2020-2023, The OpenThread Authors.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*  1. Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*  2. Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the
*     documentation and/or other materials provided with the distribution.
*  3. Neither the name of the copyright holder nor the
*     names of its contributors may be used to endorse or promote products
*     derived from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
*  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
*  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
*  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
*  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
*  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
*  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
*  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*/

/**
* @file
* @brief
*   This file includes simulation-event message definitions, and formatting and
 *   parsing functions.
*/

#ifndef PLATFORM_RFSIM_EVENT_SIM_H
#define PLATFORM_RFSIM_EVENT_SIM_H

#include "platform-rfsim.h"
#include "radio.h"

/**
 * The event types defined for communication with a simulator and/or with other simulated nodes.
 * Shared for both 'real' and virtual-time event types.
 */
enum
{
    OT_SIM_EVENT_ALARM_FIRED         = 0,
    OT_SIM_EVENT_RADIO_RECEIVED      = 1,
    OT_SIM_EVENT_UART_WRITE          = 2,
    OT_SIM_EVENT_RADIO_SPINEL_WRITE  = 3,
    OT_SIM_EVENT_POSTCMD             = 4,
    OT_SIM_EVENT_OTNS_STATUS_PUSH    = 5,
    OT_SIM_EVENT_RADIO_COMM_START    = 6,
    OT_SIM_EVENT_RADIO_TX_DONE       = 7,
    OT_SIM_EVENT_RADIO_CHAN_SAMPLE   = 8,
    OT_SIM_EVENT_RADIO_STATE         = 9,
    OT_SIM_EVENT_RADIO_RX_DONE       = 10,
    OT_SIM_EVENT_EXT_ADDR            = 11,
    OT_SIM_EVENT_NODE_INFO           = 12,
};

#define OT_EVENT_DATA_MAX_SIZE 1024

OT_TOOL_PACKED_BEGIN
struct EventHeader
{
    uint64_t mDelay;
    uint8_t  mEvent;
    uint16_t mDataLength;
} OT_TOOL_PACKED_END;

OT_TOOL_PACKED_BEGIN
struct Event
{
    uint64_t mDelay;
    uint8_t  mEvent;
    uint16_t mDataLength;
    uint8_t  mData[OT_EVENT_DATA_MAX_SIZE];
} OT_TOOL_PACKED_END;

OT_TOOL_PACKED_BEGIN
struct RadioCommEventData
{
    uint8_t  mChannel;
    int8_t   mPower;      // power value (dBm), RSSI or Tx-power
    uint8_t  mError;      // status code result of radio operation
    uint64_t mDuration;   // us duration of the radio comm operation
} OT_TOOL_PACKED_END;

OT_TOOL_PACKED_BEGIN
struct RadioStateEventData
{
    uint8_t  mChannel;
    int8_t   mTxPower;  // only valid when mEnergyState == OT_RADIO_STATE_TRANSMIT
    uint8_t  mEnergyState; // energy-state of radio (disabled, sleep, actively Tx, actively Rx)
    uint8_t  mSubState;
    uint8_t  mState; // OT state of radio (disabled, sleep, Tx, Rx)
    uint64_t mRadioTime; // the radio's time otPlatRadioGetNow()
} OT_TOOL_PACKED_END;

#define EVENT_TO_STRING(evt)
/**
 * This function sends a generic simulation event to the simulator. Event fields are
 * updated to the values as were used for sending it.
 *
 * @param[in,out]   aEvent  A pointer to the simulation event to send.
 *
 */
void otSimSendEvent(struct Event *aEvent);

/**
 * This function sends a sleep event to the simulator. The amount of time to sleep
 * for this node is determined by the alarm timer, by calling platformAlarmGetNext().
 */
void otSimSendSleepEvent(void);

/**
 * This function sends a RadioComm (Tx) simulation event to the simulator.
 *
 * @param[in]       aEventData A pointer to specific data for RadioComm event.
 * @param[in]       aPayload     A pointer to the data payload (radio frame) to send.
 * @param[in]       aLenPayload  Length of aPayload data.
 */
void otSimSendRadioCommEvent(struct RadioCommEventData *aEventData,  const uint8_t *aPayload, size_t aLenPayload);

/**
 * This function sends a Radio State simulation event to the simulator. It reports radio state
 * and indicates for how long the current radio-state will last until next state-change.
 *
 * @param[in]  aStateData                 A pointer to specific data for Radio State event.
 * @param[in]  aDeltaUntilNextRadioState  Time (us) until next radio-state change event, or UNDEFINED_TIME_US.
 */
void otSimSendRadioStateEvent(struct RadioStateEventData *aStateData, uint64_t aDeltaUntilNextRadioState);

/**
 * This functions sends a channel sample simulation event to the simulator. It is used
 * for CCA or energy scanning on channels.
 *
 * @param[in]  aChanData    A pointer to channel-sample data instructing what to sample.
 */
void otSimSendRadioChanSampleEvent(struct RadioCommEventData *aChanData);

/**
 * This function sends a Uart data event to the simulator.
 *
 * @param[in]   aData       A pointer to the UART data.
 * @param[in]   aLength     Length of UART data.
 *
 */
void otSimSendUartWriteEvent(const uint8_t *aData, uint16_t aLength);

/**
 * This function sends status push data event to the OT-NS simulator.
 *
 * @param[in]   aStatus     A pointer to the status string data.
 * @param[in]   aLength     Length of status string data.
 *
 */
void otSimSendOtnsStatusPushEvent(const char *aStatus, uint16_t aLength);

/**
 * This function send Extended Address change event to the simulator.
 * It differs from an OTNS Status Push 'extaddr' event in being not
 * encoded as a string, but binary.
 *
 * @param aExtAddress    The (new) Extended Address of the node.
 */
void otSimSendExtAddrEvent(const otExtAddress *aExtAddress);

/**
 * This function send OT node information to the simulator.
 */
void otSimSendNodeInfoEvent(uint32_t nodeId);

#endif // PLATFORM_RFSIM_EVENT_SIM_H
