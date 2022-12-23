/*
 *  Copyright (c) 2022, The OpenThread Authors.
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
 *   This file includes simulation-event message formatting and parsing functions.
 */

#include "platform-rfsim.h"
#include "event-sim.h"

// UDP communication parameters for events
extern uint16_t sPortOffset;
extern uint16_t sPortBase;
extern int      gSockFd;

void otSimSendSleepEvent(void)
{
    struct Event event;
    assert(platformAlarmGetNext() > 0);

    event.mDelay      = platformAlarmGetNext();
    event.mEvent      = OT_SIM_EVENT_ALARM_FIRED;
    event.mDataLength = sizeof(uint64_t);
    memcpy(event.mData, &gLastAlarmEventId, sizeof(uint64_t));

    otSimSendEvent(&event);
}

void otSimSendRadioCommEvent(struct RadioCommEventData *aEventData, const uint8_t *aPayload, size_t aLenPayload)
{
    struct Event event;
    assert(aLenPayload <= OT_EVENT_DATA_MAX_SIZE);
    event.mEvent = OT_SIM_EVENT_RADIO_COMM_START;
    memcpy(event.mData, aEventData, sizeof(struct RadioCommEventData));
    memcpy(event.mData + sizeof(struct RadioCommEventData), aPayload, aLenPayload);
    event.mDataLength = sizeof(struct RadioCommEventData) + aLenPayload;

    otSimSendEvent(&event);
}

void otSimSendRadioChanSampleEvent(struct RadioCommEventData *aChanData)
{
    struct Event event;
    event.mEvent = OT_SIM_EVENT_RADIO_CHAN_SAMPLE;
    event.mDelay = 0;
    memcpy(event.mData, aChanData, sizeof(struct RadioCommEventData));
    event.mDataLength = sizeof(struct RadioCommEventData);

    otSimSendEvent(&event);
}

void otSimSendRadioStateEvent(struct RadioStateEventData *aStateData, uint64_t aDeltaUntilNextRadioState)
{
    struct Event event;
    event.mEvent = OT_SIM_EVENT_RADIO_STATE;
    event.mDelay = aDeltaUntilNextRadioState;
    memcpy(event.mData, aStateData, sizeof(struct RadioStateEventData));
    event.mDataLength = sizeof(struct RadioStateEventData);

    otSimSendEvent(&event);
}

void otSimSendUartWriteEvent(const uint8_t *aData, uint16_t aLength) {
    assert(aLength <= OT_EVENT_DATA_MAX_SIZE);
    struct Event event;

    event.mEvent      = OT_SIM_EVENT_UART_WRITE;
    event.mDelay      = 0;
    event.mDataLength = aLength;
    memcpy(event.mData, aData, aLength);

    otSimSendEvent(&event);
}

void otSimSendOtnsStatusPushEvent(const char *aStatus, uint16_t aLength) {
    assert(aLength <= OT_EVENT_DATA_MAX_SIZE);
    struct Event event;

    memcpy(event.mData, aStatus, aLength);
    event.mEvent      = OT_SIM_EVENT_OTNS_STATUS_PUSH;
    event.mDelay      = 0;
    event.mDataLength = aLength;

    otSimSendEvent(&event);
}

void otSimSendEvent(struct Event *aEvent)
{
    ssize_t            rval;
    struct sockaddr_in sockaddr;
    memset(&sockaddr, 0, sizeof(sockaddr));
    sockaddr.sin_family = AF_INET;
    inet_pton(AF_INET, "127.0.0.1", &sockaddr.sin_addr);
    sockaddr.sin_port = htons(sPortBase + sPortOffset);

    // send header and data.
    rval = sendto(gSockFd, aEvent, offsetof(struct Event, mData) + aEvent->mDataLength, 0, (struct sockaddr *)&sockaddr,
                  sizeof(sockaddr));

    if (rval < 0)
    {
        perror("sendto");
        exit(EXIT_FAILURE);
    }
}
