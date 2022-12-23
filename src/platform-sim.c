/*
 *  Copyright (c) 2018-2022, The OpenThread Authors.
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
 *   This file includes the platform-specific initializers and processing functions.
 */

#include "platform-simulation.h"
#include "event-sim.h"

#include <assert.h>
#include <errno.h>
#include <libgen.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <syslog.h>

#include <openthread/tasklet.h>

#include "utils/uart.h"

uint32_t gNodeId        = 1;
uint64_t gLastAlarmEventId = 0;

extern bool          gPlatformPseudoResetWasRequested;
static volatile bool gTerminate = false;

int    gArgumentsCount = 0;
char **gArguments      = NULL;

uint64_t sNow = 0; // microseconds
int      sSockFd;
uint16_t sPortBase = 9000;
uint16_t sPortOffset;

static void handleSignal(int aSignal)
{
    OT_UNUSED_VARIABLE(aSignal);

    gTerminate = true;
}

#define VERIFY_EVENT_SIZE(X) assert( (payloadLen >= sizeof(X)) && "received event payload too small" );

static void receiveEvent(otInstance *aInstance)
{
    struct Event event;
    ssize_t      rval = recvfrom(sSockFd, (char *)&event, sizeof(event), 0, NULL, NULL);
    const uint8_t *evData = event.mData;

    if (rval < 0 || (uint16_t)rval < offsetof(struct Event, mData))
    {
        perror("recvfrom");
        exit(EXIT_FAILURE);
    }
    size_t payloadLen = rval - offsetof(struct Event, mData);

    platformAlarmAdvanceNow(event.mDelay);

    switch (event.mEvent)
    {
    case OT_SIM_EVENT_ALARM_FIRED:
        // store the optional msg id from payload
        if (payloadLen >= sizeof(gLastAlarmEventId))
        {
            gLastAlarmEventId = (uint64_t) *evData;
        }
        break;

    case OT_SIM_EVENT_UART_WRITE:
        otPlatUartReceived(event.mData, event.mDataLength);
        break;

    case OT_SIM_EVENT_RADIO_COMM_START:
        VERIFY_EVENT_SIZE(struct RadioCommEventData)
        platformRadioRxStart(aInstance, (struct RadioCommEventData *)evData);
        break;

    case OT_SIM_EVENT_RADIO_RX_DONE:
        VERIFY_EVENT_SIZE(struct RadioCommEventData)
        const size_t sz = sizeof(struct RadioCommEventData);
        platformRadioRxDone(aInstance, evData + sz,
                       event.mDataLength - sz, (struct RadioCommEventData *)evData);
        break;

    case OT_SIM_EVENT_RADIO_TX_DONE:
        VERIFY_EVENT_SIZE(struct RadioCommEventData)
        platformRadioTxDone(aInstance, (struct RadioCommEventData *)evData);
        break;

    case OT_SIM_EVENT_RADIO_CHAN_SAMPLE:
        VERIFY_EVENT_SIZE(struct RadioCommEventData)
        // TODO consider also energy-detect case. This only does CCA now.
        platformRadioCcaDone(aInstance, (struct RadioCommEventData *)evData);
        break;

    case OT_SIM_EVENT_RADIO_STATE:
        // Not further parsed. Simulator uses this to wake the OT node when it's time for a next
        // radio-state transition in the radio-sim.c state machines.
        break;

    default:
        assert(false && "Unrecognized event type received");
    }
}

void platformUartRestore(void)
{
}

otError otPlatUartEnable(void)
{
    return OT_ERROR_NONE;
}

otError otPlatUartDisable(void)
{
    return OT_ERROR_NONE;
}

otError otPlatUartSend(const uint8_t *aData, uint16_t aLength)
{
    otSimSendUartWriteEvent(aData, aLength);
    otPlatUartSendDone();

    return OT_ERROR_NONE;
}

otError otPlatUartFlush(void)
{
    return OT_ERROR_NONE;
}

static void socket_init(void)
{
    struct sockaddr_in sockaddr;
    memset(&sockaddr, 0, sizeof(sockaddr));
    sockaddr.sin_family = AF_INET;

    parseFromEnvAsUint16("PORT_BASE", &sPortBase);

    parseFromEnvAsUint16("PORT_OFFSET", &sPortOffset);
    sPortOffset *= (MAX_NETWORK_SIZE + 1);

    sockaddr.sin_port        = htons((uint16_t)(sPortBase + sPortOffset + gNodeId));
    sockaddr.sin_addr.s_addr = INADDR_ANY;

    sSockFd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    if (sSockFd == -1)
    {
        perror("socket");
        exit(EXIT_FAILURE);
    }

    if (bind(sSockFd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) == -1)
    {
        perror("bind");
        exit(EXIT_FAILURE);
    }
}

void otSysInit(int argc, char *argv[])
{
    char *endptr;

    if (gPlatformPseudoResetWasRequested)
    {
        gPlatformPseudoResetWasRequested = false;
        return;
    }

    if (argc != 2)
    {
        fprintf(stderr, "Usage: %s <nodeNumber>\n", basename(argv[0]));
        exit(EXIT_FAILURE);
    }

    openlog(basename(argv[0]), LOG_PID, LOG_USER);
    setlogmask(setlogmask(0) & LOG_UPTO(LOG_NOTICE));

    gArgumentsCount = argc;
    gArguments      = argv;

    gNodeId = (uint32_t)strtol(argv[1], &endptr, 0);

    if (*endptr != '\0' || gNodeId < 1 || gNodeId > MAX_NETWORK_SIZE)
    {
        fprintf(stderr, "Invalid NodeId: %s (must be 1-%i)\n", argv[1], MAX_NETWORK_SIZE);
        exit(EXIT_FAILURE);
    }

    socket_init();

    platformAlarmInit(1);
    platformRadioInit();
    platformRandomInit();

    signal(SIGTERM, &handleSignal);
    signal(SIGHUP, &handleSignal);
}

bool otSysPseudoResetWasRequested(void)
{
    return gPlatformPseudoResetWasRequested;
}

void otSysDeinit(void)
{
    close(sSockFd);
}

void otSysProcessDrivers(otInstance *aInstance)
{
    fd_set read_fds;
    fd_set write_fds;
    fd_set error_fds;
    int    max_fd = -1;
    int    rval;

    if (gTerminate)
    {
        exit(0);
    }

    FD_ZERO(&read_fds);
    FD_ZERO(&write_fds);
    FD_ZERO(&error_fds);

    FD_SET(sSockFd, &read_fds);
    max_fd = sSockFd;

    if (!otTaskletsArePending(aInstance) && platformAlarmGetNext() > 0 &&
        (!platformRadioIsTransmitPending() || platformRadioIsBusy()) )
    {
        // report my final radio state at end of this time instant, then go to sleep.
        platformRadioReportStateToSimulator();
        otSimSendSleepEvent();

        // wake up by reception of UDP event from simulator.
        rval = select(max_fd + 1, &read_fds, &write_fds, &error_fds, NULL);

        if ((rval < 0) && (errno != EINTR))
        {
            perror("select");
            exit(EXIT_FAILURE);
        }

        if (rval > 0 && FD_ISSET(sSockFd, &read_fds))
        {
            receiveEvent(aInstance);
        }
    }

    platformAlarmProcess(aInstance);
    platformRadioProcess(aInstance, &read_fds, &write_fds);
}

void otPlatOtnsStatus(const char *aStatus)
{
    uint16_t     statusLength = (uint16_t)strlen(aStatus);
    if (statusLength > OT_EVENT_DATA_MAX_SIZE){
        statusLength = OT_EVENT_DATA_MAX_SIZE;
        assert(statusLength <= OT_EVENT_DATA_MAX_SIZE);
    }
    otSimSendOtnsStatusPushEvent(aStatus, statusLength);
}
