/*
 *  Copyright (c) 2016-2022, The OpenThread Authors.
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
 *   This file includes the platform-specific initializers and all headers for
 *   platform-specific functions (platform....() ).
 */

#ifndef PLATFORM_SIMULATION_H_
#define PLATFORM_SIMULATION_H_

#include <openthread-core-config.h>
#include <openthread/config.h>

#include <assert.h>
#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>

#include <openthread/instance.h>

#include "platform-config.h"
#include "event-sim.h"

#define UNDEFINED_TIME_US 0  // an undefined period of time (us) that is > 0

enum {
    MAX_NETWORK_SIZE = OPENTHREAD_SIMULATION_MAX_NETWORK_SIZE,
};

/**
 * Unique node ID.
 *
 */
extern uint32_t gNodeId;

/**
 * ID of last received Alarm event from simulator, or 0 if no ID yet received.
 */
extern uint64_t gLastAlarmEventId;

/**
 * This function initializes the alarm service used by OpenThread.
 *
 */
void platformAlarmInit(uint32_t aSpeedUpFactor);

/**
 * This function retrieves the time remaining until the alarm fires.
 *
 * @param[out]  aTimeout  A pointer to the timeval struct.
 *
 */
void platformAlarmUpdateTimeout(struct timeval *aTimeout);

/**
 * This function performs alarm driver processing.
 *
 * @param[in]  aInstance  The OpenThread instance structure.
 *
 */
void platformAlarmProcess(otInstance *aInstance);

/**
 * This function returns the duration to the next alarm event time (in micro seconds)
 *
 * @returns The duration (in micro seconds, us) to the next alarm event.
 *
 */
uint64_t platformAlarmGetNext(void);

/**
 * This function returns the current alarm time.
 *
 * @returns The current alarm time (us).
 *
 */
uint64_t platformAlarmGetNow(void);

/**
 * This function advances the alarm time by @p aDelta.
 *
 * @param[in]  aDelta  The amount of time (us) to advance.
 *
 */
void platformAlarmAdvanceNow(uint64_t aDelta);

/**
 * This function initializes the radio service used by OpenThread.
 *
 */
void platformRadioInit(void);

/**
 * This function shuts down the radio service used by OpenThread.
 *
 */
void platformRadioDeinit(void);

/**
 * This function performs radio driver processing.
 *
 * @param[in]  aInstance    The OpenThread instance structure.
 * @param[in]  aReadFdSet   A pointer to the read file descriptors.
 * @param[in]  aWriteFdSet  A pointer to the write file descriptors.
 *
 */
void platformRadioProcess(otInstance *aInstance, const fd_set *aReadFdSet, const fd_set *aWriteFdSet);

/**
 * This function initializes the random number service used by OpenThread.
 *
 */
void platformRandomInit(void);

/**
 * This function restores the Uart.
 *
 */
void platformUartRestore(void);

/**
 * This function checks if radio needs to transmit a pending MAC (data) frame.
 *
 * @returns Whether radio frame Tx is pending (true) or not (false).
 *
 */
bool platformRadioIsTransmitPending(void);

/**
 * This function lets the radio report its state to the simulator, for bookkeeping and
 * energy-monitoring purposes.
 *
 */
void platformRadioReportStateToSimulator(void);

/**
 * This function checks if the radio is busy performing some task such as transmission,
 * actively receiving a frame, returning an ACK, or doing a CCA. Idle listening (Rx) does
 * not count as busy.
 *
 * @returns Whether radio is busy with a task.
 *
 */
bool platformRadioIsBusy(void);

/**
 * This function signals the start of a received radio frame.
 *
 * @param[in]  aInstance   A pointer to the OpenThread instance.
 * @param[in]  aRxParams   A pointer to parameters related to the reception event.
 *
 */
void platformRadioRxStart(otInstance *aInstance, struct RadioCommEventData *aRxParams);

/**
 * This function signals the end of a received radio frame and inputs the frame data.
 *
 * @param[in]  aInstance   A pointer to the OpenThread instance.
 * @param[in]  aBuf        A pointer to the received radio frame (struct RadioMessage).
 * @param[in]  aBufLength  The size of the received radio frame (struct RadioMessage).
 * @param[in]  aRxParams   A pointer to parameters related to the reception event.
 *
 */
void platformRadioRxDone(otInstance *aInstance, const uint8_t *aBuf, uint16_t aBufLength, struct RadioCommEventData *aRxParams);

/**
 * This function signals that virtual radio is done transmitting a single frame.
 *
 * @param[in]  aInstance     A pointer to the OpenThread instance.
 * @param[in]  aTxDoneParams A pointer to status parameters for the attempt to transmit the virtual radio frame.
 *
 */
void platformRadioTxDone(otInstance *aInstance, struct RadioCommEventData *aTxDoneParams);

/**
 * This function signals that virtual radio is done with the CCA procedure.
 *
 * @param[in]  aInstance     A pointer to the OpenThread instance.
 * @param[in]  aTxDoneParams A pointer to status result of the CCA procedure.
 *
 */
void platformRadioCcaDone(otInstance *aInstance, struct RadioCommEventData *aChanData);


#if OPENTHREAD_CONFIG_RADIO_LINK_TREL_ENABLE

/**
 * This function initializes the TREL service.
 *
 * @param[in] aSpeedUpFactor   The time speed-up factor.
 *
 */
void platformTrelInit(uint32_t aSpeedUpFactor);

/**
 * This function shuts down the TREL service.
 *
 */
void platformTrelDeinit(void);

/**
 * This function updates the file descriptor sets with file descriptors used by the TREL.
 *
 * @param[in,out]  aReadFdSet   A pointer to the read file descriptors.
 * @param[in,out]  aWriteFdSet  A pointer to the write file descriptors.
 * @param[in,out]  aTimeout     A pointer to the timeout.
 * @param[in,out]  aMaxFd       A pointer to the max file descriptor.
 *
 */
void platformTrelUpdateFdSet(fd_set *aReadFdSet, fd_set *aWriteFdSet, struct timeval *aTimeout, int *aMaxFd);

/**
 * This function performs TREL processing.
 *
 * @param[in]  aInstance    The OpenThread instance structure.
 * @param[in]  aReadFdSet   A pointer to the read file descriptors.
 * @param[in]  aWriteFdSet  A pointer to the write file descriptors.
 *
 */
void platformTrelProcess(otInstance *aInstance, const fd_set *aReadFdSet, const fd_set *aWriteFdSet);

#endif // OPENTHREAD_CONFIG_RADIO_LINK_TREL_ENABLE

#endif // PLATFORM_SIMULATION_H_
