/*
 *  Copyright (c) 2016-2023, The OpenThread Authors.
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
 *   This file includes the platform-specific OT system initializers and processing.
 */

#include "platform-rfsim.h"

#include <errno.h>
#include <libgen.h>
#include <sys/socket.h>
#include <sys/un.h>

#include <openthread/tasklet.h>

extern void platformReceiveEvent(otInstance *aInstance);
extern bool gPlatformPseudoResetWasRequested;
static void socket_init(char *socketFilePath);
static void handleSignal(int aSignal);

volatile bool gTerminate = false;
uint32_t gNodeId = 0;
int gSockFd = 0;
uint16_t sPortBase = 9000;
uint16_t sPortOffset;

void otSysInit(int argc, char *argv[]) {
    char *endptr;

    if (gPlatformPseudoResetWasRequested) {
        gPlatformPseudoResetWasRequested = false;
        return;
    }

    if (argc != 3) {
        fprintf(stderr, "Usage: %s <nodeNumber> <OTNS_PID>\n", basename(argv[0]));
        platformExit(EXIT_FAILURE);
    }

    long nodeIdParam = strtol(argv[1], &endptr, 0);
    if (*endptr != '\0' || nodeIdParam < 1 || nodeIdParam >= UINT32_MAX ) {
        fprintf(stderr, "Invalid NodeId: %s (must be >= 1 and < UINT32_MAX )\n", argv[1]);
        platformExit(EXIT_FAILURE);
    }
    gNodeId = (uint32_t) nodeIdParam;

    platformLoggingInit(argv[0]);
    socket_init(argv[2]);
    platformAlarmInit(1);
    platformRadioInit();
    platformRandomInit();

    signal(SIGTERM, &handleSignal);
    signal(SIGHUP, &handleSignal);

    otSimSendNodeInfoEvent(gNodeId);
}

bool otSysPseudoResetWasRequested(void) {
    return gPlatformPseudoResetWasRequested;
}

void otSysDeinit(void) {
    close(gSockFd);
    gSockFd = 0;
}

void otSysProcessDrivers(otInstance *aInstance) {
    fd_set read_fds;
    fd_set write_fds;
    fd_set error_fds;
    int max_fd;
    int rval;

    if (gTerminate) {
        platformExit(EXIT_SUCCESS);
    }

    FD_ZERO(&read_fds);
    FD_ZERO(&write_fds);
    FD_ZERO(&error_fds);

    FD_SET(gSockFd, &read_fds);
    max_fd = gSockFd;

    if (!otTaskletsArePending(aInstance) && platformAlarmGetNext() > 0 &&
        (!platformRadioIsTransmitPending() || platformRadioIsBusy())) {
        // report my final radio state at end of this time instant, then go to sleep.
        platformRadioReportStateToSimulator();
        otSimSendSleepEvent();

        // wake up by reception of socket event from simulator.
        rval = select(max_fd + 1, &read_fds, &write_fds, &error_fds, NULL);

        if ((rval < 0) && (errno != EINTR)) {
            perror("select");
            platformExit(EXIT_FAILURE);
        }

        if (rval > 0 && FD_ISSET(gSockFd, &read_fds)) {
            platformReceiveEvent(aInstance);
        }
    }

    platformAlarmProcess(aInstance);
    platformRadioProcess(aInstance, &read_fds, &write_fds);
}

/**
* This function parses an environment variable as an unsigned 16-bit integer.
*
* If the environment variable does not exist, this function does nothing.
* If it is not a valid integer, this function will terminate the process with an error message.
*
* @param[in]   aEnvName  The name of the environment variable.
* @param[out]  aValue    A pointer to the unsigned 16-bit integer.
*
*/
static void parseFromEnvAsUint16(const char *aEnvName, uint16_t *aValue) {
    char *env = getenv(aEnvName);

    if (env) {
        char *endptr;

        *aValue = (uint16_t) strtol(env, &endptr, 0);

        if (*endptr != '\0') {
            fprintf(stderr, "Invalid %s: %s\n", aEnvName, env);
            platformExit(EXIT_FAILURE);
        }
    }
}

/**
 * This function initialises the client socket used for communication with the simulator.
 * The port number is calculated based on environment vars (if set) or else defaults.
 */
static void socket_init(char *socketFilePath) {
    struct sockaddr_un sockaddr;
    memset(&sockaddr, 0, sizeof(struct sockaddr_un));
    sockaddr.sun_family = AF_UNIX;
    int strLen = strlen(socketFilePath);
    if (strLen >= sizeof(sockaddr.sun_path)) {
        gTerminate = true;
        otPlatLog(OT_LOG_LEVEL_CRIT,OT_LOG_REGION_PLATFORM,
                  "Unix socket path too long: %s\n", socketFilePath);
        platformExit(EXIT_FAILURE);
    }
    memcpy(sockaddr.sun_path, socketFilePath, strLen);

    gSockFd = socket(AF_UNIX, SOCK_STREAM, 0);

    if (gSockFd == -1) {
        perror("socket");
        platformExit(EXIT_FAILURE);
    }

    if (connect(gSockFd, (struct sockaddr *) &sockaddr, sizeof(sockaddr)) == -1) {
        gTerminate = true;
        otPlatLog(OT_LOG_LEVEL_CRIT,OT_LOG_REGION_PLATFORM,
                  "Unable to open Unix socket to OT-NS at: %s\n", sockaddr.sun_path);
        perror("bind");
        platformExit(EXIT_FAILURE);
    }
}

static void handleSignal(int aSignal) {
    OT_UNUSED_VARIABLE(aSignal);

    gTerminate = true;
}
