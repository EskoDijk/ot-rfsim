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

#ifndef PLATFORM_SIMULATION_RADIO_H
#define PLATFORM_SIMULATION_RADIO_H

#include <openthread/platform/radio.h>

#define FAILSAFE_TIME_US 4

enum
{
    SIM_RECEIVE_SENSITIVITY_DBM      = -100,                              // dBm
    SIM_CCA_ED_THRESHOLD_DEFAULT_DBM = SIM_RECEIVE_SENSITIVITY_DBM + 9,   // dBm, mandatory < 10 dB above rx sensitivity
    SIM_TX_POWER_DEFAULT_DBM         = 0,                                 // dBm
    SIM_CSL_ACCURACY_PPM             = 1,                                 // ppm
    SIM_CSL_UNCERTAINTY_10US         = 1,                                 // units of 10 us (floor of true uncertainty)
    kMinChannel                      = OT_RADIO_2P4GHZ_OQPSK_CHANNEL_MIN,
    kMaxChannel                      = OT_RADIO_2P4GHZ_OQPSK_CHANNEL_MAX,
    OT_RADIO_LIFS_TIME_US            = 40 * OT_RADIO_SYMBOL_TIME,
    OT_RADIO_SIFS_TIME_US            = 12 * OT_RADIO_SYMBOL_TIME,
    OT_RADIO_AIFS_TIME_US            = 12 * OT_RADIO_SYMBOL_TIME,
    OT_RADIO_CCA_TIME_US             = 8 * OT_RADIO_SYMBOL_TIME,
    OT_RADIO_TURNAROUND_TIME_US      = 40,                                // radio's turnaround: differs per radio model
    OT_RADIO_MAX_TURNAROUND_TIME_US  = 12 * OT_RADIO_SYMBOL_TIME,         // specified max turnaround time.
    OT_RADIO_MAX_ACK_WAIT_US         = (OT_RADIO_AIFS_TIME_US + (10 * OT_RADIO_SYMBOL_TIME)),
    OT_RADIO_aMaxSifsFrameSize       = 18,
    OT_RADIO_STARTUP_TIME_US         = OT_RADIO_LIFS_TIME_US,             // Disabled -> active. TODO check typical values.
};

OT_TOOL_PACKED_BEGIN
struct RadioMessage
{
    uint8_t mChannel;
    uint8_t mPsdu[OT_RADIO_FRAME_MAX_SIZE];
} OT_TOOL_PACKED_END;

/**
 * The sub-states of the virtual-time simulated radio while it is in Tx or Rx OT state.
 */
typedef enum
{
    OT_RADIO_SUBSTATE_READY,
    OT_RADIO_SUBSTATE_IFS_WAIT,
    OT_RADIO_SUBSTATE_TX_CCA,
    OT_RADIO_SUBSTATE_TX_CCA_TO_TX,
    OT_RADIO_SUBSTATE_TX_FRAME_ONGOING,
    OT_RADIO_SUBSTATE_TX_TX_TO_RX,
    OT_RADIO_SUBSTATE_TX_TX_TO_AIFS,
    OT_RADIO_SUBSTATE_TX_AIFS_WAIT,
    OT_RADIO_SUBSTATE_TX_ACK_RX_ONGOING,
    OT_RADIO_SUBSTATE_RX_FRAME_ONGOING,
    OT_RADIO_SUBSTATE_RX_AIFS_WAIT,
    OT_RADIO_SUBSTATE_RX_ACK_TX_ONGOING,
    OT_RADIO_SUBSTATE_RX_TX_TO_RX,
    OT_RADIO_SUBSTATE_RX_ENERGY_SCAN,
    OT_RADIO_SUBSTATE_STARTUP,
    OT_RADIO_SUBSTATE_INVALID,
} RadioSubState;

#endif // PLATFORM_SIMULATION_RADIO_H
