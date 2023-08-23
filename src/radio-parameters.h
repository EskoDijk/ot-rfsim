/*
*  Copyright (c) 2023, The OpenThread Authors.
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

#ifndef OT_RFSIM_RADIO_PARAMETERS_H
#define OT_RFSIM_RADIO_PARAMETERS_H

// Custom parameters for the simulated RFSIM radio, which may vary over radio vendors while still
// being standards compliant. See radio.h for further parameters.
enum
{
    RFSIM_RECEIVE_SENSITIVITY_DBM      = -100,  // dBm
    RFSIM_CCA_ED_THRESHOLD_DEFAULT_DBM = -91,   // dBm, must be < 10 dB above rx sensitivity, default
    RFSIM_TX_POWER_DEFAULT_DBM         = 0,     // dBm, default (modifiable by CLI)
    RFSIM_CSL_ACCURACY_PPM             = 1,     // ppm
    RFSIM_CSL_UNCERTAINTY_10US         = 9,     // units of 10 us (ceiling of true uncertainty)
    RFSIM_TURNAROUND_TIME_US           = 40,    // radio turnaround time (us)
    RFSIM_STARTUP_TIME_US              = 140,   // Disabled -> Enabled time (us)
    RFSIM_RAMPUP_TIME_US               = 40,    // Sleeping -> Ready time (us)
};

#endif //OT_RFSIM_RADIO_PARAMETERS_H
