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

#include "platform-rfsim.h"
#include "radio.h"

#include <sys/time.h>
#include <stdio.h>

#include <openthread/link.h>
#include <openthread/platform/alarm-micro.h>
#include <openthread/platform/alarm-milli.h>
#include <openthread/platform/radio.h>
#include <openthread/platform/time.h>
#include <openthread/platform/otns.h>

#include "utils/code_utils.h"
#include "utils/mac_frame.h"
#include "utils/soft_source_match_table.h"
#include "event-sim.h"
#include "common/debug.hpp"

// declaration of radio functions
static void setRadioSubState(RadioSubState aState, uint64_t timeToRemainInState);
static void startCcaForTransmission(otInstance *aInstance);
static void signalRadioTxDone(otInstance *aInstance, otRadioFrame *aFrame, otRadioFrame *aAckFrame, otError aError);
void radioSendMessage(otInstance *aInstance);
void radioTransmit(struct RadioMessage *aMessage, const struct otRadioFrame *aFrame);
void setRadioState(otRadioState aState);
void radioPrepareAck(void);
bool IsTimeAfterOrEqual(uint32_t aTimeA, uint32_t aTimeB);
void radioProcessFrame(otInstance *aInstance, otError aError);

#if OPENTHREAD_CONFIG_THREAD_VERSION >= OT_THREAD_VERSION_1_2
static uint8_t generateAckIeData(uint8_t *aLinkMetricsIeData, uint8_t aLinkMetricsIeDataLen);
#endif

static otRadioState  sLastReportedState          = OT_RADIO_STATE_INVALID;
static RadioSubState sLastReportedSubState       = OT_RADIO_SUBSTATE_INVALID;
static uint8_t       sLastReportedChannel        = 0;
static uint64_t      sLastReportedRadioEventTime = 0;
static uint8_t       sOngoingOperationChannel    = kMinChannel;
static uint64_t      sNextRadioEventTime         = OT_RADIO_STARTUP_TIME_US;
static uint64_t      sReceiveTimestamp           = 0;
static RadioSubState sSubState                   = OT_RADIO_SUBSTATE_DISABLED;
static struct RadioCommEventData sLastTxEventData; // metadata about last/ongoing Tx action.

static int8_t   sEnergyScanResult  = OT_RADIO_RSSI_INVALID;
static bool     sEnergyScanning    = false;
static uint32_t sEnergyScanEndTime = 0;

static otRadioState        sState = OT_RADIO_STATE_DISABLED;
static struct RadioMessage sReceiveMessage;
static struct RadioMessage sTransmitMessage;
static struct RadioMessage sAckMessage;
static otRadioFrame        sReceiveFrame;
static otRadioFrame        sTransmitFrame;
static otRadioFrame        sAckFrame;

#if OPENTHREAD_CONFIG_MAC_HEADER_IE_SUPPORT
static otRadioIeInfo sTransmitIeInfo;
#endif

static otExtAddress   sExtAddress;
static otShortAddress sShortAddress;
static otPanId        sPanid;
static bool           sPromiscuous = false;
static int8_t         sTxPower     = SIM_TX_POWER_DEFAULT_DBM;
static int8_t         sCcaEdThresh = SIM_CCA_ED_THRESHOLD_DEFAULT_DBM;
static int8_t         sLnaGain     = 0;
static uint16_t       sRegionCode  = 0;
static int8_t         sChannelMaxTransmitPower[kMaxChannel - kMinChannel + 1];
static uint8_t        sCurrentChannel = kMinChannel;
static bool           sSrcMatchEnabled = false;

#if OPENTHREAD_CONFIG_THREAD_VERSION >= OT_THREAD_VERSION_1_2
static uint8_t sAckIeData[OT_ACK_IE_MAX_SIZE];
static uint8_t sAckIeDataLength = 0;
#endif

#if OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE
static uint32_t sCslSampleTime;
static uint32_t sCslPeriod;
#endif

#if OPENTHREAD_CONFIG_PLATFORM_RADIO_COEX_ENABLE
static bool sRadioCoexEnabled = true;
#endif

otRadioCaps gRadioCaps =
#if OPENTHREAD_CONFIG_THREAD_VERSION >= OT_THREAD_VERSION_1_2
        OT_RADIO_CAPS_TRANSMIT_SEC;
#else
OT_RADIO_CAPS_NONE;
#endif

static uint32_t         sMacFrameCounter;
static uint8_t          sKeyId;
static otMacKeyMaterial sPrevKey;
static otMacKeyMaterial sCurrKey;
static otMacKeyMaterial sNextKey;
static otRadioKeyType   sKeyType;

bool IsTimeAfterOrEqual(uint32_t aTimeA, uint32_t aTimeB)
{
    return (aTimeA - aTimeB) < (1U << 31);
}

static void ReverseExtAddress(otExtAddress *aReversed, const otExtAddress *aOrigin)
{
    for (size_t i = 0; i < sizeof(*aReversed); i++)
    {
        aReversed->m8[i] = aOrigin->m8[sizeof(*aOrigin) - 1 - i];
    }
}

static bool hasFramePending(const otRadioFrame *aFrame)
{
    bool         rval = false;
    otMacAddress src;

    otEXPECT_ACTION(sSrcMatchEnabled, rval = true);
    otEXPECT(otMacFrameGetSrcAddr(aFrame, &src) == OT_ERROR_NONE);

    switch (src.mType)
    {
        case OT_MAC_ADDRESS_TYPE_SHORT:
            rval = utilsSoftSrcMatchShortFindEntry(src.mAddress.mShortAddress) >= 0;
            break;
        case OT_MAC_ADDRESS_TYPE_EXTENDED:
        {
            otExtAddress extAddr;

            ReverseExtAddress(&extAddr, &src.mAddress.mExtAddress);
            rval = utilsSoftSrcMatchExtFindEntry(&extAddr) >= 0;
            break;
        }
        default:
            break;
    }

    exit:
    return rval;
}

static uint16_t crc16_citt(uint16_t aFcs, uint8_t aByte)
{
    // CRC-16/CCITT, CRC-16/CCITT-TRUE, CRC-CCITT
    // width=16 poly=0x1021 init=0x0000 refin=true refout=true xorout=0x0000 check=0x2189 name="KERMIT"
    // http://reveng.sourceforge.net/crc-catalogue/16.htm#crc.cat.kermit
    static const uint16_t sFcsTable[256] = {
            0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5,
            0xe97e, 0xf8f7, 0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e, 0x9cc9, 0x8d40, 0xbfdb, 0xae52,
            0xdaed, 0xcb64, 0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3,
            0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5, 0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
            0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9,
            0x2732, 0x36bb, 0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285, 0x430c, 0x7197, 0x601e,
            0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72, 0x6306, 0x728f,
            0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
            0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738, 0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862,
            0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb,
            0x4e64, 0x5fed, 0x6d76, 0x7cff, 0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948,
            0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
            0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226,
            0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c, 0xc60c, 0xd785, 0xe51e, 0xf497,
            0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb, 0xd68d, 0xc704,
            0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232, 0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
            0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb,
            0x0e70, 0x1ff9, 0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c,
            0x3de3, 0x2c6a, 0x1ef1, 0x0f78};
    return (aFcs >> 8) ^ sFcsTable[(aFcs ^ aByte) & 0xff];
}

void otPlatRadioGetIeeeEui64(otInstance *aInstance, uint8_t *aIeeeEui64)
{
    OT_UNUSED_VARIABLE(aInstance);

    assert(aInstance != NULL);

    aIeeeEui64[0] = 0x18;
    aIeeeEui64[1] = 0xb4;
    aIeeeEui64[2] = 0x30;
    aIeeeEui64[3] = 0x00;
    aIeeeEui64[4] = (gNodeId >> 24) & 0xff;
    aIeeeEui64[5] = (gNodeId >> 16) & 0xff;
    aIeeeEui64[6] = (gNodeId >> 8) & 0xff;
    aIeeeEui64[7] = gNodeId & 0xff;
}

void otPlatRadioSetPanId(otInstance *aInstance, otPanId aPanid)
{
    OT_UNUSED_VARIABLE(aInstance);

    assert(aInstance != NULL);

    sPanid = aPanid;
    utilsSoftSrcMatchSetPanId(aPanid);
}

void otPlatRadioSetExtendedAddress(otInstance *aInstance, const otExtAddress *aExtAddress)
{
    OT_UNUSED_VARIABLE(aInstance);

    assert(aInstance != NULL);

    ReverseExtAddress(&sExtAddress, aExtAddress);

    otSimSendExtAddrEvent(&sExtAddress);
}

void otPlatRadioSetShortAddress(otInstance *aInstance, otShortAddress aShortAddress)
{
    OT_UNUSED_VARIABLE(aInstance);

    assert(aInstance != NULL);

    sShortAddress = aShortAddress;
}

void otPlatRadioSetPromiscuous(otInstance *aInstance, bool aEnable)
{
    OT_UNUSED_VARIABLE(aInstance);

    assert(aInstance != NULL);

    sPromiscuous = aEnable;
}

void platformRadioInit(void)
{
    sReceiveFrame.mPsdu  = sReceiveMessage.mPsdu;
    sTransmitFrame.mPsdu = sTransmitMessage.mPsdu;
    sAckFrame.mPsdu      = sAckMessage.mPsdu;

#if OPENTHREAD_CONFIG_MAC_HEADER_IE_SUPPORT
    sTransmitFrame.mInfo.mTxInfo.mIeInfo = &sTransmitIeInfo;
#else
    sTransmitFrame.mInfo.mTxInfo.mIeInfo = NULL;
#endif

    for (size_t i = 0; i <= kMaxChannel - kMinChannel; i++)
    {
        sChannelMaxTransmitPower[i] = OT_RADIO_POWER_INVALID;
    }
    sReceiveFrame.mInfo.mRxInfo.mRssi = OT_RADIO_RSSI_INVALID;

#if OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE
    otLinkMetricsInit(SIM_RECEIVE_SENSITIVITY_DBM);
#endif
}

#if OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE
static uint16_t getCslPhase(void)
{
    // The CSL-Phase-Time is the time between 1) start of MHR of current frame to be sent,
    // and 2) start of MHR of next frame that will be CSL-received (i.e. sampled);
    // - equal to the time between 1) start of preamble of current frame to be sent,
    // and 2) start of preamble reception of next frame that will be CSL-received;
    // - equal to the time between 1) end of last symbol of SFD of current frame to be sent,
    // and 2) end of last symbol of SFD of next frame that will be CSL-received.
    //
    // Calculation assumes Tx frame will be sent 'now' i.e. start of first symbol of preamble is now.
    // That is valid because `getCslPhase()` will be called directly before `radioTransmit()`, so in the
    // same simulated time instant.
    uint32_t txSfdEndTime = otPlatAlarmMicroGetNow();
    uint32_t cslPeriodInUs = sCslPeriod * OT_US_PER_TEN_SYMBOLS;
    uint32_t diff = ((sCslSampleTime % cslPeriodInUs) - (txSfdEndTime % cslPeriodInUs) + cslPeriodInUs) % cslPeriodInUs;

    // phase integer needs to be 'rounded up' in fractional cases. Otherwise, CSL receiver
    // might miss the first part of transmission.
    if ( diff % OT_US_PER_TEN_SYMBOLS > 0)
        diff += OT_US_PER_TEN_SYMBOLS;
    return (uint16_t)( diff / OT_US_PER_TEN_SYMBOLS);
}
#endif

bool otPlatRadioIsEnabled(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    return (sState != OT_RADIO_STATE_DISABLED) ? true : false;
}

otError otPlatRadioEnable(otInstance *aInstance)
{
    if (!otPlatRadioIsEnabled(aInstance))
    {
        setRadioState(OT_RADIO_STATE_SLEEP);
    }

    return OT_ERROR_NONE;
}

otError otPlatRadioDisable(otInstance *aInstance)
{
    otError error = OT_ERROR_NONE;

    otEXPECT(otPlatRadioIsEnabled(aInstance));
    otEXPECT_ACTION(sState == OT_RADIO_STATE_SLEEP, error = OT_ERROR_INVALID_STATE);

    setRadioState(OT_RADIO_STATE_DISABLED);

    exit:
    return error;
}

otError otPlatRadioSleep(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    assert(aInstance != NULL);

    otError error = OT_ERROR_INVALID_STATE;

    if (sState == OT_RADIO_STATE_SLEEP || sState == OT_RADIO_STATE_RECEIVE)
    {
        error  = OT_ERROR_NONE;
        setRadioState(OT_RADIO_STATE_SLEEP);
    }

    return error;
}

otError otPlatRadioReceive(otInstance *aInstance, uint8_t aChannel)
{
    OT_UNUSED_VARIABLE(aInstance);

    assert(aInstance != NULL);

    otError error = OT_ERROR_INVALID_STATE;

    if (sState != OT_RADIO_STATE_DISABLED)
    {
        error                  = OT_ERROR_NONE;
        sReceiveFrame.mChannel = aChannel;
        sCurrentChannel        = aChannel;
        setRadioState(OT_RADIO_STATE_RECEIVE);
    }

    return error;
}

otError otPlatRadioTransmit(otInstance *aInstance, otRadioFrame *aFrame)
{
    OT_UNUSED_VARIABLE(aInstance);

    assert(aInstance != NULL);
    assert(aFrame != NULL);

    otError error = OT_ERROR_INVALID_STATE;

    if (sState == OT_RADIO_STATE_RECEIVE)
    {
        error           = OT_ERROR_NONE;
        sCurrentChannel = aFrame->mChannel;
        setRadioState(OT_RADIO_STATE_TRANSMIT);
    }

    return error;
}

otRadioFrame *otPlatRadioGetTransmitBuffer(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    return &sTransmitFrame;
}

int8_t otPlatRadioGetRssi(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    // return the most recent RSSI measurement - which is (currently) the one from the received frame.
    // (regardless of which channel it was received on)
    return sReceiveFrame.mInfo.mRxInfo.mRssi;
}

otRadioCaps otPlatRadioGetCaps(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    assert(aInstance != NULL);

    return gRadioCaps;
}

bool otPlatRadioGetPromiscuous(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    assert(aInstance != NULL);

    return sPromiscuous;
}

static void radioComputeCrc(struct RadioMessage *aMessage, uint16_t aLength)
{
    uint16_t crc        = 0;
    uint16_t crc_offset = aLength - sizeof(uint16_t);

    for (uint16_t i = 0; i < crc_offset; i++)
    {
        crc = crc16_citt(crc, aMessage->mPsdu[i]);
    }

    aMessage->mPsdu[crc_offset]     = crc & 0xff;
    aMessage->mPsdu[crc_offset + 1] = crc >> 8;
}

static otError radioProcessTransmitSecurity(otRadioFrame *aFrame)
{
    otError error = OT_ERROR_NONE;
#if OPENTHREAD_CONFIG_THREAD_VERSION >= OT_THREAD_VERSION_1_2
    otMacKeyMaterial *key = NULL;
    uint8_t           keyId;

    otEXPECT(otMacFrameIsSecurityEnabled(aFrame) && otMacFrameIsKeyIdMode1(aFrame) &&
             !aFrame->mInfo.mTxInfo.mIsSecurityProcessed);

    if (otMacFrameIsAck(aFrame))
    {
        keyId = otMacFrameGetKeyId(aFrame);

        otEXPECT_ACTION(keyId != 0, error = OT_ERROR_FAILED);

        if (keyId == sKeyId)
        {
            key = &sCurrKey;
        }
        else if (keyId == sKeyId - 1)
        {
            key = &sPrevKey;
        }
        else if (keyId == sKeyId + 1)
        {
            key = &sNextKey;
        }
        else
        {
            error = OT_ERROR_SECURITY;
            otEXPECT(false);
        }
    }
    else
    {
        key   = &sCurrKey;
        keyId = sKeyId;
    }

    aFrame->mInfo.mTxInfo.mAesKey = key;

    if (!aFrame->mInfo.mTxInfo.mIsHeaderUpdated)
    {
        otMacFrameSetKeyId(aFrame, keyId);
        otMacFrameSetFrameCounter(aFrame, sMacFrameCounter++);
    }
#else
    otEXPECT(!aFrame->mInfo.mTxInfo.mIsSecurityProcessed);
#endif // OPENTHREAD_CONFIG_THREAD_VERSION >= OT_THREAD_VERSION_1_2

    otMacFrameProcessTransmitAesCcm(aFrame, &sExtAddress);

    exit:
    return error;
}

void radioSendMessage(otInstance *aInstance)
{
#if OPENTHREAD_CONFIG_MAC_HEADER_IE_SUPPORT && OPENTHREAD_CONFIG_TIME_SYNC_ENABLE
    if (sTransmitFrame.mInfo.mTxInfo.mIeInfo->mTimeIeOffset != 0)
    {
        uint8_t *timeIe = sTransmitFrame.mPsdu + sTransmitFrame.mInfo.mTxInfo.mIeInfo->mTimeIeOffset;
        uint64_t time = (uint64_t)((int64_t)otPlatTimeGet() + sTransmitFrame.mInfo.mTxInfo.mIeInfo->mNetworkTimeOffset);

        *timeIe = sTransmitFrame.mInfo.mTxInfo.mIeInfo->mTimeSyncSeq;

        *(++timeIe) = (uint8_t)(time & 0xff);
        for (uint8_t i = 1; i < sizeof(uint64_t); i++)
        {
            time        = time >> 8;
            *(++timeIe) = (uint8_t)(time & 0xff);
        }
    }
#endif // OPENTHREAD_CONFIG_MAC_HEADER_IE_SUPPORT && OPENTHREAD_CONFIG_TIME_SYNC_ENABLE

#if OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE
    if (sCslPeriod > 0 && !sTransmitFrame.mInfo.mTxInfo.mIsHeaderUpdated)
    {
        otMacFrameSetCslIe(&sTransmitFrame, (uint16_t)sCslPeriod, getCslPhase());
    }
#endif

    sTransmitMessage.mChannel = sTransmitFrame.mChannel;

    otEXPECT(radioProcessTransmitSecurity(&sTransmitFrame) == OT_ERROR_NONE);
    otPlatRadioTxStarted(aInstance, &sTransmitFrame);
    radioComputeCrc(&sTransmitMessage, sTransmitFrame.mLength);
    radioTransmit(&sTransmitMessage, &sTransmitFrame);

    exit:
    return;
}

void radioPrepareAck(void)
{
    if (
#if OPENTHREAD_CONFIG_THREAD_VERSION >= OT_THREAD_VERSION_1_2
// Determine if frame pending bit should be set
((otMacFrameIsVersion2015(&sReceiveFrame) && otMacFrameIsCommand(&sReceiveFrame)) ||
 otMacFrameIsData(&sReceiveFrame) || otMacFrameIsDataRequest(&sReceiveFrame))
#else
otMacFrameIsDataRequest(&sReceiveFrame)
#endif
&& hasFramePending(&sReceiveFrame))
    {
        sReceiveFrame.mInfo.mRxInfo.mAckedWithFramePending = true;
    }

#if OPENTHREAD_CONFIG_THREAD_VERSION >= OT_THREAD_VERSION_1_2
    // Use enh-ack for 802.15.4-2015 frames
    if (otMacFrameIsVersion2015(&sReceiveFrame))
    {
        uint8_t  linkMetricsDataLen = 0;
        uint8_t *dataPtr            = NULL;

#if OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE
        uint8_t      linkMetricsData[OT_ENH_PROBING_IE_DATA_MAX_SIZE];
        otMacAddress macAddress;

        otEXPECT(otMacFrameGetSrcAddr(&sReceiveFrame, &macAddress) == OT_ERROR_NONE);

        linkMetricsDataLen = otLinkMetricsEnhAckGenData(&macAddress, sReceiveFrame.mInfo.mRxInfo.mLqi,
                                                        sReceiveFrame.mInfo.mRxInfo.mRssi, linkMetricsData);

        if (linkMetricsDataLen > 0)
        {
            dataPtr = linkMetricsData;
        }
#endif

        sAckIeDataLength = generateAckIeData(dataPtr, linkMetricsDataLen);

        otEXPECT(otMacFrameGenerateEnhAck(&sReceiveFrame, sReceiveFrame.mInfo.mRxInfo.mAckedWithFramePending,
                                          sAckIeData, sAckIeDataLength, &sAckFrame) == OT_ERROR_NONE);
#if OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE
        if (sCslPeriod > 0)
        {
            otMacFrameSetCslIe(&sAckFrame, (uint16_t)sCslPeriod, getCslPhase());
        }
#endif
        if (otMacFrameIsSecurityEnabled(&sAckFrame))
        {
            otEXPECT(radioProcessTransmitSecurity(&sAckFrame) == OT_ERROR_NONE);
        }
    }
    else
#endif
    {
        otMacFrameGenerateImmAck(&sReceiveFrame, sReceiveFrame.mInfo.mRxInfo.mAckedWithFramePending, &sAckFrame);
    }

    sAckMessage.mChannel = sReceiveFrame.mChannel;
    radioComputeCrc(&sAckMessage, sAckFrame.mLength);

#if OPENTHREAD_CONFIG_THREAD_VERSION >= OT_THREAD_VERSION_1_2
    exit:
#endif
    return;
}

#if OPENTHREAD_CONFIG_THREAD_VERSION >= OT_THREAD_VERSION_1_2
static uint8_t generateAckIeData(uint8_t *aLinkMetricsIeData, uint8_t aLinkMetricsIeDataLen)
{
    OT_UNUSED_VARIABLE(aLinkMetricsIeData);
    OT_UNUSED_VARIABLE(aLinkMetricsIeDataLen);

    uint8_t offset = 0;

#if OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE
    if (sCslPeriod > 0)
    {
        offset += otMacFrameGenerateCslIeTemplate(sAckIeData);
    }
#endif

#if OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE
    if (aLinkMetricsIeData != NULL && aLinkMetricsIeDataLen > 0)
    {
        offset += otMacFrameGenerateEnhAckProbingIe(sAckIeData, aLinkMetricsIeData, aLinkMetricsIeDataLen);
    }
#endif

    return offset;
}
#endif

void radioProcessFrame(otInstance *aInstance, otError aError)
{
    otError      error = aError;
    otMacAddress macAddress;
    OT_UNUSED_VARIABLE(macAddress);

    // sReceiveFrame RSSI and LQI are set in platformRadioReceive()
    sReceiveFrame.mInfo.mRxInfo.mAckedWithFramePending = false;
    sReceiveFrame.mInfo.mRxInfo.mAckedWithSecEnhAck    = false;

    otEXPECT(sPromiscuous == false); // Ack never sent in promiscuous mode https://github.com/openthread/openthread/issues/4161

    otEXPECT_ACTION(otMacFrameDoesAddrMatch(&sReceiveFrame, sPanid, sShortAddress, &sExtAddress),
                    error = OT_ERROR_ABORT);

#if OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE
    otEXPECT_ACTION(otMacFrameGetSrcAddr(&sReceiveFrame, &macAddress) == OT_ERROR_NONE, error = OT_ERROR_PARSE);
#endif

    // generate acknowledgment
    if (otMacFrameIsAckRequested(&sReceiveFrame) && error == OT_ERROR_NONE)
    {
        radioPrepareAck();
#if OPENTHREAD_CONFIG_THREAD_VERSION >= OT_THREAD_VERSION_1_2
        if (otMacFrameIsSecurityEnabled(&sAckFrame))
        {
            sReceiveFrame.mInfo.mRxInfo.mAckedWithSecEnhAck = true;
            sReceiveFrame.mInfo.mRxInfo.mAckFrameCounter    = otMacFrameGetFrameCounter(&sAckFrame);
        }
#endif // OPENTHREAD_CONFIG_THREAD_VERSION >= OT_THREAD_VERSION_1_2
    }

    exit:

    // If Rx-frame was received and it is for me, call receive-done handler.
    if (error != OT_ERROR_ABORT)
    {
#if OPENTHREAD_CONFIG_DIAG_ENABLE
        if (otPlatDiagModeGet())
        {
            otPlatDiagRadioReceiveDone(aInstance, error == OT_ERROR_NONE ? &sReceiveFrame : NULL, error);
        }
        else
#endif
        {
            otPlatRadioReceiveDone(aInstance, error == OT_ERROR_NONE ? &sReceiveFrame : NULL, error);
        }
    }
}

bool platformRadioIsTransmitPending(void)
{
    return sState == OT_RADIO_STATE_TRANSMIT && !platformRadioIsTransmitOngoing();
}

bool platformRadioIsTransmitOngoing(void)
{
    return  sSubState == OT_RADIO_SUBSTATE_TX_CCA ||
            sSubState == OT_RADIO_SUBSTATE_TX_CCA_TO_TX ||
            sSubState == OT_RADIO_SUBSTATE_TX_FRAME_ONGOING ||
            sSubState == OT_RADIO_SUBSTATE_TX_TX_TO_RX ||
            sSubState == OT_RADIO_SUBSTATE_TX_TX_TO_AIFS ||
            sSubState == OT_RADIO_SUBSTATE_TX_AIFS_WAIT ||
            sSubState == OT_RADIO_SUBSTATE_TX_ACK_RX_ONGOING;
}

bool platformRadioIsReceiveOngoing(void)
{
    return  sSubState == OT_RADIO_SUBSTATE_RX_FRAME_ONGOING ||
            sSubState == OT_RADIO_SUBSTATE_RX_AIFS_WAIT ||
            sSubState == OT_RADIO_SUBSTATE_RX_ACK_TX_ONGOING ||
            sSubState == OT_RADIO_SUBSTATE_RX_TX_TO_RX ||
            sSubState == OT_RADIO_SUBSTATE_RX_ENERGY_SCAN;
}

void otPlatRadioEnableSrcMatch(otInstance *aInstance, bool aEnable)
{
    OT_UNUSED_VARIABLE(aInstance);

    assert(aInstance != NULL);

    sSrcMatchEnabled = aEnable;
}

otError otPlatRadioEnergyScan(otInstance *aInstance, uint8_t aScanChannel, uint16_t aScanDuration)
{
    OT_UNUSED_VARIABLE(aInstance);

    otError error = OT_ERROR_NONE;

    assert(aInstance != NULL);
    OT_ASSERT(aScanChannel >= kMinChannel && aScanChannel <= kMaxChannel);
    OT_ASSERT(aScanDuration > 0);

    otEXPECT_ACTION((gRadioCaps & OT_RADIO_CAPS_ENERGY_SCAN), error = OT_ERROR_NOT_IMPLEMENTED);
    otEXPECT_ACTION(!sEnergyScanning, error = OT_ERROR_BUSY);

    // TODO: need to get energy scan Rssi from simulator at end of period.
    sEnergyScanResult  = OT_RADIO_RSSI_INVALID;
    sEnergyScanning    = true;
    sEnergyScanEndTime = otPlatAlarmMilliGetNow() + aScanDuration;

    exit:
    return error;
}

otError otPlatRadioGetTransmitPower(otInstance *aInstance, int8_t *aPower)
{
    OT_UNUSED_VARIABLE(aInstance);

    assert(aInstance != NULL);

    int8_t maxPower = sChannelMaxTransmitPower[sCurrentChannel - kMinChannel];
    *aPower = sTxPower < maxPower ? sTxPower : maxPower;

    return OT_ERROR_NONE;
}

otError otPlatRadioSetTransmitPower(otInstance *aInstance, int8_t aPower)
{
    OT_UNUSED_VARIABLE(aInstance);

    assert(aInstance != NULL);

    sTxPower = aPower;

    return OT_ERROR_NONE;
}

otError otPlatRadioGetCcaEnergyDetectThreshold(otInstance *aInstance, int8_t *aThreshold)
{
    OT_UNUSED_VARIABLE(aInstance);

    assert(aInstance != NULL);

    *aThreshold = sCcaEdThresh;

    return OT_ERROR_NONE;
}

otError otPlatRadioSetCcaEnergyDetectThreshold(otInstance *aInstance, int8_t aThreshold)
{
    OT_UNUSED_VARIABLE(aInstance);

    assert(aInstance != NULL);

    sCcaEdThresh = aThreshold;

    return OT_ERROR_NONE;
}

otError otPlatRadioGetFemLnaGain(otInstance *aInstance, int8_t *aGain)
{
    OT_UNUSED_VARIABLE(aInstance);

    OT_ASSERT(aInstance != NULL && aGain != NULL);

    *aGain = sLnaGain;

    return OT_ERROR_NONE;
}

otError otPlatRadioSetFemLnaGain(otInstance *aInstance, int8_t aGain)
{
    OT_UNUSED_VARIABLE(aInstance);

    assert(aInstance != NULL);

    sLnaGain = aGain;

    return OT_ERROR_NONE;
}

int8_t otPlatRadioGetReceiveSensitivity(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    assert(aInstance != NULL);

    return SIM_RECEIVE_SENSITIVITY_DBM;
}

otRadioState otPlatRadioGetState(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    return sState;
}

#if OPENTHREAD_CONFIG_PLATFORM_RADIO_COEX_ENABLE
otError otPlatRadioSetCoexEnabled(otInstance *aInstance, bool aEnabled)
{
    OT_UNUSED_VARIABLE(aInstance);

    assert(aInstance != NULL);

    sRadioCoexEnabled = aEnabled;
    return OT_ERROR_NONE;
}

bool otPlatRadioIsCoexEnabled(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    assert(aInstance != NULL);

    return sRadioCoexEnabled;
}

otError otPlatRadioGetCoexMetrics(otInstance *aInstance, otRadioCoexMetrics *aCoexMetrics)
{
    OT_UNUSED_VARIABLE(aInstance);

    otError error = OT_ERROR_NONE;

    assert(aInstance != NULL);
    otEXPECT_ACTION(aCoexMetrics != NULL, error = OT_ERROR_INVALID_ARGS);

    memset(aCoexMetrics, 0, sizeof(otRadioCoexMetrics));

    aCoexMetrics->mStopped                            = false;
    aCoexMetrics->mNumGrantGlitch                     = 1;
    aCoexMetrics->mNumTxRequest                       = 2;
    aCoexMetrics->mNumTxGrantImmediate                = 3;
    aCoexMetrics->mNumTxGrantWait                     = 4;
    aCoexMetrics->mNumTxGrantWaitActivated            = 5;
    aCoexMetrics->mNumTxGrantWaitTimeout              = 6;
    aCoexMetrics->mNumTxGrantDeactivatedDuringRequest = 7;
    aCoexMetrics->mNumTxDelayedGrant                  = 8;
    aCoexMetrics->mAvgTxRequestToGrantTime            = 9;
    aCoexMetrics->mNumRxRequest                       = 10;
    aCoexMetrics->mNumRxGrantImmediate                = 11;
    aCoexMetrics->mNumRxGrantWait                     = 12;
    aCoexMetrics->mNumRxGrantWaitActivated            = 13;
    aCoexMetrics->mNumRxGrantWaitTimeout              = 14;
    aCoexMetrics->mNumRxGrantDeactivatedDuringRequest = 15;
    aCoexMetrics->mNumRxDelayedGrant                  = 16;
    aCoexMetrics->mAvgRxRequestToGrantTime            = 17;
    aCoexMetrics->mNumRxGrantNone                     = 18;

    exit:
    return error;
}
#endif // OPENTHREAD_CONFIG_PLATFORM_RADIO_COEX_ENABLE

uint64_t otPlatRadioGetNow(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    return otPlatTimeGet();
}

#if OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE
otError otPlatRadioEnableCsl(otInstance *        aInstance,
                             uint32_t            aCslPeriod,
                             otShortAddress      aShortAddr,
                             const otExtAddress *aExtAddr)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aShortAddr);
    OT_UNUSED_VARIABLE(aExtAddr);

    otError error = OT_ERROR_NONE;

    sCslPeriod = aCslPeriod;

    return error;
}

void otPlatRadioUpdateCslSampleTime(otInstance *aInstance, uint32_t aCslSampleTime)
{
    OT_UNUSED_VARIABLE(aInstance);

    sCslSampleTime = aCslSampleTime;
}
#endif // OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE

uint8_t otPlatRadioGetCslAccuracy(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    return SIM_CSL_ACCURACY_PPM;
}

#if OPENTHREAD_CONFIG_MAC_CSL_TRANSMITTER_ENABLE
uint8_t otPlatRadioGetCslUncertainty(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    return SIM_CSL_UNCERTAINTY_10US;
}
#endif // OPENTHREAD_CONFIG_MAC_CSL_TRANSMITTER_ENABLE

void otPlatRadioSetMacKey(otInstance *            aInstance,
                          uint8_t                 aKeyIdMode,
                          uint8_t                 aKeyId,
                          const otMacKeyMaterial *aPrevKey,
                          const otMacKeyMaterial *aCurrKey,
                          const otMacKeyMaterial *aNextKey,
                          otRadioKeyType          aKeyType)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aKeyIdMode);

    otEXPECT(aPrevKey != NULL && aCurrKey != NULL && aNextKey != NULL);

    sKeyId   = aKeyId;
    sKeyType = aKeyType;
    memcpy(&sPrevKey, aPrevKey, sizeof(otMacKeyMaterial));
    memcpy(&sCurrKey, aCurrKey, sizeof(otMacKeyMaterial));
    memcpy(&sNextKey, aNextKey, sizeof(otMacKeyMaterial));

    exit:
    return;
}

void otPlatRadioSetMacFrameCounter(otInstance *aInstance, uint32_t aMacFrameCounter)
{
    OT_UNUSED_VARIABLE(aInstance);

    sMacFrameCounter = aMacFrameCounter;
}

otError otPlatRadioSetChannelMaxTransmitPower(otInstance *aInstance, uint8_t aChannel, int8_t aMaxPower)
{
    OT_UNUSED_VARIABLE(aInstance);

    otError error = OT_ERROR_NONE;

    otEXPECT_ACTION(aChannel >= kMinChannel && aChannel <= kMaxChannel, error = OT_ERROR_INVALID_ARGS);
    sChannelMaxTransmitPower[aChannel - kMinChannel] = aMaxPower;

    exit:
    return error;
}

#if OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE
otError otPlatRadioConfigureEnhAckProbing(otInstance *         aInstance,
                                          otLinkMetrics        aLinkMetrics,
                                          const otShortAddress aShortAddress,
                                          const otExtAddress * aExtAddress)
{
    OT_UNUSED_VARIABLE(aInstance);

    return otLinkMetricsConfigureEnhAckProbing(aShortAddress, aExtAddress, aLinkMetrics);
}
#endif

otError otPlatRadioSetRegion(otInstance *aInstance, uint16_t aRegionCode)
{
    OT_UNUSED_VARIABLE(aInstance);

    sRegionCode = aRegionCode;
    return OT_ERROR_NONE;
}

otError otPlatRadioGetRegion(otInstance *aInstance, uint16_t *aRegionCode)
{
    OT_UNUSED_VARIABLE(aInstance);
    otError error = OT_ERROR_NONE;

    otEXPECT_ACTION(aRegionCode != NULL, error = OT_ERROR_INVALID_ARGS);

    *aRegionCode = sRegionCode;
    exit:
    return error;
}


void radioTransmit(struct RadioMessage *aMessage, const struct otRadioFrame *aFrame)
{
    // ( 4B preamble + 1B SFD + 1B PHY header + MAC frame ) @250kbps
    uint64_t frameDurationUs = (6 + aFrame->mLength) * OT_RADIO_SYMBOLS_PER_OCTET * OT_RADIO_SYMBOL_TIME;

    int8_t maxPower            = sChannelMaxTransmitPower[aFrame->mChannel - kMinChannel];
    sLastTxEventData.mChannel  = aFrame->mChannel;
    sLastTxEventData.mPower    = sTxPower < maxPower ? sTxPower : maxPower;
    sLastTxEventData.mError    = OT_ERROR_NONE;
    sLastTxEventData.mDuration = frameDurationUs;

    otSimSendRadioCommEvent(&sLastTxEventData, (const uint8_t*) aMessage, aFrame->mLength + offsetof(struct RadioMessage, mPsdu));
}

void radioReceive(otInstance *aInstance, otError aError)
{
    otEXPECT(sState == OT_RADIO_STATE_RECEIVE || sState == OT_RADIO_STATE_TRANSMIT);

    bool isAck = otMacFrameIsAck(&sReceiveFrame);
    sReceiveFrame.mInfo.mRxInfo.mTimestamp = sReceiveTimestamp;

    if (platformRadioIsTransmitOngoing() && otMacFrameIsAckRequested(&sTransmitFrame))
    {
        otError txDoneError = OT_ERROR_NONE;
        // TODO: for Enh-Ack, look at address match too.
        bool isAwaitedAckReceived = isAck && aError == OT_ERROR_NONE &&
                                    otMacFrameGetSequence(&sReceiveFrame) == otMacFrameGetSequence(&sTransmitFrame);
        if (!isAwaitedAckReceived)
        {
            txDoneError = OT_ERROR_NO_ACK;
        }
        signalRadioTxDone(aInstance,&sTransmitFrame, (isAck ? &sReceiveFrame : NULL), txDoneError);
    }
    else if (!isAck || sPromiscuous)
    {
        radioProcessFrame(aInstance, aError);
    }

    exit:
    return;
}

// helper function to invoke otPlatRadioTxDone() and its diagnostic equivalent.
static void signalRadioTxDone(otInstance *aInstance, otRadioFrame *aFrame, otRadioFrame *aAckFrame, otError aError) {
    setRadioState(OT_RADIO_STATE_RECEIVE); // set per state diagram in radio.hpp
#if OPENTHREAD_CONFIG_DIAG_ENABLE
    if (otPlatDiagModeGet())
    {
        otPlatDiagRadioTransmitDone(aInstance, aFrame, aError);
    }
    else
#endif
    {
        otPlatRadioTxDone(aInstance, aFrame, aAckFrame, aError);
    }
}

void platformRadioReportStateToSimulator()
{
    struct RadioStateEventData stateReport;

    if (sLastReportedState != sState || sLastReportedChannel != sOngoingOperationChannel ||
        sLastReportedSubState != sSubState || sLastReportedRadioEventTime != sNextRadioEventTime)
    {
        sLastReportedState    = sState;
        sLastReportedChannel  = sOngoingOperationChannel;
        sLastReportedSubState = sSubState;
        sLastReportedRadioEventTime = sNextRadioEventTime;

        // determine the energy-state from subState. TODO: distinguish more (detailed) energy levels.
        uint8_t energyState = OT_RADIO_STATE_INVALID;
        switch(sSubState) {
            case OT_RADIO_SUBSTATE_READY:
            case OT_RADIO_SUBSTATE_IFS_WAIT:
            case OT_RADIO_SUBSTATE_TX_CCA:
            case OT_RADIO_SUBSTATE_TX_CCA_TO_TX:
            case OT_RADIO_SUBSTATE_TX_TX_TO_RX:
            case OT_RADIO_SUBSTATE_TX_TX_TO_AIFS:
            case OT_RADIO_SUBSTATE_TX_AIFS_WAIT:
            case OT_RADIO_SUBSTATE_TX_ACK_RX_ONGOING:
            case OT_RADIO_SUBSTATE_RX_FRAME_ONGOING:
            case OT_RADIO_SUBSTATE_RX_AIFS_WAIT:
            case OT_RADIO_SUBSTATE_RX_TX_TO_RX:
            case OT_RADIO_SUBSTATE_RX_ENERGY_SCAN:
            case OT_RADIO_SUBSTATE_STARTUP:
                energyState = OT_RADIO_STATE_RECEIVE;
                break;
            case OT_RADIO_SUBSTATE_TX_FRAME_ONGOING:
            case OT_RADIO_SUBSTATE_RX_ACK_TX_ONGOING:
                energyState = OT_RADIO_STATE_TRANSMIT;
                break;
            default:
                energyState = OT_RADIO_STATE_SLEEP;
                break;
        }

        stateReport.mChannel     = sOngoingOperationChannel;
        stateReport.mEnergyState = energyState;
        stateReport.mSubState    = sSubState;
        stateReport.mTxPower     = sTxPower;
        stateReport.mState       = sState; // also include the OT radio state.

        // determine next radio-event time, so that simulator can guarantee this node will
        // execute again at that time.
        uint64_t delayUntilNextRadioState = 0;
        if (sNextRadioEventTime > otPlatTimeGet())
        {
            delayUntilNextRadioState = sNextRadioEventTime - otPlatTimeGet();
        }
        otSimSendRadioStateEvent(&stateReport, delayUntilNextRadioState);
    }
}

void setRadioState(otRadioState aState)
{
    sState = aState;
}

static void setRadioSubState(RadioSubState aState, uint64_t timeToRemainInState)
{
    if (timeToRemainInState == UNDEFINED_TIME_US)
    {
        sNextRadioEventTime = UNDEFINED_TIME_US;
    }else
    {
        sNextRadioEventTime = otPlatTimeGet() + timeToRemainInState;
    }
    sSubState = aState;
}

static void startCcaForTransmission(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    // send CCA event, wait for simulator to send back the channel sampling result.
    struct RadioCommEventData chanSampleData;
    chanSampleData.mChannel = sTransmitFrame.mChannel;
    chanSampleData.mDuration = OT_RADIO_CCA_TIME_US;
    otSimSendRadioChanSampleEvent(&chanSampleData);
}

bool platformRadioIsBusy(void)
{
    return (sState == OT_RADIO_STATE_TRANSMIT || sState == OT_RADIO_STATE_RECEIVE ) &&
           (sSubState != OT_RADIO_SUBSTATE_READY);
}

void platformRadioRxStart(otInstance *aInstance, struct RadioCommEventData *aRxParams)
{
    OT_UNUSED_VARIABLE(aInstance);

    otEXPECT(sOngoingOperationChannel == aRxParams->mChannel); // must be on my listening channel.
    otEXPECT(sState == OT_RADIO_STATE_RECEIVE || sState == OT_RADIO_STATE_TRANSMIT); // and in valid states.
    otEXPECT(sSubState == OT_RADIO_SUBSTATE_READY || sSubState == OT_RADIO_SUBSTATE_IFS_WAIT ||
             sSubState == OT_RADIO_SUBSTATE_TX_AIFS_WAIT);

    // radio can only receive in particular states.
    if (sSubState == OT_RADIO_SUBSTATE_TX_AIFS_WAIT)
    {
        setRadioSubState(OT_RADIO_SUBSTATE_TX_ACK_RX_ONGOING, aRxParams->mDuration + FAILSAFE_TIME_US);
    }
    else
    {
        setRadioSubState(OT_RADIO_SUBSTATE_RX_FRAME_ONGOING, aRxParams->mDuration + FAILSAFE_TIME_US);
    }

    // Record SFD end-of-last-symbol timestamp. The simulator signals start of first symbol of
    // preamble so we need to adapt to "when SFD was received" (end of last symbol of SFD = start of PHY hdr)
    sReceiveTimestamp = otPlatTimeGet() + OT_RADIO_SHR_DURATION_US;

    exit:
    return;
}

void platformRadioRxDone(otInstance *aInstance, const uint8_t *aBuf, uint16_t aBufLength, struct RadioCommEventData *aRxParams)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_ASSERT(sizeof(sReceiveMessage) >= aBufLength);

    // only process in valid substates:
    otEXPECT( sSubState == OT_RADIO_SUBSTATE_RX_FRAME_ONGOING || sSubState == OT_RADIO_SUBSTATE_TX_ACK_RX_ONGOING );
    memcpy(&sReceiveMessage, aBuf, aBufLength);

    sReceiveFrame.mLength             = (uint8_t) aBufLength - offsetof(struct RadioMessage, mPsdu);
    sReceiveFrame.mInfo.mRxInfo.mRssi = aRxParams->mPower;
    sReceiveFrame.mInfo.mRxInfo.mLqi  = OT_RADIO_LQI_NONE; // No support of LQI reporting.

    bool isAck = otMacFrameIsAck(&sReceiveFrame);
    bool isAckRequested = otMacFrameIsAckRequested(&sReceiveFrame);
    bool isAddressedToMe = otMacFrameDoesAddrMatch(&sReceiveFrame, sPanid, sShortAddress, &sExtAddress);

    radioReceive(aInstance, aRxParams->mError );

    if (sSubState == OT_RADIO_SUBSTATE_RX_FRAME_ONGOING && isAckRequested && !isAck && isAddressedToMe)
    {
        // Rx done, need to send Ack. Wait exactly time AIFS before sending out the Ack.
        setRadioSubState(OT_RADIO_SUBSTATE_RX_AIFS_WAIT, OT_RADIO_AIFS_TIME_US);
    }
    else if (sSubState == OT_RADIO_SUBSTATE_RX_FRAME_ONGOING)
    {
        // Rx done, but no Ack is sent. Wait at least turnaround time before I'm ready to Tx (if needed).
        setRadioSubState(OT_RADIO_SUBSTATE_IFS_WAIT, OT_RADIO_TURNAROUND_TIME_US);
    }
    else if (sSubState == OT_RADIO_SUBSTATE_TX_ACK_RX_ONGOING)
    {
        // I was in Tx, and a frame (likely the expected Ack, but maybe not) is received. Need to wait IFS time
        // before I can transmit again.
        uint64_t ifsTime = (sTransmitFrame.mLength > OT_RADIO_aMaxSifsFrameSize) ? OT_RADIO_LIFS_TIME_US : OT_RADIO_SIFS_TIME_US;
        setRadioSubState(OT_RADIO_SUBSTATE_IFS_WAIT, ifsTime);
    }

    exit:
    return;
}

void platformRadioCcaDone(otInstance *aInstance, struct RadioCommEventData *aChanData)
{
    OT_UNUSED_VARIABLE(aInstance);
    otEXPECT(aChanData->mChannel == sTransmitFrame.mChannel);
    otEXPECT(sSubState == OT_RADIO_SUBSTATE_TX_CCA);

    if (aChanData->mPower < sCcaEdThresh || aChanData->mPower == OT_RADIO_RSSI_INVALID)  // channel clear?
    {
        setRadioSubState(OT_RADIO_SUBSTATE_TX_CCA_TO_TX, OT_RADIO_TURNAROUND_TIME_US);
    }
    else
    {
        // CCA failure case - channel not clear.
        setRadioSubState(OT_RADIO_SUBSTATE_READY, UNDEFINED_TIME_US);
        signalRadioTxDone(aInstance, &sTransmitFrame, NULL, OT_ERROR_CHANNEL_ACCESS_FAILURE);
    }

    exit:
    return;
}

void platformRadioTxDone(otInstance *aInstance, struct RadioCommEventData *aTxDoneParams)
{
    OT_UNUSED_VARIABLE(aInstance);

    if (sSubState == OT_RADIO_SUBSTATE_RX_ACK_TX_ONGOING)
    {
        // Ack Tx is done now.
        setRadioSubState(OT_RADIO_SUBSTATE_RX_TX_TO_RX, OT_RADIO_TURNAROUND_TIME_US);
    }
    else if (sSubState == OT_RADIO_SUBSTATE_TX_FRAME_ONGOING)
    {
        // if not waiting for ACK -> go to Rx state; see state diagram.
        // if Tx was failure: no wait for ACK, abort current Tx and go to Rx state.
        if (!otMacFrameIsAckRequested(&sTransmitFrame) || aTxDoneParams->mError != OT_ERROR_NONE)
        {
            setRadioSubState(OT_RADIO_SUBSTATE_TX_TX_TO_RX, OT_RADIO_TURNAROUND_TIME_US);
            signalRadioTxDone(aInstance, &sTransmitFrame, NULL, aTxDoneParams->mError);
        }
        else
        {
            // Ack frame is to be received, change radio from Tx to Rx mode; move into AIFS wait period.
            // TODO: could let radio sleep during part of AIFS, to save energy.
            setRadioSubState(OT_RADIO_SUBSTATE_TX_TX_TO_AIFS, OT_RADIO_TURNAROUND_TIME_US);
        }
    }
}

void platformRadioProcess(otInstance *aInstance, const fd_set *aReadFdSet, const fd_set *aWriteFdSet)
{
    OT_UNUSED_VARIABLE(aReadFdSet);
    OT_UNUSED_VARIABLE(aWriteFdSet);

    // if stack wants to transmit a frame while radio is busy with Rx: signal CCA failure directly.
    // there is no need to sample the radio channel in this case. Also do not wait until the end of Rx period to
    // signal the error, otherwise multiple radio nodes become sync'ed on their CCA period that would follow.
    if (platformRadioIsTransmitPending() && platformRadioIsReceiveOngoing())
    {
        signalRadioTxDone(aInstance, &sTransmitFrame, NULL, OT_ERROR_CHANNEL_ACCESS_FAILURE);
    }

    // Tx/Rx state machine. Execute time and data based state transitions for substate.
    // Event based transitions are in functions called by platform-sim.c receiveEvent().
    if (otPlatTimeGet() >= sNextRadioEventTime)
    {
        uint64_t ifsTime = (sTransmitFrame.mLength > OT_RADIO_aMaxSifsFrameSize) ? OT_RADIO_LIFS_TIME_US : OT_RADIO_SIFS_TIME_US;
        switch (sSubState)
        {
            case OT_RADIO_SUBSTATE_DISABLED: // when radio circuitry is in disabled mode.
                if (sState != OT_RADIO_STATE_DISABLED){
                    setRadioSubState(OT_RADIO_SUBSTATE_STARTUP, OT_RADIO_STARTUP_TIME_US);
                }
                break;

            case OT_RADIO_SUBSTATE_SLEEP: // when radio is in sleep mode.
                if (sState == OT_RADIO_STATE_RECEIVE || platformRadioIsTransmitPending()) {
                    setRadioSubState(OT_RADIO_SUBSTATE_STARTUP, OT_RADIO_STARTUP_TIME_US);
                } else if (sState == OT_RADIO_STATE_DISABLED){
                    setRadioSubState(OT_RADIO_SUBSTATE_DISABLED, UNDEFINED_TIME_US);
                }
                break;

            case OT_RADIO_SUBSTATE_STARTUP: // at end of radio startup (coming either from Sleep or from Disabled).
                if (sState == OT_RADIO_STATE_SLEEP) {
                    setRadioSubState(OT_RADIO_SUBSTATE_SLEEP, UNDEFINED_TIME_US);
                }else {
                    setRadioSubState(OT_RADIO_SUBSTATE_READY, UNDEFINED_TIME_US);
                }
                break;

            case OT_RADIO_SUBSTATE_READY:  // in ready/idle substate: decide when to start transmitting frame.
                sOngoingOperationChannel = sCurrentChannel;
                if (platformRadioIsTransmitPending())
                {
                    setRadioSubState(OT_RADIO_SUBSTATE_TX_CCA, OT_RADIO_CCA_TIME_US + FAILSAFE_TIME_US);
                    startCcaForTransmission(aInstance);
                }else if (sState == OT_RADIO_STATE_SLEEP){
                    // radio to sleep when nothing to transmit and sleep mode was requested.
                    setRadioSubState(OT_RADIO_SUBSTATE_SLEEP, UNDEFINED_TIME_US);
                }else if (sState == OT_RADIO_STATE_DISABLED){
                    setRadioSubState(OT_RADIO_SUBSTATE_DISABLED, UNDEFINED_TIME_US);
                }
                break;

            case OT_RADIO_SUBSTATE_TX_CCA:
                // CCA period timed out without CCA sample from simulator. Normally should not happen.
                signalRadioTxDone(aInstance, &sTransmitFrame, NULL, OT_ERROR_CHANNEL_ACCESS_FAILURE);
                setRadioSubState(OT_RADIO_SUBSTATE_READY, UNDEFINED_TIME_US);
                break;

            case OT_RADIO_SUBSTATE_TX_CCA_TO_TX: // at end of transition from end-of-CCA to Tx
                radioSendMessage(aInstance); // Creates the sLastTxEventData.
                setRadioSubState(OT_RADIO_SUBSTATE_TX_FRAME_ONGOING, sLastTxEventData.mDuration + FAILSAFE_TIME_US);
                break;

            case OT_RADIO_SUBSTATE_TX_FRAME_ONGOING:
                setRadioSubState(OT_RADIO_SUBSTATE_TX_TX_TO_RX, OT_RADIO_TURNAROUND_TIME_US);
                break;

            case OT_RADIO_SUBSTATE_TX_TX_TO_RX:
                // no Ack was requested
                setRadioState(OT_RADIO_STATE_RECEIVE);
                setRadioSubState(OT_RADIO_SUBSTATE_IFS_WAIT, ifsTime - OT_RADIO_TURNAROUND_TIME_US);
                break;

            case OT_RADIO_SUBSTATE_TX_TX_TO_AIFS:
                // set a max wait time for start of Ack frame to be received.
                setRadioSubState(OT_RADIO_SUBSTATE_TX_AIFS_WAIT, OT_RADIO_MAX_ACK_WAIT_US);
                break;

            case OT_RADIO_SUBSTATE_TX_AIFS_WAIT:
                // if we arrive here on the timeout timer, an Ack or frame start wasn't received in the meantime.
                // so go to ready state and fail the Tx.
                setRadioSubState(OT_RADIO_SUBSTATE_READY, UNDEFINED_TIME_US);
                signalRadioTxDone(aInstance, &sTransmitFrame, NULL, OT_ERROR_NO_ACK);
                break;

            case OT_RADIO_SUBSTATE_TX_ACK_RX_ONGOING:
                // wait until Ack receive is done. In platformRadioRxDone() the next state is selected.
                // if we get here on the timer, the presumed Ack frame reception was never finished.
                setRadioSubState(OT_RADIO_SUBSTATE_IFS_WAIT, ifsTime);
                signalRadioTxDone(aInstance, &sTransmitFrame, NULL, OT_ERROR_NO_ACK);
                break;

            case OT_RADIO_SUBSTATE_IFS_WAIT:
                setRadioSubState(OT_RADIO_SUBSTATE_READY, UNDEFINED_TIME_US);
                break;

                // --- Below is the state machine for Rx states.
            case OT_RADIO_SUBSTATE_RX_FRAME_ONGOING:
                // wait until frame Rx is done. In platformRadioRxDone() the next state is selected.
                // below is a timer-based failsafe in case the RxDone message from simulator was never received.
                setRadioSubState(OT_RADIO_SUBSTATE_IFS_WAIT, OT_RADIO_TURNAROUND_TIME_US);
                break;

            case OT_RADIO_SUBSTATE_RX_AIFS_WAIT:
                // if Ack is ready to be transmitted after AIFS period, send it.
                radioPrepareAck();  // prepare the Ack again, now (redo it - with proper CSL timing)
                radioTransmit(&sAckMessage, &sAckFrame);  // send the Ack. Creates sLastTxEventData.
                setRadioSubState(OT_RADIO_SUBSTATE_RX_ACK_TX_ONGOING, sLastTxEventData.mDuration);
                break;

            case OT_RADIO_SUBSTATE_RX_ACK_TX_ONGOING:
                // at end of Ack transmission by receiver.
                setRadioSubState(OT_RADIO_SUBSTATE_RX_TX_TO_RX, OT_RADIO_TURNAROUND_TIME_US);
                break;

            case OT_RADIO_SUBSTATE_RX_TX_TO_RX:
                // After Ack Tx by receiver.
                setRadioSubState(OT_RADIO_SUBSTATE_IFS_WAIT, OT_RADIO_TURNAROUND_TIME_US);
                break;

            case OT_RADIO_SUBSTATE_RX_ENERGY_SCAN:
                if (IsTimeAfterOrEqual(otPlatAlarmMilliGetNow(), sEnergyScanEndTime))
                {
                    otPlatRadioEnergyScanDone(aInstance, sEnergyScanResult);
                    setRadioSubState(OT_RADIO_SUBSTATE_READY, UNDEFINED_TIME_US);
                    sEnergyScanning = false;
                }
                break;

            default:
                OT_ASSERT(false && "Illegal state found");
        }
    }
}
