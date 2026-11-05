/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 * 
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef RTX_H
#define RTX_H

#include "core/datatypes.h"
#include <stdint.h>
#include "core/cps.h"
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint8_t opMode;         /**< Operating mode (FM, DMR, ...) */

    uint8_t bandwidth : 2,  /**< Channel bandwidth             */
            txDisable : 1,  /**< Disable TX operation          */
            scan      : 1,  /**< Scan enabled                  */
            opStatus  : 2,  /**< Operating status (OFF, ...)   */
            _padding  : 2;  /**< Padding to 8 bits             */

    freq_t rxFrequency;     /**< RX frequency, in Hz           */
    freq_t txFrequency;     /**< TX frequency, in Hz           */

    uint32_t txPower;       /**< TX power, in mW               */
    uint8_t  sqlLevel;      /**< Squelch opening level         */

    uint16_t rxToneEn : 1,  /**< RX CTC/DCS tone enable        */
             rxTone   : 15; /**< RX CTC/DCS tone               */

    uint16_t txToneEn : 1,  /**< TX CTC/DCS tone enable        */
             txTone   : 15; /**< TX CTC/DCS tone               */

    bool     toneEn;

    uint8_t  can      : 4,  /**< M17 Channel Access Number     */
             canRxEn  : 1,  /**< M17 Check CAN on RX           */
             _unused  : 3;

    char     source_address[10];       /**< M17 call source address    */
    char     destination_address[10];  /**< M17 call routing address   */
    bool     invertRxPhase;            /**< M17 RX phase inversion     */
    bool     lsfOk;                    /**  M17 LSF is valid           */
    char     M17_dst[10];              /**  M17 LSF destination        */
    char     M17_src[10];              /**  M17 LSF source             */
    char     M17_link[10];             /**  M17 LSF traffic originator */
    char     M17_refl[10];             /**  M17 LSF reflector module   */
    char     M17_meta_text[53];        /**< M17 Meta Text              */
#ifdef CONFIG_DMR_CS7000
    uint8_t  dmrColourCode;            /**< DMR colour code            */
    uint32_t dmrRxTalkgroup;           /**< DMR RX talkgroup, 0 = all  */
    uint8_t  dmrTimeslot;              /**< DMR RX timeslot, 0 = any   */
    uint32_t dmrRadioId;               /**< Own DMR ID, 0 = not set    */
    uint8_t  dmrMicGain;               /**< DMR mic gain step, 0..8    */
    uint32_t dmrPrivateId;             /**< DMR private call ID, 0 = TG */
    bool     dmrTalkerAlias;           /**< DMR: send callsign as alias */
    uint8_t  dmrMonitor;               /**< DMR monitor: 0, 1 TS, 2 any */
    uint32_t dmrAprsId;                /**< DMR APRS destination, 0 off */
    bool     dmrAprsPtt;               /**< DMR APRS after each TX      */
    uint16_t dmrAprsInterval;          /**< DMR APRS beacon, s, 0 off   */
#endif
#if defined(CONFIG_P25)
    uint32_t P25_SrcId;                /**< P25 received call source ID    */
    uint32_t P25_DstId;                /**< P25 received call destination  */
    uint32_t p25SrcId;                 /**< P25 own ID (DMR ID)            */
    uint32_t p25DstId;                 /**< P25 destination ID (talkgroup) */
    uint16_t p25Nac;                   /**< P25 network access code        */
    uint8_t  p25TxLevel;               /**< P25 TX baseband level          */
    uint8_t  p25Preamble;              /**< P25 TX preamble option         */
    uint8_t  p25TxInvert : 1,          /**< P25 invert TX baseband         */
             p25RxInvert : 1,          /**< P25 invert RX baseband         */
             p25DstCheck : 1;          /**< P25 play only calls to us      */
#endif
#ifdef CONFIG_YSF_CS7000
    char     ysfDest[11];              /**< YSF destination, "" = all   */
    uint8_t  ysfTxDgid;                /**< YSF DG-ID sent, 0..99       */
    uint8_t  ysfRxDgid;                /**< YSF DG-ID received, 0 = all */
#endif
#if defined(CONFIG_YSF_CS7000) && defined(CONFIG_P25)
    bool     ysfTxVw;                  /**< YSF voice full rate (VW)    */
#endif
#ifdef CONFIG_NXDN_CS7000
    uint8_t  nxdnRan;                  /**< NXDN RAN, 0 = any           */
    uint16_t nxdnId;                   /**< Own NXDN ID, 0 = not set    */
    uint16_t nxdnTg;                   /**< NXDN talkgroup called       */
#endif
}
rtxStatus_t;

/**
 * \enum bandwidth Enumeration type defining the current rtx bandwidth.
 */
enum bandwidth
{
    BW_12_5 = 0,    /**< 12.5kHz bandwidth */
    BW_25   = 1     /**< 25kHz bandwidth   */
};

/**
 * \enum opmode Enumeration type defining the current rtx operating mode.
 */
enum opmode
{
    OPMODE_NONE = 0,        /**< No opMode selected */
    OPMODE_FM   = 1,        /**< Analog FM          */
    OPMODE_DMR  = 2,        /**< DMR                */
    OPMODE_M17  = 3,        /**< M17                */
#ifdef CONFIG_P25
    OPMODE_P25  = 4,        /**< P25                */
#endif
    OPMODE_YSF  = 5,        /**< Yaesu System Fusion */
    OPMODE_NXDN = 6         /**< NXDN (NXDN48)       */
};

/**
 * \enum opstatus Enumeration type defining the current rtx operating status.
 */
enum opstatus
{
    OFF = 0,        /**< OFF          */
    RX  = 1,        /**< Receiving    */
    TX  = 2         /**< Transmitting */
};


/**
 * Initialise rtx stage.
 * @param m: pointer to the mutex protecting the shared configuration data
 * structure.
 */
void rtx_init(pthread_mutex_t *m);

/**
 * Shut down rtx stage
 */
void rtx_terminate();

/**
 * Post a new RTX configuration on the internal message queue. Data structure
 * \b must be protected by the same mutex whose pointer has been passed as a
 * parameter to rtx_init(). This driver only copies its content into the internal
 * data structure, eventual garbage collection has to be performed by caller.
 * @param cfg: pointer to a structure containing the new RTX configuration.
 */
void rtx_configure(const rtxStatus_t *cfg);

/**
 * Obtain a copy of the RTX driver's internal status data structure.
 * @return copy of the RTX driver's internal status data structure.
 */
rtxStatus_t rtx_getCurrentStatus();

/**
 * High-level code is in charge of calling this function periodically, since it
 * contains all the RTX management functionalities.
 */
void rtx_task();

/**
 * Get current RSSI in dBm.
 * @return RSSI value in dBm.
 */
rssi_t rtx_getRssi();

/**
 * Get current status of the RX squelch. This function is thread-safe and can
 * be called also from threads other than the one running the RTX task.
 * @return true if RX squelch is open.
 */
bool rtx_rxSquelchOpen();

#ifdef __cplusplus
}
#endif

#endif /* RTX_H */
