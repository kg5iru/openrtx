/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 * 
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef SETTINGS_H
#define SETTINGS_H

#include "hwconfig.h"
#include <stdbool.h>

typedef enum
{
    TIMER_OFF =  0,
    TIMER_5S  =  1,
    TIMER_10S =  2,
    TIMER_15S =  3,
    TIMER_20S =  4,
    TIMER_25S =  5,
    TIMER_30S =  6,
    TIMER_1M  =  7,
    TIMER_2M  =  8,
    TIMER_3M  =  9,
    TIMER_4M  = 10,
    TIMER_5M  = 11,
    TIMER_15M = 12,
    TIMER_30M = 13,
    TIMER_45M = 14,
    TIMER_1H  = 15
}
display_timer_t;

typedef struct
{
    uint8_t brightness;           // Display brightness
    uint8_t contrast;             // Display contrast
    uint8_t sqlLevel;             // Squelch level
    uint8_t voxLevel;             // Vox level
    int8_t  utc_timezone;         // Timezone, in units of half hours
    bool    gps_enabled;          // GPS active
    char    callsign[10];         // Plaintext callsign
    uint8_t display_timer   : 4,  // Standby timer
            m17_can         : 4;  // M17 CAN
    uint8_t vpLevel         : 3,  // Voice prompt level
            vpPhoneticSpell : 1,  // Phonetic spell enabled
            macroMenuLatch  : 1,  // Automatic latch of macro menu
            _reserved       : 3;
    bool    m17_can_rx;           // Check M17 CAN on RX
    char    m17_dest[10];         // M17 destination
    bool    showBatteryIcon;      // Battery display true: icon, false: percentage
    bool    gpsSetTime;           // Use GPS to ajust RTC time
    char    M17_meta_text[53];    // M17 Meta Text to send
#ifdef CONFIG_DMR_CS7000
    uint8_t dmr_cc;               // DMR colour code + 1, 0 = not set (1)
    uint32_t dmr_rxTg;            // DMR talkgroup to receive, 0 = all
    uint8_t dmr_ts;               // DMR timeslot to receive, 0 = any, 1 or 2
    uint32_t dmr_id;              // Own DMR radio ID, 0 = not set (no TX)
    uint8_t dmr_micGain;          // DMR mic gain step + 1, 0 = not set (0 dB)
    uint32_t dmr_privId;          // DMR private call target, 0 = group calls
    uint8_t dmr_taOff;            // 1 = don't send the DMR talker alias
    uint8_t dmr_monitor;          // DMR monitor: 0 off, 1 own TS, 2 any TS
    uint32_t dmr_aprsId;          // DMR GPS/APRS destination, 0 = off
    uint8_t dmr_aprsPtt;          // 1 = send the position after each TX
    uint8_t dmr_aprsInterval;     // DMR APRS beacon interval index, 0 = off
#endif
#ifdef CONFIG_YSF_CS7000
    char    ysf_dest[10];         // YSF destination callsign, empty = all
    uint8_t ysf_txDgid;           // YSF DG-ID sent, 0..99
    uint8_t ysf_rxDgid;           // YSF DG-ID received, 0 = all, 1..99
#endif
}
__attribute__((packed)) settings_t;


static const settings_t default_settings =
{
    100,                          // Brightness
#ifdef CONFIG_SCREEN_CONTRAST
    CONFIG_DEFAULT_CONTRAST,      // Contrast
#else
    255,                          // Contrast
#endif
    4,                            // Squelch level, 4 = S3
    0,                            // Vox level
    0,                            // UTC Timezone
    false,                        // GPS enabled
    "N0CALL",                     // Default callsign
    TIMER_30S,                    // 30 seconds
    0,                            // M17 CAN
    0,                            // Voice prompts off
    0,                            // Phonetic spell off
    1,                            // Automatic latch of macro menu enabled
    0,                            // not used
    false,                        // Check M17 CAN on RX
    "",                           // Empty M17 destination
    false,                        // Display battery icon
    false,                        // Update RTC with GPS
    "OpenRTX",                    // Default M17 meta text
#ifdef CONFIG_DMR_CS7000
    0,                            // DMR colour code not set (1)
    0,                            // Receive all DMR talkgroups
    0,                            // Receive either DMR timeslot
    0,                            // DMR radio ID not set
    0,                            // DMR mic gain not set (0 dB)
    0,                            // DMR group calls
    0,                            // Send the DMR talker alias
    0,                            // DMR monitor off
    0,                            // DMR APRS off
    0,                            // No DMR APRS after TX
    0,                            // No DMR APRS beacon
#endif
#ifdef CONFIG_YSF_CS7000
    "",                           // YSF call to all
    0,                            // YSF DG-ID 0
    0,                            // Receive every YSF DG-ID
#endif
};

#ifdef CONFIG_DMR_CS7000
/*
 * The DMR fields were added at the end of settings_t: settings saved by an
 * older build read back with them zeroed, so the colour code is stored plus
 * one and zero means the usual default of 1.
 */
static inline uint8_t settings_dmrColourCode(const settings_t *s)
{
    return (s->dmr_cc == 0) ? 1 : ((s->dmr_cc - 1) & 0x0F);
}

static inline void settings_setDmrColourCode(settings_t *s, uint8_t cc)
{
    s->dmr_cc = (cc & 0x0F) + 1;
}

/*
 * DMR microphone gain, in 3 dB steps from -12 dB (step 0) to +12 dB (step
 * 8); stored plus one, zero (older settings) meaning 0 dB.
 */
#define DMR_MIC_GAIN_STEPS 9

static inline uint8_t settings_dmrMicGainStep(const settings_t *s)
{
    if ((s->dmr_micGain == 0) || (s->dmr_micGain > DMR_MIC_GAIN_STEPS))
        return 4;
    return s->dmr_micGain - 1;
}

static inline void settings_setDmrMicGainStep(settings_t *s, uint8_t step)
{
    s->dmr_micGain = (step % DMR_MIC_GAIN_STEPS) + 1;
}

/*
 * DMR digital monitor: 0 = off (talkgroup, private call and timeslot
 * filters apply), 1 = single timeslot (every call on the timeslot set),
 * 2 = any timeslot (every call).
 */
static inline uint8_t settings_dmrMonitor(const settings_t *s)
{
    return (s->dmr_monitor > 2) ? 0 : s->dmr_monitor;
}

/*
 * DMR GPS position (digital APRS) beacon interval: index 0 = off, then 1, 2,
 * 5, 10 and 30 minutes.
 */
#define DMR_APRS_INTERVALS 6

static inline uint16_t settings_dmrAprsIntervalSec(uint8_t index)
{
    static const uint16_t sec[DMR_APRS_INTERVALS] = { 0, 60, 120, 300, 600,
                                                      1800 };
    return (index < DMR_APRS_INTERVALS) ? sec[index] : 0;
}

// DMR timeslot filter: 0 = either timeslot, 1 = TS1, 2 = TS2
static inline uint8_t settings_dmrTimeslot(const settings_t *s)
{
    return (s->dmr_ts > 2) ? 0 : s->dmr_ts;
}
#endif

#ifdef CONFIG_YSF_CS7000
/*
 * YSF DG-ID (Digital Group ID), 0..99, as Yaesu radios set it.  Received
 * calls are filtered on the RX DG-ID unless it is 0, which receives every
 * DG-ID, as on a Yaesu radio.
 */
#define YSF_DGID_MAX 99

static inline uint8_t settings_ysfDgid(uint8_t value)
{
    return (value > YSF_DGID_MAX) ? 0 : value;
}
#endif

#endif /* SETTINGS_H */
