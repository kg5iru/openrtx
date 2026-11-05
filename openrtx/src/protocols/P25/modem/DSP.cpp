/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "protocols/P25/modem/DSP.hpp"
#include "hwconfig.h"

#ifdef CONFIG_P25

Fir<std::tuple_size<decltype(P25::tx_pulse_48k)>::value>
    P25::pulse_48k(P25::tx_pulse_48k);
Fir<std::tuple_size<decltype(P25::rx_filter_24k)>::value>
    P25::rxf_24k(P25::rx_filter_24k);

#endif /* CONFIG_P25 */
