/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * Log platform hooks for STM32H7: timestamps from the Cortex-M7 cycle
 * counter (DWT->CYCCNT), extended to 64 bits in software. The counter wraps
 * every 10.7 s at 400 MHz; log_task() reads it every few milliseconds, so no
 * wrap is missed while the log thread runs.
 */

#include "core/log.h"
#include "stm32h7xx.h"

#define CPU_MHZ 400u // see startPll() in rcc.cpp

static uint32_t lastCycles;
static uint32_t wraps;

void log_portInit()
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55; // unlock the DWT, required on Cortex-M7
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t log_portLock()
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

void log_portUnlock(uint32_t key)
{
    __set_PRIMASK(key);
}

uint64_t log_portTime()
{
    uint32_t cycles = DWT->CYCCNT;
    if (cycles < lastCycles)
        wraps++;

    lastCycles = cycles;
    return (((uint64_t)wraps) << 32) | cycles;
}

uint64_t log_portTimeToUs(uint64_t time)
{
    return time / CPU_MHZ;
}
