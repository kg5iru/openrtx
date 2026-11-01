/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * Lets the STM32H7 CMSIS headers compile on a PC: replaces cmsis_gcc.h (ARM
 * inline assembly) with plain definitions. Force-included by run.sh.
 */

#ifndef HOST_CMSIS_H
#define HOST_CMSIS_H

#include <stdint.h>

#define __CMSIS_GCC_H
#define __ASM __asm
#define __INLINE inline
#define __STATIC_INLINE static inline
#define __STATIC_FORCEINLINE static inline
#define __NO_RETURN __attribute__((__noreturn__))
#define __USED __attribute__((used))
#define __WEAK __attribute__((weak))
#define __PACKED __attribute__((packed, aligned(1)))
#define __PACKED_STRUCT struct __attribute__((packed, aligned(1)))
#define __PACKED_UNION union __attribute__((packed, aligned(1)))
#define __ALIGNED(x) __attribute__((aligned(x)))
#define __RESTRICT __restrict
#define __COMPILER_BARRIER() __asm volatile("" ::: "memory")

static inline void __DSB(void)
{
}
static inline void __ISB(void)
{
}
static inline void __DMB(void)
{
}
static inline void __NOP(void)
{
}
static inline void __disable_irq(void)
{
}
static inline void __enable_irq(void)
{
}
static inline uint32_t __get_PRIMASK(void)
{
    return 0;
}
static inline void __set_PRIMASK(uint32_t v)
{
    (void)v;
}

#endif
