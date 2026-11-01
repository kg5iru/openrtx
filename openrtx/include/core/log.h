/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef LOG_H
#define LOG_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * System log, sent to a host over rtxlink (protocol RTXLINK_FRAME_LOG).
 *
 *     LOG_E("dmr", "bad sync 0x%02lx", sync);
 *     LOG_D("dmr", "burst %lu slot %lu", n, slot);
 *     LOG_HEX(LOG_LVL_DEBUG, "dmr", "lc", lcBytes, 9);
 *     LOG_TEXT(LOG_LVL_INFO, "ui", callsignBuffer);
 *
 * Calls are cheap and safe from any context, interrupts included: a call
 * takes a timestamp and copies the format pointer and the raw arguments into
 * a ring buffer with interrupts disabled for a few dozen cycles. Formatting
 * happens later, in the log thread, with snprintf.
 *
 * Rules that follow from deferred formatting:
 * - At most 8 arguments, each at most 32 bits wide (pointer size on the
 *   host). 64-bit integers and doubles are rejected at compile time; floats
 *   are converted to integers, so scale them first.
 * - On ARM, uint32_t is 'unsigned long': print it with %lu / %lx. The format
 *   is checked by GCC like a printf, so mismatches give a warning.
 * - "%s" arguments must point to strings that live forever (literals,
 *   constant tables). For anything else use LOG_TEXT, which copies up to
 *   LOG_MAX_BYTES characters.
 *
 * When the ring is full new records are dropped and counted; a warning with
 * the count is emitted once there is room again. Records wait in the ring
 * until a host opens the port (DTR set), so boot messages are not lost.
 *
 * Without CONFIG_LOG every macro compiles to nothing.
 */

enum LogLevel {
    LOG_LVL_NONE = 0,
    LOG_LVL_ERROR = 1,
    LOG_LVL_WARN = 2,
    LOG_LVL_INFO = 3,
    LOG_LVL_DEBUG = 4
};

#define LOG_MAX_ARGS 8
#define LOG_MAX_BYTES 32

/* printf format checking, never called. */
static inline void __attribute__((format(printf, 1, 2)))
log_checkFormat(const char *fmt, ...)
{
    (void)fmt;
}

#ifdef CONFIG_LOG

/* Levels above this one are removed at compile time. */
#ifndef LOG_COMPILE_LEVEL
#define LOG_COMPILE_LEVEL LOG_LVL_DEBUG
#endif

/* Level in effect at boot, can be changed at run time from the host. */
#ifndef LOG_DEFAULT_LEVEL
#define LOG_DEFAULT_LEVEL LOG_LVL_DEBUG
#endif

/**
 * Set up the log and start rtxlink on the given device. Logging works before
 * this call too: records are kept until the host connects.
 *
 * @return the device initialization result, 0 on success.
 */
struct chardev;
int log_init(const struct chardev *dev);

/**
 * Send queued records and handle host requests. Call from one low-priority
 * thread every few milliseconds.
 */
void log_task();

/** Change the run-time level. */
void log_setLevel(uint8_t level);

/** Current run-time level. */
uint8_t log_getLevel();

/* Back ends of the macros below, not meant to be called directly. */
void log_write(uint8_t level, const char *tag, const char *fmt, unsigned nargs,
               ...);
void log_hexdump(uint8_t level, const char *tag, const char *label,
                 const void *data, size_t len);
void log_text(uint8_t level, const char *tag, const char *text);

/*
 * Platform hooks, implemented once per MCU family.
 * - log_portLock/Unlock: disable and restore all interrupts.
 * - log_portTime: free-running 64-bit timestamp, called with the lock held.
 * - log_portTimeToUs: convert a timestamp to microseconds.
 */
void log_portInit();
uint32_t log_portLock();
void log_portUnlock(uint32_t key);
uint64_t log_portTime();
uint64_t log_portTimeToUs(uint64_t time);

/* Argument handling: count, size check and cast to uintptr_t. */
#define LOG_CAT_(a, b) a##b
#define LOG_CAT(a, b) LOG_CAT_(a, b)
#define LOG_FIRST_(f, ...) f
#define LOG_FIRST(...) LOG_FIRST_(__VA_ARGS__, _)
#define LOG_COUNT_(f, _1, _2, _3, _4, _5, _6, _7, _8, N, ...) N
#define LOG_COUNT(...) LOG_COUNT_(__VA_ARGS__, 8, 7, 6, 5, 4, 3, 2, 1, 0, _)
/*
 * A compile error "size of array 'log_argument_wider_than_32_bits' is
 * negative" (or "size of unnamed array is negative" in C++) means a 64-bit
 * integer or a double was passed: see the rules at the top of this file.
 */
#ifdef __cplusplus
#define LOG_DECAY(x) (+(x)) // arrays become pointers, chars ints
#define LOG_CHECK(x) \
    ((void)sizeof(char[(sizeof(LOG_DECAY(x)) <= sizeof(uintptr_t)) ? 1 : -1]))
#else
#define LOG_DECAY(x) (0 ? (x) : (x))
#define LOG_CHECK(x)                                                \
    ((void)sizeof(struct {                                          \
        char log_argument_wider_than_32_bits                        \
            [(sizeof(LOG_DECAY(x)) <= sizeof(uintptr_t)) ? 1 : -1]; \
    }))
#endif
#define LOG_ARG(x) (LOG_CHECK(x), (uintptr_t)(x))
#define LOG_MAP0(f)
#define LOG_MAP1(f, a) , LOG_ARG(a)
#define LOG_MAP2(f, a, b) LOG_MAP1(f, a) LOG_MAP1(f, b)
#define LOG_MAP3(f, a, b, c) LOG_MAP2(f, a, b) LOG_MAP1(f, c)
#define LOG_MAP4(f, a, b, c, d) LOG_MAP2(f, a, b) LOG_MAP2(f, c, d)
#define LOG_MAP5(f, a, b, c, d, e) LOG_MAP4(f, a, b, c, d) LOG_MAP1(f, e)
#define LOG_MAP6(f, a, b, c, d, e, g) LOG_MAP4(f, a, b, c, d) LOG_MAP2(f, e, g)
#define LOG_MAP7(f, a, b, c, d, e, g, h) \
    LOG_MAP4(f, a, b, c, d) LOG_MAP3(f, e, g, h)
#define LOG_MAP8(f, a, b, c, d, e, g, h, i) \
    LOG_MAP4(f, a, b, c, d) LOG_MAP4(f, e, g, h, i)

#define LOG_AT(lvl, tag, ...)                                             \
    do {                                                                  \
        if ((lvl) <= LOG_COMPILE_LEVEL) {                                 \
            if (0)                                                        \
                log_checkFormat(__VA_ARGS__);                             \
            log_write((lvl), (tag), LOG_FIRST(__VA_ARGS__),               \
                      LOG_COUNT(__VA_ARGS__) LOG_CAT(                     \
                          LOG_MAP, LOG_COUNT(__VA_ARGS__))(__VA_ARGS__)); \
        }                                                                 \
    } while (0)

#define LOG_HEX(lvl, tag, label, data, len)                    \
    do {                                                       \
        if ((lvl) <= LOG_COMPILE_LEVEL)                        \
            log_hexdump((lvl), (tag), (label), (data), (len)); \
    } while (0)

#define LOG_TEXT(lvl, tag, text)            \
    do {                                    \
        if ((lvl) <= LOG_COMPILE_LEVEL)     \
            log_text((lvl), (tag), (text)); \
    } while (0)

#else /* CONFIG_LOG */

#define LOG_AT(lvl, tag, ...)             \
    do {                                  \
        if (0)                            \
            log_checkFormat(__VA_ARGS__); \
    } while (0)

#define LOG_HEX(lvl, tag, label, data, len) \
    do {                                    \
        (void)sizeof(data);                 \
        (void)sizeof(len);                  \
    } while (0)

#define LOG_TEXT(lvl, tag, text) \
    do {                         \
        (void)sizeof(text);      \
    } while (0)

#endif /* CONFIG_LOG */

#define LOG_E(tag, ...) LOG_AT(LOG_LVL_ERROR, tag, __VA_ARGS__)
#define LOG_W(tag, ...) LOG_AT(LOG_LVL_WARN, tag, __VA_ARGS__)
#define LOG_I(tag, ...) LOG_AT(LOG_LVL_INFO, tag, __VA_ARGS__)
#define LOG_D(tag, ...) LOG_AT(LOG_LVL_DEBUG, tag, __VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif /* LOG_H */
