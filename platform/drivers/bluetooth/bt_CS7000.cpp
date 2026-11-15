/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "peripherals/gpio.h"
#include "interfaces/delays.h"
#include "core/log.h"
#include "hwconfig.h"
#include "rcc.h"
#include "drivers/bluetooth/bt_proto_CS7000.h"
#include "drivers/bluetooth/bt_CS7000.h"
#include <pthread.h>
#include <string.h>
#ifdef CONFIG_LOG
#include "usb_cdc.h"
#endif

#define BT_BAUD 38400
#define RX_RING 512 // power of two
#define LINE_MAX 80
#define REPLY_WAIT_MS 1500

enum PowerState {
    PWR_PROBE, // see whether the module is already on
    PWR_PULSE, // pulse VREG_ENABLE
    PWR_START, // AT*I and ATO, as the stock firmware does at boot
    PWR_RUNNING,
    PWR_FAILED
};

static uint8_t rxRing[RX_RING];
static volatile uint32_t rxHead; // interrupt handler
static uint32_t rxTail;          // thread
static volatile uint32_t rxErrors;

static volatile bool present;
static struct btLink bt;
static volatile bool linkConnected;
static volatile bool linkAudio;
static volatile bool linkPtt;

static char line[LINE_MAX];
static size_t lineLen;
static bool lineOverflow;
static long long lastLineTime; // last complete line from the module

extern "C" void USART1_IRQHandler()
{
    uint32_t isr = USART1->ISR;

    if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) {
        USART1->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF
                    | USART_ICR_PECF;
        rxErrors = rxErrors + 1;
    }

    while (USART1->ISR & USART_ISR_RXNE_RXFNE) {
        uint8_t c = (uint8_t)USART1->RDR;
        uint32_t head = rxHead;
        if ((head - rxTail) < RX_RING) {
            rxRing[head & (RX_RING - 1)] = c;
            rxHead = head + 1;
        }
    }
}

static void uartInit()
{
    gpio_setMode(BLTH_RXD, ALTERNATE | ALTERNATE_FUNC(7)); // PA9, USART1 TX
    gpio_setMode(BLTH_TXD, ALTERNATE | ALTERNATE_FUNC(7)); // PA10, USART1 RX

    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    __DSB();

    uint32_t quot = rcc_getPeriphClock(USART1);
    quot = (2 * quot) / BT_BAUD;

    USART1->CR1 = 0;
    USART1->BRR = (quot / 2) + (quot & 1);
    USART1->CR3 = USART_CR3_ONEBIT;
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE_RXFNEIE
                | USART_CR1_UE;

    NVIC_ClearPendingIRQ(USART1_IRQn);
    NVIC_SetPriority(USART1_IRQn, 14);
    NVIC_EnableIRQ(USART1_IRQn);
}

static void uartWrite(const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t *)data;

    for (size_t i = 0; i < len; i++) {
        while ((USART1->ISR & USART_ISR_TXE_TXFNF) == 0)
            ;
        USART1->TDR = p[i];
    }
}

static void sendCommand(const char *cmd, const char *arg)
{
    char buf[64];
    size_t n = btProto_command(buf, sizeof(buf), cmd, arg);
    if (n == 0)
        return;

    uartWrite(buf, n);
    LOG_I("bt", "> %s", cmd);
    if (arg != NULL)
        LOG_TEXT(LOG_LVL_INFO, "bt", arg);
}

// Log a line from the module in LOG_MAX_BYTES pieces
static void logLine(const char *text, size_t len)
{
    char piece[LOG_MAX_BYTES + 1];

    for (size_t off = 0; off < len; off += LOG_MAX_BYTES) {
        size_t n = len - off;
        if (n > LOG_MAX_BYTES)
            n = LOG_MAX_BYTES;
        memcpy(piece, &text[off], n);
        piece[n] = '\0';
        LOG_TEXT(LOG_LVL_INFO, "bt", piece);
    }
}

static long long infoDue; // time to send a delayed AT*I, 0 = none

static void handleLine(const char *text, size_t len)
{
    lastLineTime = getTick();
    logLine(text, len);

    bool wasConnected = bt.connected;
    bool wasAudio = bt.audio;
    bool wasPtt = bt.ptt;

    switch (btProto_line(&bt, text, len)) {
        case BT_ACT_INFO:
            sendCommand("AT*I", NULL);
            break;

        case BT_ACT_INFO_LATER:
            infoDue = getTick() + 500;
            break;

        case BT_ACT_CONNECT:
            sendCommand("AT*C", bt.addr);
            break;

        case BT_ACT_AUDIO:
            sendCommand("AT*A", NULL);
            break;

        default:
            break;
    }

    if (bt.connected != wasConnected)
        LOG_I("bt", "headset %s", bt.connected ? "connected" : "gone");
    if (bt.audio != wasAudio)
        LOG_I("bt", "audio link %s", bt.audio ? "open" : "closed");
    if (bt.ptt != wasPtt)
        LOG_I("bt", "PTT %s (not keying yet)", bt.ptt ? "pressed" : "released");

    linkConnected = bt.connected;
    linkAudio = bt.audio;
    linkPtt = bt.ptt;
}

// Bytes from the module: to the USB bridge, and into lines
static void receive()
{
    uint8_t chunk[64];
    size_t n = 0;

    while ((rxTail != rxHead) && (n < sizeof(chunk))) {
        chunk[n++] = rxRing[rxTail & (RX_RING - 1)];
        rxTail++;
    }

    if (n == 0)
        return;

#ifdef CONFIG_LOG
    if (usbbt_connected())
        usb_bt_chardev.api->write(&usb_bt_chardev, chunk, n);
#endif

    for (size_t i = 0; i < n; i++) {
        char c = (char)chunk[i];

        if ((c == '\r') || (c == '\n')) {
            if ((lineLen > 0) && !lineOverflow)
                handleLine(line, lineLen);
            else if (lineOverflow)
                LOG_W("bt", "line over %d bytes dropped", LINE_MAX);
            lineLen = 0;
            lineOverflow = false;
            continue;
        }

        // NULs pad the module's output the same way as the commands
        if (c == '\0')
            continue;

        if (lineLen < sizeof(line))
            line[lineLen++] = c;
        else
            lineOverflow = true;
    }
}

#ifdef CONFIG_LOG
// Bytes from the computer go to the module as they are
static void bridge()
{
    uint8_t buf[64];
    ssize_t n = usb_bt_chardev.api->read(&usb_bt_chardev, buf, sizeof(buf));
    if (n > 0)
        uartWrite(buf, (size_t)n);
}
#endif

// VREG_ENABLE works as a power button: low, high for 500 ms, low
static void powerPulse()
{
    gpioDev_clear(BLTH_PWR_EN);
    sleepFor(0, 200);
    gpioDev_set(BLTH_PWR_EN);
    sleepFor(0, 500);
    gpioDev_clear(BLTH_PWR_EN);
}

/*
 * Wait up to REPLY_WAIT_MS for a line from the module.  Single bytes do not
 * count: an unpowered module holds its UART_TX low, which reads as NULs with
 * framing errors.
 */
static bool waitReply()
{
    long long start = getTick();

    while ((getTick() - start) < REPLY_WAIT_MS) {
        receive();
        if (lastLineTime >= start)
            return true;
        sleepFor(0, 10);
    }

    return false;
}

static void *btThread(void *arg)
{
    (void)arg;

    PowerState state = PWR_PROBE;
    int pulses = 0;
    uint32_t errorsLogged = 0;

    while (true) {
        switch (state) {
            case PWR_PROBE:
                // A software reset leaves the module on; a pulse would then
                // turn it off.  Ask first.
                sendCommand("AT*I", NULL);
                state = waitReply() ? PWR_START : PWR_PULSE;
                if (state == PWR_START)
                    LOG_I("bt", "module already on");
                break;

            case PWR_PULSE:
                if (pulses >= 2) {
                    LOG_E("bt", "module does not answer");
                    state = PWR_FAILED;
                    break;
                }
                LOG_I("bt", "power pulse");
                powerPulse();
                pulses++;
                sleepFor(0, 500);
                sendCommand("AT*I", NULL);
                state = waitReply() ? PWR_START : PWR_PULSE;
                break;

            case PWR_START:
                sleepFor(0, 500);
                sendCommand("ATO", NULL);
                state = PWR_RUNNING;
                break;

            default:
                break;
        }

        receive();
#ifdef CONFIG_LOG
        bridge();
#endif

        if ((infoDue != 0) && (getTick() >= infoDue)) {
            infoDue = 0;
            sendCommand("AT*I", NULL);
        }

        if (rxErrors != errorsLogged) {
            errorsLogged = rxErrors;
            LOG_W("bt", "UART errors: %lu", (unsigned long)errorsLogged);
        }

        sleepFor(0, 10);
    }

    return NULL;
}

void btCs7000_init()
{
    gpio_setMode(BLTH_DETECT, INPUT_PULL_DOWN);
    delayMs(1);
    present = (gpio_readPin(BLTH_DETECT) != 0);

    LOG_I("bt", "module %s", present ? "fitted" : "not fitted");
    if (!present)
        return;

    btProto_reset(&bt);
    uartInit();

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, 2048);
    pthread_t thread;
    pthread_create(&thread, &attr, btThread, NULL);
}

bool btCs7000_present()
{
    return present;
}

bool btCs7000_connected()
{
    return linkConnected;
}

bool btCs7000_audio()
{
    return linkAudio;
}

bool btCs7000_ptt()
{
    return linkPtt;
}
