/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <catch2/catch_test_macros.hpp>

#include <cstring>
#include <string>

#include "drivers/bluetooth/bt_proto_CS7000.h"

static enum btAction feed(struct btLink *link, const char *text)
{
    return btProto_line(link, text, strlen(text));
}

TEST_CASE("CS7000 Bluetooth commands are framed like the stock firmware",
          "[bt]")
{
    char buf[64];

    size_t n = btProto_command(buf, sizeof(buf), "AT*I", NULL);
    REQUIRE(n == 30 + 4 + 1);
    for (int i = 0; i < 30; i++)
        REQUIRE(buf[i] == '\0');
    REQUIRE(std::string(&buf[30], 4) == "AT*I");
    REQUIRE(buf[34] == '\r');

    n = btProto_command(buf, sizeof(buf), "AT*C", "0123456789AB");
    REQUIRE(n == 30 + 4 + 12 + 1);
    REQUIRE(std::string(&buf[30], 16) == "AT*C0123456789AB");
    REQUIRE(buf[46] == '\r');

    // Too long for the buffer: nothing written
    REQUIRE(btProto_command(buf, 40, "AT*C", "0123456789AB") == 0);
}

TEST_CASE("CS7000 Bluetooth link follows the module's status lines", "[bt]")
{
    struct btLink link;
    btProto_reset(&link);

    REQUIRE(feed(&link, "I*IF") == BT_ACT_INFO);

    // A paired device is reported: connect to it
    REQUIRE(feed(&link, "I*PD0123456789ABAirPods") == BT_ACT_CONNECT);
    REQUIRE(std::string(link.addr) == "0123456789AB");

    // Too short to carry an address: ignored
    REQUIRE(feed(&link, "I*PD0123") == BT_ACT_NONE);

    // Connected: open the audio, no more connects
    REQUIRE(feed(&link, "I*CHC00123456789AB") == BT_ACT_AUDIO);
    REQUIRE(link.connected);
    REQUIRE(feed(&link, "I*PD0123456789ABAirPods") == BT_ACT_NONE);

    REQUIRE(feed(&link, "I*AHC0") == BT_ACT_NONE);
    REQUIRE(link.audio);
    REQUIRE(feed(&link, "I*AHD0") == BT_ACT_NONE);
    REQUIRE_FALSE(link.audio);

    // A failed attempt asks again later; a disconnect clears everything
    REQUIRE(feed(&link, "I*CHD1") == BT_ACT_INFO_LATER);
    feed(&link, "I*AHC0");
    feed(&link, "I*BH01");
    REQUIRE(feed(&link, "I*CHD0") == BT_ACT_NONE);
    REQUIRE_FALSE(link.connected);
    REQUIRE_FALSE(link.audio);
    REQUIRE_FALSE(link.ptt);

    // Unknown lines change nothing
    REQUIRE(feed(&link, "C:OK") == BT_ACT_NONE);
    REQUIRE(feed(&link, "") == BT_ACT_NONE);
}

TEST_CASE("CS7000 Bluetooth push-to-talk in every accessory form", "[bt]")
{
    struct btLink link;
    btProto_reset(&link);

    const char *presses[] = { "I*BH01", "C:BEGIN*", "AT+CTKST=1", "+PTT=P",
                              "#p" };
    const char *releases[] = { "I*BH00", "C:END*", "AT+CTKST=0", "+PTT=R",
                               "#r" };

    for (int i = 0; i < 5; i++) {
        feed(&link, presses[i]);
        REQUIRE(link.ptt);
        feed(&link, releases[i]);
        REQUIRE_FALSE(link.ptt);
    }

    // I*E0 toggles
    feed(&link, "I*E0");
    REQUIRE(link.ptt);
    feed(&link, "I*E0");
    REQUIRE_FALSE(link.ptt);
}
