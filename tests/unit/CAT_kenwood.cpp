/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <catch2/catch_test_macros.hpp>

#include <cstring>
#include <string>

#include "core/cat.h"

// A radio in a few variables
static freq_t rxF = 430950000, txF = 430950000;
static bool ptt = false;
static rssi_t level = -127;
static bool sql = false;

static freq_t getRx()
{
    return rxF;
}

static freq_t getTx()
{
    return txF;
}

static bool okFreq(freq_t f)
{
    return (f >= 400000000) && (f <= 480000000);
}

static bool setRx(freq_t f)
{
    if (!okFreq(f))
        return false;
    txF = f + (txF - rxF);
    rxF = f;
    return true;
}

static bool setTx(freq_t f)
{
    if (!okFreq(f))
        return false;
    txF = f;
    return true;
}

static bool isTx()
{
    return ptt;
}

static void setPtt(bool on)
{
    ptt = on;
}

static rssi_t getRssi()
{
    return level;
}

static bool isBusy()
{
    return sql;
}

static const struct catRadio radio = { getRx, getTx,  setRx,   setTx,
                                       isTx,  setPtt, getRssi, isBusy };

static std::string cat(const char *cmd)
{
    char reply[48];
    size_t n = cat_execute(&radio, cmd, strlen(cmd), reply, sizeof(reply));
    return std::string(reply, n);
}

TEST_CASE("CAT: identification and fixed answers", "[cat]")
{
    REQUIRE(cat("ID") == "ID019;");
    REQUIRE(cat("AI") == "AI0;");
    REQUIRE(cat("AI2") == "");
    REQUIRE(cat("PS") == "PS1;");
    REQUIRE(cat("PS1") == "");
    REQUIRE(cat("FR") == "FR0;");
    REQUIRE(cat("FT") == "FT0;");
    REQUIRE(cat("MD") == "MD4;");
    REQUIRE(cat("MD2") == "");
    REQUIRE(cat("id") == "ID019;");
    REQUIRE(cat("XX") == "?;");
    REQUIRE(cat("E") == "?;");
    REQUIRE(cat("EX0310000") == "?;");
}

TEST_CASE("CAT: frequencies", "[cat]")
{
    rxF = 430950000;
    txF = 430950000;
    REQUIRE(cat("FA") == "FA00430950000;");
    REQUIRE(cat("FA00445950000") == "");
    REQUIRE(rxF == 445950000);
    REQUIRE(txF == 445950000);
    REQUIRE(cat("FB00440950000") == "");
    REQUIRE(txF == 440950000);
    REQUIRE(cat("FB") == "FB00440950000;");
    // Setting FA keeps the shift
    REQUIRE(cat("FA00446000000") == "");
    REQUIRE(txF == 441000000);
    // Out of band, malformed: refused, nothing changes
    REQUIRE(cat("FA00146520000") == "?;");
    REQUIRE(cat("FA0044595000x") == "?;");
    REQUIRE(cat("FA004459500001") == "?;");
    REQUIRE(rxF == 446000000);
}

TEST_CASE("CAT: status and PTT", "[cat]")
{
    rxF = 445950000;
    txF = 445950000;
    ptt = false;
    std::string s = cat("IF");
    REQUIRE(s.size() == 38);
    REQUIRE(s.substr(0, 13) == "IF00445950000");
    REQUIRE(s[28] == '0'); // receiving
    REQUIRE(s[29] == '4'); // FM
    REQUIRE(s[30] == '0'); // VFO A
    REQUIRE(s[32] == '0'); // no split
    REQUIRE(s[36] == '0'); // simplex
    REQUIRE(s[37] == ';');

    REQUIRE(cat("TX") == "");
    REQUIRE(ptt);
    s = cat("IF");
    REQUIRE(s[28] == '1');
    REQUIRE(cat("RX") == "");
    REQUIRE(!ptt);
    REQUIRE(cat("TX1") == "");
    REQUIRE(ptt);
    REQUIRE(cat("RX") == "");

    txF = 440950000; // minus shift
    REQUIRE(cat("IF")[36] == '2');
    txF = 450950000;
    REQUIRE(cat("IF")[36] == '1');
}

TEST_CASE("CAT: S-meter and busy", "[cat]")
{
    REQUIRE(cat_smeter(-150) == 0);
    REQUIRE(cat_smeter(-93) == 15); // S9
    REQUIRE(cat_smeter(-120) == 7); // about S4.5
    REQUIRE(cat_smeter(-53) == 25); // S9+40
    REQUIRE(cat_smeter(-20) == 30);
    level = -93;
    REQUIRE(cat("SM0") == "SM00015;");
    REQUIRE(cat("SM") == "SM00015;");
    sql = true;
    REQUIRE(cat("BY") == "BY10;");
    sql = false;
    REQUIRE(cat("BY") == "BY00;");
}

// A serial port in two strings
static std::string portIn, portOut;

static int portInit(const struct chardev *)
{
    return 0;
}

static int portTerminate(const struct chardev *)
{
    return 0;
}

static ssize_t portRead(const struct chardev *, void *data, const size_t len)
{
    size_t n = (portIn.size() < len) ? portIn.size() : len;
    memcpy(data, portIn.data(), n);
    portIn.erase(0, n);
    return n;
}

static ssize_t portWrite(const struct chardev *, const void *data,
                         const size_t len)
{
    portOut.append((const char *)data, len);
    return len;
}

static int portIoctl(const struct chardev *, const int, const void *)
{
    return 0;
}

static const struct chardev_api portApi = { portInit, portTerminate, portRead,
                                            portWrite, portIoctl };
static const struct chardev port = { NULL, NULL, &portApi };

TEST_CASE("CAT: the serial side", "[cat]")
{
    rxF = 430950000;
    txF = 430950000;
    ptt = false;
    REQUIRE(cat_init(&port, &radio) == 0);

    // Several commands at once, split anywhere, with line ends around
    portIn = "ID;FA;\r\nF";
    cat_task(0);
    REQUIRE(portOut == "ID019;FA00430950000;");
    portOut.clear();
    portIn = "A00445950000;FA;";
    cat_task(10);
    REQUIRE(portOut == "FA00445950000;");
    portOut.clear();

    // A set command followed by the check Hamlib sends
    portIn = "FA00446000000;ID;";
    cat_task(20);
    REQUIRE(portOut == "ID019;");
    REQUIRE(rxF == 446000000);
    portOut.clear();

    // An overlong line is dropped whole, what follows still works
    portIn = std::string(100, 'Z') + ";ID;";
    cat_task(30);
    REQUIRE(portOut == "ID019;");
    portOut.clear();

    // A transmit request ends after the time-out
    portIn = "TX;";
    cat_task(1000);
    REQUIRE(ptt);
    cat_task(1000 + (CAT_TX_TIMEOUT_S * 1000LL) - 10);
    REQUIRE(ptt);
    cat_task(1000 + (CAT_TX_TIMEOUT_S * 1000LL) + 10);
    REQUIRE(!ptt);

    // A new request restarts the time
    portIn = "TX;";
    cat_task(2000000);
    REQUIRE(ptt);
    portIn = "RX;";
    cat_task(2000100);
    REQUIRE(!ptt);
}
