/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <cstring>
#include "protocols/DMR/SlotType.hpp"
#include "protocols/DMR/Emb.hpp"
#include "protocols/DMR/FullLinkControl.hpp"
#include "protocols/DMR/SingleBlockPdu.hpp"
#include "protocols/DMR/Constants.hpp"

using namespace DMR;

TEST_CASE("SlotType round-trips every colour code / data type combination", "[dmr][frames]")
{
    for (uint8_t cc = 0; cc < 16; cc++)
    {
        for (uint8_t dt = 0; dt < 16; dt++)
        {
            SlotType st{cc, (DataType) dt};
            uint8_t bits[20];
            st.encode(bits);

            SlotType decoded;
            bool ok = SlotType::decode(bits, decoded);

            REQUIRE(ok);
            REQUIRE(decoded.colorCode == cc);
            REQUIRE((uint8_t) decoded.dataType == dt);
        }
    }
}

TEST_CASE("Emb round-trips colour code / PI / LCSS", "[dmr][frames]")
{
    for (uint8_t cc = 0; cc < 16; cc++)
    {
        for (uint8_t lcss = 0; lcss < 4; lcss++)
        {
            Emb e{cc, true, lcss};
            uint8_t bits[16];
            e.encode(bits);

            Emb decoded;
            bool ok = Emb::decode(bits, decoded);

            REQUIRE(ok);
            REQUIRE(decoded.colorCode == cc);
            REQUIRE(decoded.pi == true);
            REQUIRE(decoded.lcss == lcss);
        }
    }
}

TEST_CASE("FullLinkControl round-trips and rejects the wrong mask", "[dmr][frames]")
{
    FullLinkControl lc;
    lc.protectFlag = false;
    lc.flco = 0x03;
    lc.featureSetId = 0x00;
    for (int i = 0; i < 7; i++)
        lc.payload[i] = (uint8_t)(i * 7 + 1);

    uint8_t block[12];
    lc.encode(block, CrcMask::VoiceLcHeader);

    FullLinkControl decoded;
    bool ok = FullLinkControl::decode(block, CrcMask::VoiceLcHeader, decoded);

    REQUIRE(ok);
    REQUIRE(decoded.flco == lc.flco);
    REQUIRE(decoded.featureSetId == lc.featureSetId);
    REQUIRE(memcmp(decoded.payload, lc.payload, 7) == 0);

    FullLinkControl shouldFail;
    REQUIRE_FALSE(FullLinkControl::decode(block, CrcMask::TerminatorWithLc, shouldFail));
}

TEST_CASE("SingleBlockPdu round-trips and detects corruption", "[dmr][frames]")
{
    SingleBlockPdu pdu;
    for (int i = 0; i < 10; i++)
        pdu.content[i] = (uint8_t)(0x30 + i);

    uint8_t block[12];
    pdu.encode(block, CrcMask::Csbk);

    SingleBlockPdu decoded;
    bool ok = SingleBlockPdu::decode(block, CrcMask::Csbk, decoded);

    REQUIRE(ok);
    REQUIRE(memcmp(decoded.content, pdu.content, 10) == 0);

    block[2] ^= 0xFF;
    SingleBlockPdu shouldFail;
    REQUIRE_FALSE(SingleBlockPdu::decode(block, CrcMask::Csbk, shouldFail));
}
