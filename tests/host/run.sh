#!/bin/sh
#
# SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Host tests for the USB system log: log core + rtxlink framing (decoded by
# scripts/rtxlink_log.py and compared with a golden output) and the CS7000-PLUS
# USB CDC driver's request handling against a register model.
# Run from the repository root: sh tests/host/run.sh

set -e
out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT

CFLAGS="-std=gnu11 -Wall -Wextra -Werror -DCONFIG_LOG -Iopenrtx/include"

cc $CFLAGS -o "$out/log_test" tests/host/log_test.c \
    openrtx/src/core/log.c openrtx/src/core/rtxlink.c
"$out/log_test" "$out/stream.bin"

python3 scripts/rtxlink_log.py --input "$out/stream.bin" --color never \
    --delta > "$out/decoded.txt"
diff -u tests/host/log_test.expected "$out/decoded.txt"
echo "rtxlink_log.py: decoded output matches"

c++ -std=gnu++14 -Wall -Wextra -Werror -DSTM32H743xx \
    -include tests/host/shim/host_cmsis.h \
    -Iplatform/mcu/STM32H7xx/drivers -Iopenrtx/include -Iplatform \
    -fpermissive -isystem platform/mcu/CMSIS/Include \
    -isystem platform/mcu/CMSIS/Device/ST/STM32H7xx/Include \
    -o "$out/usb_test" tests/host/usb_cdc_test.cpp
"$out/usb_test"
