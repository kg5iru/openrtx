/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef VP_USE_FILESYSTEM
.section .note.GNU-stack,""
.section .rodata

.global _vpdata_start
.global _vpdata_end

_vpdata_start:
#ifndef VP_NO_DATA
    .incbin "../voiceprompts.vpc"
#endif

.set _vpdata_end, .
#endif
