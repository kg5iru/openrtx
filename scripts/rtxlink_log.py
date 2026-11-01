#!/usr/bin/env python3
#
# SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Console for the OpenRTX system log sent over USB with rtxlink.

    python3 scripts/rtxlink_log.py                  # find the radio, print
    python3 scripts/rtxlink_log.py --level info     # less chatty
    python3 scripts/rtxlink_log.py --tag dmr --delta --out dmr.log

The radio appears as a USB serial port (VID:PID 0483:5740, product
"OpenRTX rtxlink"; /dev/cu.usbmodem* on macOS). The script waits for it,
reconnects after a reboot, and keeps going until Ctrl-C.

Output, one line per record:

    [   12.345678] I dmr: call start: flco 0 src 3100001 dst 91

time is seconds since the radio booted, then the level (E W I D), the tag and
the message. --delta adds the milliseconds since the previous printed record.

Needs pyserial (pip install pyserial).
"""

import argparse
import os
import re
import sys
import time

SLIP_END = 0xC0
SLIP_ESC = 0xDB
SLIP_ESC_END = 0xDC
SLIP_ESC_ESC = 0xDD

PROTO_STDIO = 0x00
PROTO_LOG = 0x04

LEVELS = {'none': 0, 'error': 1, 'warn': 2, 'info': 3, 'debug': 4}
LEVEL_CHARS = {1: 'E', 2: 'W', 3: 'I', 4: 'D'}
LEVEL_COLORS = {1: '\033[31m', 2: '\033[33m', 3: '', 4: '\033[2m'}
RESET_COLOR = '\033[0m'

USB_VID = 0x0483
USB_PID = 0x5740


def crc16(data, crc=0x1D0F):
    """CRC-16, polynomial 0x1021, initial value 0x1D0F, as rtxlink uses."""
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def encode_frame(proto, payload):
    """Build a complete rtxlink frame, SLIP encoded."""
    body = bytes([proto]) + bytes(payload)
    crc = crc16(body)
    body += bytes([crc >> 8, crc & 0xFF])
    out = bytearray([SLIP_END])
    for b in body:
        if b == SLIP_END:
            out += bytes([SLIP_ESC, SLIP_ESC_END])
        elif b == SLIP_ESC:
            out += bytes([SLIP_ESC, SLIP_ESC_ESC])
        else:
            out.append(b)
    out.append(SLIP_END)
    return bytes(out)


class FrameDecoder:
    """Streaming SLIP decoder with CRC check. feed() yields (proto, payload)."""

    def __init__(self, max_len=1024):
        self.buf = bytearray()
        self.esc = False
        self.overflow = False
        self.max_len = max_len
        self.crc_errors = 0
        self.frames = 0

    def feed(self, data):
        for b in data:
            if b == SLIP_END:
                if self.buf and not self.overflow:
                    frame = self._check(bytes(self.buf))
                    if frame is not None:
                        yield frame
                self.buf.clear()
                self.esc = False
                self.overflow = False
                continue
            if b == SLIP_ESC:
                self.esc = True
                continue
            if self.esc:
                self.esc = False
                if b == SLIP_ESC_END:
                    b = SLIP_END
                elif b == SLIP_ESC_ESC:
                    b = SLIP_ESC
            if len(self.buf) < self.max_len:
                self.buf.append(b)
            else:
                self.overflow = True

    def _check(self, frame):
        if len(frame) < 3:
            self.crc_errors += 1
            return None
        body, tail = frame[:-2], frame[-2:]
        crc = crc16(body)
        if tail not in (bytes([crc >> 8, crc & 0xFF]),
                        bytes([crc & 0xFF, crc >> 8])):
            self.crc_errors += 1
            return None
        self.frames += 1
        return body[0], body[1:]


def parse_record(payload):
    """Decode a LOG 'R' payload into (level, microseconds, tag, text)."""
    if len(payload) < 11 or payload[0] != ord('R'):
        return None
    level = payload[1]
    us = int.from_bytes(payload[2:10], 'little')
    rest = payload[10:]
    nul = rest.find(b'\0')
    if nul < 0:
        return None
    tag = rest[:nul].decode('ascii', 'replace')
    text = rest[nul + 1:].decode('utf-8', 'replace')
    return level, us, tag, text


class Printer:
    def __init__(self, args, out=sys.stdout):
        self.args = args
        self.out = out
        self.color = args.color == 'always' or (
            args.color == 'auto' and out.isatty())
        self.log_file = open(args.out, 'a') if args.out else None
        self.last_us = None
        self.min_level = LEVELS[args.show]
        self.grep = re.compile(args.grep) if args.grep else None

    def note(self, text):
        line = '--- %s ---' % text
        print(line, file=self.out, flush=True)
        if self.log_file:
            print(line, file=self.log_file, flush=True)

    def record(self, level, us, tag, text):
        if self.last_us is not None and us < self.last_us:
            self.note('radio rebooted')
            self.last_us = None
        if level > self.min_level:
            return
        if self.args.tag and tag not in self.args.tag:
            return
        if self.grep and not self.grep.search(text):
            return

        delta = ''
        if self.args.delta:
            if self.last_us is None:
                delta = '          '
            else:
                delta = '%+9.3f ' % ((us - self.last_us) / 1000.0)
        self.last_us = us

        plain = '[%6d.%06d] %s%s %s: %s' % (
            us // 1000000, us % 1000000, delta,
            LEVEL_CHARS.get(level, '?'), tag, text)
        if self.color and LEVEL_COLORS.get(level):
            print(LEVEL_COLORS[level] + plain + RESET_COLOR, file=self.out,
                  flush=True)
        else:
            print(plain, file=self.out, flush=True)
        if self.log_file:
            print(plain, file=self.log_file, flush=True)

    def frame(self, proto, payload):
        if proto == PROTO_LOG and payload[:1] == b'R':
            rec = parse_record(payload)
            if rec is not None:
                self.record(*rec)
                return
        if proto == PROTO_LOG and payload[:1] == b'L' and len(payload) >= 2:
            names = {v: k for k, v in LEVELS.items()}
            self.note('radio log level: %s' % names.get(payload[1],
                                                       payload[1]))
            return
        if proto == PROTO_STDIO:
            sys.stdout.write(payload.decode('utf-8', 'replace'))
            sys.stdout.flush()
            return
        self.note('frame proto 0x%02x: %s' % (proto, payload.hex(' ')))


def find_port():
    from serial.tools import list_ports
    for p in list_ports.comports():
        if p.vid == USB_VID and p.pid == USB_PID:
            return p.device
    return None


def run_serial(args, printer, decoder):
    import serial

    waiting_said = False
    while True:
        port = args.port or find_port()
        if port is None or (args.port and not os.path.exists(args.port)):
            if not waiting_said:
                printer.note('waiting for the radio (USB 0483:5740)')
                waiting_said = True
            time.sleep(0.5)
            continue

        try:
            # Opening the port asserts DTR: that is what tells the radio
            # someone is listening.
            ser = serial.Serial(port, 115200, timeout=0.1)
        except (serial.SerialException, OSError) as e:
            if not waiting_said:
                printer.note('cannot open %s: %s' % (port, e))
                waiting_said = True
            time.sleep(0.5)
            continue

        waiting_said = False
        printer.note('connected to %s' % port)
        try:
            if args.level is not None:
                ser.write(encode_frame(PROTO_LOG, b'L' + bytes([LEVELS[args.level]])))
            while True:
                data = ser.read(4096)
                for proto, payload in decoder.feed(data):
                    printer.frame(proto, payload)
        except (serial.SerialException, OSError):
            printer.note('disconnected')
            if args.once:
                return
        finally:
            ser.close()


def run_file(args, printer, decoder):
    with open(args.input, 'rb') as f:
        data = f.read()
    for proto, payload in decoder.feed(data):
        printer.frame(proto, payload)


def main():
    ap = argparse.ArgumentParser(
        description='Show the OpenRTX system log received over USB (rtxlink).')
    ap.add_argument('--port', help='serial port (default: find the radio by '
                    'its USB VID:PID)')
    ap.add_argument('--level', choices=[k for k in LEVELS if k != 'none'],
                    help='set the radio log level on connect (persists until '
                    'the radio reboots)')
    ap.add_argument('--show', choices=list(LEVELS), default='debug',
                    help='hide records above this level on the host side')
    ap.add_argument('--tag', action='append',
                    help='only show this tag (repeat for more)')
    ap.add_argument('--grep', help='only show messages matching this regex')
    ap.add_argument('--delta', action='store_true',
                    help='show milliseconds since the previous record')
    ap.add_argument('--out', help='also append plain text to this file')
    ap.add_argument('--color', choices=['auto', 'always', 'never'],
                    default='auto')
    ap.add_argument('--once', action='store_true',
                    help='exit when the radio disconnects instead of waiting')
    ap.add_argument('--input', help='decode a raw capture file instead of a '
                    'serial port')
    args = ap.parse_args()

    printer = Printer(args)
    decoder = FrameDecoder()
    try:
        if args.input:
            run_file(args, printer, decoder)
        else:
            try:
                import serial  # noqa: F401
            except ImportError:
                sys.exit('pyserial is missing: pip install pyserial')
            run_serial(args, printer, decoder)
    except KeyboardInterrupt:
        pass
    finally:
        if decoder.crc_errors:
            print('%d frames with a bad CRC were dropped' % decoder.crc_errors,
                  file=sys.stderr)


if __name__ == '__main__':
    main()
