#!/usr/bin/env python
'''
*******************************************************
 Copyright (c) MLRS project
 GPL3
 https://www.gnu.org/licenses/gpl-3.0.de.html
*******************************************************
 run_rx_ota.py
 updates a receiver over the air, via the CLI of the tx module, see --help
 port can also be the usb serial (VCP) of an EdgeTX radio, for an internal tx module, it is found by itself
 The receiver pulls the image, see mLRS/Common/ota/ota_protocol.h. This tool only answers its requests.
********************************************************
'''
import argparse
import re
import gzip
import struct
import sys
import time
import zlib

import serial # pyserial
import serial.tools.list_ports


# must match Common/ota/ota_protocol.h and CommonTx/ota_relay_tx.h
OTA_PROTOCOL_VERSION = 1
OTA_APP_INFO_MAGIC = struct.pack('<I', 0x4F4C524D)
OTA_CMD_INFO, OTA_CMD_DATA, OTA_CMD_RESULT = 1, 2, 3
OTA_CMD_RESPONSE = 0x80
OTA_STATUS_OK, OTA_STATUS_ERR_TARGET, OTA_STATUS_ERR_LENGTH, OTA_STATUS_ERR_VERSION = 0, 1, 2, 3
OTA_STATUS = ['ok', 'wrong target', 'bad length', 'wrong protocol version', 'flash error', 'image check failed', 'not supported', 'bad data']
OTA_FLAG_DEFLATE = 0x01
OTA_FLAG_GZIP = 0x02
OTA_RELAY_STX = 0xA5
ESP_IMAGE_MAGIC = 0xE9

# internal tx module via the EdgeTX serial passthrough
MBRIDGE_CMD_RX_OTA_JRPIN5 = 20
JRPIN5_BAUDS = [400000, 921600, 1870000] # = crsf_baud[] of the tx, it autobauds
OTA_RELAY_JRPIN5_BAUDRATE = 115200 # the module goes to it when its relay starts

RESULT_LINGER_S = 0.5 # keep answering, the receiver repeats its result if our answer got lost


def _crc32_mpeg2_table():
    table = []
    for i in range(256):
        c = i << 24
        for j in range(8): c = ((c << 1) ^ 0x04C11DB7) if (c & 0x80000000) else (c << 1)
        table.append(c & 0xFFFFFFFF)
    return table

CRC32_MPEG2_TABLE = _crc32_mpeg2_table()


def crc32_mpeg2(data, crc=0xFFFFFFFF): # CRC-32/MPEG-2, = ota_crc() of the receiver
    for b in data: crc = ((crc << 8) & 0xFFFFFFFF) ^ CRC32_MPEG2_TABLE[((crc >> 24) ^ b) & 0xFF]
    return crc


# the image which goes to the receiver
# ESP: the .bin of the build
# STM32: the .hex of the build has the OTA loader and the app, the app goes, it's the part with the app info
def load_image(path):
    with open(path, 'rb') as F: content = F.read()
    if not path.lower().endswith('.hex'): return content
    segments, base = [], 0 # [address, bytearray], the small gaps between sections are filled
    for line in content.decode(errors='replace').split():
        if line[:1] != ':': continue
        rec = bytes.fromhex(line[1:])
        n, adr, typ, data = rec[0], (rec[1] << 8) | rec[2], rec[3], rec[4:4 + rec[0]]
        if typ == 4: base = ((data[0] << 8) | data[1]) << 16
        if typ != 0: continue
        adr += base
        end = segments[-1][0] + len(segments[-1][1]) if segments else 0
        if segments and 0 <= adr - end < 256:
            segments[-1][1] += b'\xFF' * (adr - end) + data
        else:
            segments.append([adr, bytearray(data)])
    for adr, data in segments:
        if OTA_APP_INFO_MAGIC in data: return bytes(data)
    return b''


# the receiver's device name, and its firmware version, from the app info in the image
def image_info(image):
    is_esp = len(image) >= 16 and image[0] == ESP_IMAGE_MAGIC
    is_stm32 = len(image) >= 16 and (struct.unpack_from('<I', image)[0] & 0xFFF00000) == 0x20000000 # stack pointer
    if not is_esp and not is_stm32: return None
    pos = image.find(OTA_APP_INFO_MAGIC)
    if pos < 0 or image.find(OTA_APP_INFO_MAGIC, pos + 4) >= 0: return None
    magic, version, name = struct.unpack_from('<II21s', image, pos)
    return name.split(b'\0')[0].decode(errors='replace'), version


def crc16(data): # CRC-16/MCRF4XX (MAVLink X.25), = fmav_crc_calculate()
    crc = 0xFFFF
    for b in data:
        tmp = (b ^ (crc & 0xFF)) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def crc8_crsf(data): # CRC-8/DVB-S2, poly 0xD5
    crc = 0
    for b in data:
        crc ^= b
        for i in range(8): crc = ((crc << 1) ^ 0xD5) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def crsf_frame(body): # to the tx module
    return bytes([0xEE, len(body) + 1]) + body + bytes([crc8_crsf(body)])


# the module goes on only after channel frames, all mid but throttle (ch3, AETR) at -100%
CRSF_RC_FRAME = crsf_frame(bytes([0x16]) + sum(((992 if i != 2 else 173) << (11 * i) for i in range(16))).to_bytes(22, 'little'))
CRSF_RX_OTA_FRAME = crsf_frame(bytes([0x81, 0x4F, 0x57, 0xA0 + MBRIDGE_CMD_RX_OTA_JRPIN5])) # 'O', 'W', cmd, as the lua scripts


# makes the radio pass its usb serial through to the internal tx module
# stopping the pulses powers the module off, so it is powered on again, and boots into its firmware
def edgetx_passthrough(ser):
    for i in range(2):
        ser.write(b'\r\n')
        time.sleep(1.0)
        if b'>' in ser.read(ser.in_waiting): break
    else: fail(None, 'no answer on ' + ser.port + ', neither from a tx CLI nor a radio CLI')
    for cmd in (b'set pulses 0', b'set rfmod 0 bootpin 0', b'set rfmod 0 power on', b'serialpassthrough rfmod 0 %d' % JRPIN5_BAUDS[0]):
        ser.write(cmd + b'\n')
        time.sleep(0.5)
        print('radio:', cmd.decode(), '->', ser.read(ser.in_waiting).decode(errors='replace').strip().replace('\r\n', ' | '))


# plays the radio: sends channel frames, and looks at the link statistics the module answers with
# the module autobauds and stays at one of its rates, so we have to find it, the radio follows our rate
# returns true when the module reports a link to the receiver
def crsf_wait_connected(ser, timeout=25.0):
    found = False # the module's baudrate is found
    baud_i = 0
    tbaud = 0
    tend = time.time() + timeout
    while time.time() < tend:
        if not found and time.time() - tbaud > 0.5:
            ser.baudrate = JRPIN5_BAUDS[baud_i]
            baud_i = (baud_i + 1) % len(JRPIN5_BAUDS)
            tbaud = time.time()
            ser.reset_input_buffer()
            buf = b''
        ser.write(CRSF_RC_FRAME)
        time.sleep(0.004)
        buf = (buf + ser.read(ser.in_waiting))[-128:]
        for i in range(len(buf) - 3): # frames from the module with valid crc
            f = buf[i:i + 2 + buf[i + 1]]
            if f[0] != 0xEA or len(f) < 4 or len(f) != 2 + f[1] or crc8_crsf(f[2:-1]) != f[-1]: continue
            if not found: print('module talks CRSF at %d baud' % ser.baudrate); found = True
            if f[2] == 0x14 and len(f) == 14 and f[5] > 0: return True # link statistics, uplink lq
    if not found: print('WARNING: nothing heard from the module')
    return False


class cRelay:
    def __init__(self, ser):
        self.ser = ser
        self.buf = b''

    def send(self, packet):
        body = bytes([len(packet)]) + packet
        self.ser.write(bytes([OTA_RELAY_STX]) + body + struct.pack('<H', crc16(body)))

    def answer(self, cmd, payload=b''):
        self.send(bytes([cmd | OTA_CMD_RESPONSE, 0, 0]) + payload) # session id is filled in by the tx

    # returns the next packet from the receiver, None if none came in time
    def receive(self, timeout=0.5):
        tend = time.time() + timeout
        while True:
            # find a frame in what we have
            while len(self.buf) >= 4:
                if self.buf[0] != OTA_RELAY_STX: self.buf = self.buf[1:]; continue
                n = self.buf[1]
                if len(self.buf) < n + 4: break
                frame, self.buf = self.buf[:n + 4], self.buf[n + 4:]
                if crc16(frame[1:n + 2]) == struct.unpack('<H', frame[n + 2:])[0]: return frame[2:n + 2]
            if time.time() > tend: return None
            self.buf += self.ser.read(max(1, self.ser.in_waiting)) # timeout is fixed, setting it reconfigures the port

    def end(self):
        self.send(b'')


def fail(relay, txt):
    print('ERROR:', txt)
    if relay: relay.end()
    sys.exit(1)


def status_str(status):
    return OTA_STATUS[status] if status < len(OTA_STATUS) else str(status)


def main():
    parser = argparse.ArgumentParser(description='updates a receiver over the air, via the CLI of the tx module')
    parser.add_argument('port', help='com port of the tx module, or the usb serial of the radio for an internal one')
    parser.add_argument('firmware', help='the .bin of the build, the .hex for STM32')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--timeout', type=float, default=300.0, help='seconds the update may take')
    parser.add_argument('--no-cli', action='store_true', help="don't send the rxota CLI command, the tx is in relay mode already")
    args = parser.parse_args()
    port, baud, timeout, no_cli = args.port, args.baud, args.timeout, args.no_cli

    image = load_image(args.firmware)

    # the app info sits somewhere in the image
    info = image_info(image)
    if not info: fail(None, 'not an OTA image')
    name, version = info
    print('image: %s, version %d, length %d' % (name, version, len(image)))

    # the usb-uart adapter of an ESP32 tx may reset it or hold it in boot with DTR/RTS set, a usb com port may need them
    is_usb_com = any(p.device == port and p.vid == 0x0483 for p in serial.tools.list_ports.comports())
    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = port, baud, 0.5
    if not is_usb_com: ser.dtr, ser.rts = False, False
    ser.open()
    relay = cRelay(ser)

    radio = False # port is the usb serial of an EdgeTX radio, the tx module is an internal one
    if not no_cli:
        # the receiver must be connected to get told to go into ota
        # a receiver which is in its ota update already doesn't connect, so go on in any case
        print('waiting for the receiver to be connected...')
        cli_ok = False
        for i in range(15):
            ser.reset_input_buffer()
            ser.write(b'v;')
            time.sleep(1.0)
            res = ser.read(ser.in_waiting)
            if b'Tx: ' in res: cli_ok = True
            rx = re.search(rb'Rx: ([^,\r\n]+), ([^\r\n]+)', res)
            if rx or (not cli_ok and i == 1): break # the tx CLI answers within 1 s, else it's a radio
        radio = not cli_ok
        if radio:
            print('no tx CLI, trying the radio for an internal tx module')
            edgetx_passthrough(ser)
            connected = crsf_wait_connected(ser)
        else:
            connected = rx is not None
            if connected:
                rx_name = rx.group(1).decode(errors='replace').strip()
                print('receiver: %s, %s' % (rx_name, rx.group(2).decode(errors='replace').strip()))
                if rx_name != name[:20]: fail(None, 'image is for %s, not for this receiver' % name)
        if not connected: print('WARNING: receiver not connected, it gets the update only if it is in it already')
        if radio:
            # the mBridge command starts the relay, which then has the JR pin5 uart as plain serial
            for i in range(3): ser.write(CRSF_RX_OTA_FRAME); time.sleep(0.004) # a lost frame doesn't matter then
            time.sleep(1.5) # relay starts after 1 s
            ser.baudrate = OTA_RELAY_JRPIN5_BAUDRATE
        else:
            ser.reset_input_buffer()
            ser.write(b'rxota;')
            time.sleep(1.5) # relay starts after 1 s
            if b'relay starts' not in ser.read(ser.in_waiting): fail(None, 'tx did not take the rxota command')
    ser.timeout = 0.01
    ser.reset_input_buffer()

    print('waiting for the receiver...')
    data = None # what is sent, set when the receiver has told what it can take
    requests = repeats = 0
    last_offset = -1
    tstart = tlast = time.time()
    tresult = None
    while True:
        if tresult is not None and time.time() - tresult > RESULT_LINGER_S: break
        if time.time() - tstart > timeout: fail(relay, 'timeout')
        if time.time() - tlast > 10.0: fail(relay, 'receiver went silent' if data else 'no receiver')
        packet = relay.receive()
        if not packet or len(packet) < 3: continue
        tlast = time.time()
        cmd, payload = packet[0], packet[3:]

        if cmd == OTA_CMD_INFO and len(payload) >= 1:
            status = OTA_STATUS_OK
            if payload[0] != OTA_PROTOCOL_VERSION or len(payload) != 26:
                status = OTA_STATUS_ERR_VERSION
            else:
                rx_version, rx_space, rx_flags, rx_name = struct.unpack('<BIB20s', payload)
                rx_name = rx_name.split(b'\0')[0].decode(errors='replace')
                if data is None: print('receiver in its update: %s' % rx_name)
                if rx_name != name[:20]: status = OTA_STATUS_ERR_TARGET
                elif len(image) > rx_space: status = OTA_STATUS_ERR_LENGTH
            if status != OTA_STATUS_OK:
                relay.answer(OTA_CMD_INFO, bytes([status]) + bytes(13))
                fail(relay, status_str(status))
            if data is None:
                data, flags = image, 0
                if rx_flags & OTA_FLAG_DEFLATE: # ESP32
                    data, flags = zlib.compress(image, 9), OTA_FLAG_DEFLATE
                elif rx_flags & OTA_FLAG_GZIP: # ESP8266
                    data, flags = gzip.compress(image, 9, mtime=0), OTA_FLAG_GZIP
                if flags: print('compressed to %d bytes, %d %%' % (len(data), 100 * len(data) // len(image)))
                tstart = time.time()
            relay.answer(OTA_CMD_INFO, struct.pack('<BIBII', OTA_STATUS_OK, len(data), flags, len(image), crc32_mpeg2(data)))

        elif cmd == OTA_CMD_DATA and len(payload) == 5 and data is not None:
            offset, max_len = struct.unpack('<IB', payload)
            if offset >= len(data) or max_len == 0: continue
            requests += 1
            if offset <= last_offset: repeats += 1
            last_offset = offset
            chunk = data[offset:offset + max_len]
            relay.answer(OTA_CMD_DATA, struct.pack('<I', offset) + chunk)
            print('\r%3d %%' % (100 * (offset + len(chunk)) // len(data)), end='', flush=True)

        elif cmd == OTA_CMD_RESULT and len(payload) == 1:
            relay.answer(OTA_CMD_RESULT)
            if tresult is None:
                tresult = time.time()
                result = payload[0]
                print('\rdone in %.1f s, %d requests, %d repeated' % (tresult - tstart, requests, repeats))

    relay.end()
    if result != OTA_STATUS_OK: fail(None, 'receiver rejected the image, ' + status_str(result))
    print('receiver accepted the image and reboots')
    if radio: print('power cycle the radio, to get it and the module out of the passthrough')


if __name__ == "__main__":
    main()
