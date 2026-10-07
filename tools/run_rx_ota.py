#!/usr/bin/env python
'''
*******************************************************
 Copyright (c) MLRS project
 GPL3
 https://www.gnu.org/licenses/gpl-3.0.de.html
*******************************************************
 run_rx_ota.py
 updates a receiver which has an OTA loader over the air, via the CLI of the tx module
 usage: run_rx_ota.py <port> <firmware> [--baud 115200] [--window 2] [--timeout 60] [--no-cli] [--no-compress] [--radio]
 firmware: the .bin of the build
 --window: number of data blocks sent ahead, 1 = each block waits for the response to the one before
 --timeout: seconds the transfer may take
 --no-cli: don't send the rxota CLI command, the tx is in relay mode already
 --radio: the tx module is an internal one, port is the usb serial (VCP) of the EdgeTX radio
   with --no-cli the radio is in serial passthrough to the module already
 --no-compress: don't compress the image, even if the receiver can handle that
********************************************************
'''
import gzip
import struct
import sys
import time
import zlib

import serial # pyserial
import serial.tools.list_ports


# must match Common/ota/ota_loader.h and CommonTx/ota_relay_tx.h
OTA_APP_INFO_OFFSET = 0x0200
OTA_APP_INFO_MAGIC = 0x4F4C524D
OTA_CMD_HELLO, OTA_CMD_BEGIN, OTA_CMD_DATA, OTA_CMD_END = 1, 2, 3, 4
OTA_CMD_RESPONSE = 0x80
OTA_STATUS = ['ok', 'wrong target', 'bad length', 'bad state', 'flash error', 'image check failed', 'not supported', 'bad data']
OTA_LOADER_VERSION = 1
OTA_FLAG_DEFLATE = 0x01
OTA_FLAG_GZIP = 0x02
OTA_RELAY_STX = 0xA5
MBRIDGE_CMD_RX_OTA = 19
JRPIN5_BAUDS = [400000, 921600, 1870000] # of the uart between radio and internal tx module, = txcrsf_bauds[]
OTA_RELAY_JRPIN5_BAUDRATE = 230400 # the module goes to it when its relay starts
ESP_IMAGE_MAGIC = 0xE9

RETRIES = 10
WINDOW = 2 # data blocks on their way, the tx gets the next one while it does the radio with the current one


def crc16(data): # = fmav_crc_calculate()
    crc = 0xFFFF
    for b in data:
        tmp = (b ^ (crc & 0xFF)) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def crc8_crsf(data): # poly 0xD5
    crc = 0
    for b in data:
        crc ^= b
        for i in range(8): crc = ((crc << 1) ^ 0xD5) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


# mBridge command in a CRSF frame, as the lua script sends it: 0xEE, len, 0x81, 'O', 'W', 0xA0 + cmd, crc8
def crsf_mbridge_cmd(cmd):
    body = bytes([0x81, 0x4F, 0x57, 0xA0 + cmd])
    return bytes([0xEE, len(body) + 1]) + body + bytes([crc8_crsf(body)])


# makes the radio pass its usb serial through to the internal tx module
# stopping the pulses powers the module off, so it is powered on again, and boots into its firmware
def edgetx_passthrough(ser):
    for cmd in (b'set pulses 0', b'set rfmod 0 bootpin 0', b'set rfmod 0 power on', b'serialpassthrough rfmod 0 %d' % JRPIN5_BAUDS[0]):
        ser.write(cmd + b'\n')
        time.sleep(0.5)
        print('radio:', cmd.decode(), '->', ser.read(500).decode(errors='replace').strip().replace('\r\n', ' | '))


# plays the radio: sends channel frames, and looks at the link statistics the module answers with
# the module autobauds at startup, and stays at one of its rates, so we have to find it, the radio follows our rate
# returns true when the module reports a link to the receiver
def crsf_wait_connected(ser, timeout=25.0):
    body = bytes([0x16]) + bytes([0xE0, 0x03, 0x1F, 0xF8, 0xC0, 0x07, 0x3E, 0xF0, 0x81, 0x0F, 0x7C] * 2) # all channels mid
    rc = bytes([0xC8, len(body) + 1]) + body + bytes([crc8_crsf(body)])
    buf = b''
    found = False # the baudrate of the module
    baud_i = 0
    tbaud = 0
    tend = time.time() + timeout
    ser.timeout = 0.02
    while time.time() < tend:
        if not found and time.time() - tbaud > 0.5:
            ser.baudrate = JRPIN5_BAUDS[baud_i]
            baud_i = (baud_i + 1) % len(JRPIN5_BAUDS)
            tbaud = time.time()
            ser.reset_input_buffer()
            buf = b''
        ser.write(rc)
        buf = (buf + ser.read(200))[-600:]
        pos = 0
        while pos + 2 < len(buf):
            n = buf[pos + 1]
            if buf[pos] not in (0xEA, 0xC8) or n < 2 or n > 62: pos += 1; continue
            if pos + 2 + n > len(buf): break
            frame = buf[pos:pos + 2 + n]
            if crc8_crsf(frame[2:-1]) != frame[-1]: pos += 1; continue
            if not found: print('module is talking CRSF at %d baud' % ser.baudrate); found = True
            if frame[2] == 0x14 and n >= 12 and frame[5] > 0: return True # link statistics, uplink lq
            pos += 2 + n
        buf = buf[pos:]
    if not found: print('WARNING: nothing heard from the module')
    return False


class cRelay:
    def __init__(self, ser):
        self.ser = ser
        self.no_response = 0 # retries since nothing came back
        self.bad_response = 0 # retries since something else came back

    def send(self, packet):
        body = bytes([len(packet)]) + packet
        self.ser.write(bytes([OTA_RELAY_STX]) + body + struct.pack('<H', crc16(body)))

    # returns the response of the loader, empty if the tx says there is none, None if the tx says nothing
    def receive(self, timeout=1.5):
        self.ser.timeout = timeout
        tend = time.time() + timeout # a port which sends something else all the time must not hang us
        while True:
            c = self.ser.read(1)
            if len(c) == 0 or time.time() > tend: return None
            if c[0] == OTA_RELAY_STX: break
        head = self.ser.read(1)
        if len(head) != 1: return None
        data = self.ser.read(head[0] + 2)
        if len(data) != head[0] + 2: return None
        if crc16(head + data[:-2]) != struct.unpack('<H', data[-2:])[0]: return None
        return data[:-2]

    # sends a command to the loader, returns status and payload of its response
    def command(self, cmd, payload=b'', retries=RETRIES):
        for n in range(retries):
            self.send(bytes([cmd, 0, 0]) + payload) # session id is filled in by the tx
            res = self.receive()
            if res and len(res) >= 4 and res[0] == (cmd | OTA_CMD_RESPONSE):
                return res[3], res[4:]
            if not res: self.no_response += 1
            else: self.bad_response += 1
        return None, None

    def end(self):
        self.send(b'')


def fail(relay, txt):
    print('ERROR:', txt)
    if relay: relay.end()
    sys.exit(1)


def status_str(status):
    return OTA_STATUS[status] if status < len(OTA_STATUS) else str(status)


def main():
    args = sys.argv[1:]
    baud = 115200
    if '--baud' in args:
        i = args.index('--baud')
        baud = int(args[i+1])
        del args[i:i+2]
    window = WINDOW
    if '--window' in args:
        i = args.index('--window')
        window = max(1, int(args[i+1]))
        del args[i:i+2]
    timeout = 60.0
    if '--timeout' in args:
        i = args.index('--timeout')
        timeout = float(args[i+1])
        del args[i:i+2]
    no_cli = '--no-cli' in args
    no_compress = '--no-compress' in args
    radio = '--radio' in args
    args = [a for a in args if not a.startswith('--')]
    if len(args) != 2:
        print(__doc__)
        sys.exit(1)
    port, filename = args

    F = open(filename, mode='rb')
    image = F.read()
    F.close()

    if len(image) < OTA_APP_INFO_OFFSET + 16: fail(None, 'not an OTA image')
    if image[0] == ESP_IMAGE_MAGIC:
        # esp firmware.bin, the app info sits somewhere, the receiver checks the image's own checksum
        pos = image.find(struct.pack('<I', OTA_APP_INFO_MAGIC))
        if pos < 0 or image.find(struct.pack('<I', OTA_APP_INFO_MAGIC), pos + 4) >= 0:
            fail(None, 'not an OTA image')
        magic, target_id, length, version = struct.unpack_from('<IIII', image, pos)
        if length != 0: fail(None, 'not an OTA image')
        length = len(image)
    else:
        fail(None, 'not an OTA image')
    print('image: target %08X' % target_id, 'version', version, 'length', length)

    # the usb-uart adapter of an ESP32 tx may reset it or hold it in boot with DTR/RTS set, a usb com port may need them
    is_usb_com = any(p.device == port and p.vid == 0x0483 for p in serial.tools.list_ports.comports())
    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = port, baud, 0.5
    if not is_usb_com: ser.dtr, ser.rts = False, False
    ser.open()
    relay = cRelay(ser)

    if radio:
        if not no_cli: edgetx_passthrough(ser)
        # the module tells the receiver to go into ota only if it is connected
        print('waiting for the receiver to be connected...')
        if not crsf_wait_connected(ser): print('WARNING: receiver is not connected, trying anyway')
        ser.timeout = 0.5
        ser.write(crsf_mbridge_cmd(MBRIDGE_CMD_RX_OTA)) # the module is still listening for CRSF
        time.sleep(2.0) # relay starts after 1 s
        ser.baudrate = OTA_RELAY_JRPIN5_BAUDRATE
    elif not no_cli:
        # opening the port may have reset the tx, and the receiver must be connected to get told to go into ota
        # a receiver which sits in its loader doesn't connect, so go on in any case
        print('waiting for the receiver to be connected...')
        tend = time.time() + 15.0
        while time.time() < tend:
            ser.reset_input_buffer()
            ser.write(b'v;')
            time.sleep(1.0)
            res = ser.read(1000)
            if b'Rx: ' in res and b'not connected' not in res: break
        ser.write(b'rxota;')
        time.sleep(2.0) # relay starts after 1 s
    ser.reset_input_buffer()

    print('looking for receiver...')
    status, payload = relay.command(OTA_CMD_HELLO, retries=20)
    if status is None: fail(relay, 'no response from receiver')
    if len(payload) < 2 or payload[0] != OTA_LOADER_VERSION or len(payload) < 11:
        fail(relay, 'receiver has loader version %s, this tool is for version %d' % (payload[0] if payload else '?', OTA_LOADER_VERSION))
    loader_version, block_size, rx_target_id, app_size_max, rx_flags = struct.unpack('<BBIIB', payload[:11])
    print('receiver: target %08X' % rx_target_id, 'loader version', loader_version)
    if rx_target_id != target_id: fail(relay, 'image is not for this receiver')
    if length > app_size_max: fail(relay, 'image is too large')

    data, flags = image, 0
    if (rx_flags & OTA_FLAG_DEFLATE) and not no_compress:
        data, flags = zlib.compress(image, 9), OTA_FLAG_DEFLATE
    elif (rx_flags & OTA_FLAG_GZIP) and not no_compress:
        data, flags = gzip.compress(image, 9, mtime=0), OTA_FLAG_GZIP
    if flags:
        print('compressed to %d bytes, %d %%' % (len(data), 100 * len(data) // length))

    status, payload = relay.command(OTA_CMD_BEGIN, struct.pack('<IIBI', len(data), target_id, flags, length))
    if status != 0: fail(relay, 'begin failed' + ('' if status is None else ', ' + status_str(status)))

    block_num = (len(data) + block_size - 1) // block_size
    block = 0
    resent = 0 # blocks the loader asked for again
    relay.no_response = relay.bad_response = 0 # count only the transfer
    tstart = time.time()
    # the tx answers each block it gets, in order, and the loader tells in each response which block it wants next
    # so blocks can be sent ahead, if one gets lost the loader refuses those behind it, and we go back
    pending = 0 # blocks sent for which the answer of the tx is to come
    send_next = 0
    resync = False # something went wrong, wait for what is on its way, then go on with the block the loader wants
    fails = 0
    while block < block_num or pending:
        if time.time() - tstart > timeout: fail(relay, 'timeout, at block %d of %d' % (block, block_num))
        while not resync and pending < window and send_next < block_num:
            relay.send(bytes([OTA_CMD_DATA, 0, 0]) + struct.pack('<H', send_next) + data[send_next*block_size:(send_next+1)*block_size])
            send_next += 1
            pending += 1
        if pending:
            res = relay.receive()
            pending = (pending - 1) if res is not None else 0 # if the tx is silent nothing more is to come
            expected = send_next - pending # the block the loader wants if all went well up to this response
            if res and len(res) >= 6 and res[0] == (OTA_CMD_DATA | OTA_CMD_RESPONSE):
                if res[3] != 0: fail(relay, 'block %d failed, ' % block + status_str(res[3]))
                block = struct.unpack('<H', res[4:6])[0]
                if block != expected: resync = True
                else: fails = 0
                print('\r%3d %%' % (100 * block // block_num), end='', flush=True)
            else:
                if not res: relay.no_response += 1
                else: relay.bad_response += 1
                resync = True
        if resync and not pending:
            resync = False
            resent += 1
            fails += 1
            if fails > RETRIES: fail(relay, 'block %d failed' % block)
            send_next = block
    print('\rdone in %.1f s' % (time.time() - tstart))
    print('%d blocks, window %d, retries: %d no response, %d bad response, %d times gone back' %
        (block_num, window, relay.no_response, relay.bad_response, resent))

    status, payload = relay.command(OTA_CMD_END, struct.pack('<I', zlib.crc32(data)))
    relay.end()
    if status is None:
        print('WARNING: no response to end, receiver may have rebooted already, check that it connects')
    elif status != 0:
        fail(None, 'receiver rejected the image, ' + status_str(status))
    else:
        print('receiver accepted the image and reboots')


if __name__ == "__main__":
    main()
