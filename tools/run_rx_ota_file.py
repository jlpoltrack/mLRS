#!/usr/bin/env python
'''
*******************************************************
 Copyright (c) MLRS project
 GPL3
 https://www.gnu.org/licenses/gpl-3.0.de.html
*******************************************************
 run_rx_ota_file.py
 makes the image file of a receiver firmware, which goes onto the SD card of the radio, into /FIRMWARE
 the lua script mLRS-RxUpdate.lua then updates the receiver with it over the air
 usage: run_rx_ota_file.py <firmware.bin> [<image file>]
 image file: default is the name of the firmware with .ota
********************************************************
'''
import os
import struct
import sys
import zlib

from run_rx_ota import image_info, crc32_mpeg2


# must match lua/mLRS-RxUpdate.lua
OTA_FILE_MAGIC = 0x32544F4D

# the lua script puts these around the raw deflate stream
ZLIB_HEAD = bytes([0x78, 0xDA])
GZIP_HEAD = bytes([0x1F, 0x8B, 0x08, 0, 0, 0, 0, 0, 0x02, 0xFF])


def fail(txt):
    print('ERROR:', txt)
    sys.exit(1)


def main():
    args = sys.argv[1:]
    if len(args) < 1 or len(args) > 2:
        print(__doc__)
        sys.exit(1)
    filename = args[0]
    outname = args[1] if len(args) > 1 else os.path.splitext(filename)[0] + '.ota'

    with open(filename, 'rb') as F: image = F.read()

    # esp firmware.bin, the app info sits somewhere
    info = image_info(image)
    if not info: fail('not an OTA image')
    name, version = info

    # raw deflate stream, the lua script makes it into the zlib or gzip stream the receiver wants
    deflate = zlib.compressobj(9, zlib.DEFLATED, -15)
    data = deflate.compress(image) + deflate.flush()
    adler32, crc32 = zlib.adler32(image), zlib.crc32(image)
    zlib_stream = ZLIB_HEAD + data + struct.pack('>I', adler32)
    gzip_stream = GZIP_HEAD + data + struct.pack('<II', crc32, len(image))
    if zlib.decompress(zlib_stream) != image or zlib.decompress(gzip_stream, 16 + 15) != image:
        fail('compression failed')

    # the crcs of the streams are what the receiver checks, see Common/ota/ota_protocol.h
    header = struct.pack('<IIIIIIII20s', OTA_FILE_MAGIC, version, len(data), len(image), adler32, crc32,
                         crc32_mpeg2(zlib_stream), crc32_mpeg2(gzip_stream), name[:20].encode())

    with open(outname, 'wb') as F: F.write(header + data)
    print('image: %s, version %d, length %d' % (name, version, len(image)))
    print(outname, len(header) + len(data), 'bytes')


if __name__ == "__main__":
    main()
