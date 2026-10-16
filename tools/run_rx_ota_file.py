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
 usage: run_rx_ota_file.py <firmware.bin, .hex for STM32> [<image file>]
 image file: default is the name of the firmware with .ota
********************************************************
'''
import os
import struct
import sys
import zlib

from run_rx_ota import load_image, image_info, crc32_mpeg2, ESP_IMAGE_MAGIC


# must match lua/mLRS-RxUpdate.lua
OTA_FILE_MAGIC = 0x32544F4D
OTA_FILE_FLAG_RAW = 0x01 # the image as it is, for STM32, which takes no compressed image

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

    image = load_image(filename)

    # the app info sits somewhere
    info = image_info(image)
    if not info: fail('not an OTA image')
    name, version = info

    # STM32, the image as it is, the crcs of the streams are both the crc of the image
    if image[0] != ESP_IMAGE_MAGIC:
        crc = crc32_mpeg2(image)
        header = struct.pack('<IIIIIIII20sI', OTA_FILE_MAGIC, version, len(image), len(image), 0, 0, crc, crc,
                             name[:20].encode(), OTA_FILE_FLAG_RAW)
        with open(outname, 'wb') as F: F.write(header + image)
        print('image: %s, version %d, length %d, not compressed' % (name, version, len(image)))
        print(outname, len(header) + len(image), 'bytes')
        return

    # raw deflate stream, the lua script makes it into the zlib or gzip stream the receiver wants
    deflate = zlib.compressobj(9, zlib.DEFLATED, -15)
    data = deflate.compress(image) + deflate.flush()
    adler32, crc32 = zlib.adler32(image), zlib.crc32(image)
    zlib_stream = ZLIB_HEAD + data + struct.pack('>I', adler32)
    gzip_stream = GZIP_HEAD + data + struct.pack('<II', crc32, len(image))
    if zlib.decompress(zlib_stream) != image or zlib.decompress(gzip_stream, 16 + 15) != image:
        fail('compression failed')

    # the crcs of the streams are what the receiver checks, see Common/ota/ota_protocol.h
    header = struct.pack('<IIIIIIII20sI', OTA_FILE_MAGIC, version, len(data), len(image), adler32, crc32,
                         crc32_mpeg2(zlib_stream), crc32_mpeg2(gzip_stream), name[:20].encode(), 0)

    with open(outname, 'wb') as F: F.write(header + data)
    print('image: %s, version %d, length %d' % (name, version, len(image)))
    print(outname, len(header) + len(data), 'bytes')


if __name__ == "__main__":
    main()
