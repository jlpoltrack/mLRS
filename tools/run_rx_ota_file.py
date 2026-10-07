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
 usage: run_rx_ota_file.py <firmware> [<image file>]
 firmware: the .hex or .bin of the build, for STM32 and ESP receivers
 image file: default is the name of the firmware with .ota
********************************************************
'''
import os
import struct
import sys
import zlib

from run_rx_ota import OTA_APP_INFO_OFFSET, OTA_APP_INFO_MAGIC, ESP_IMAGE_MAGIC, hex_to_bin, stm32_app


# must match Common/ota/ota_loader.h
OTA_FILE_MAGIC = 0x53544F4D
OTA_FILE_FLAG_DEFLATE = 0x01


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

    F = open(filename, mode='rb')
    image = F.read()
    F.close()
    if filename.lower().endswith('.hex'): image = hex_to_bin(image)

    if len(image) < OTA_APP_INFO_OFFSET + 16: fail('not an OTA image')
    if image[0] == ESP_IMAGE_MAGIC:
        # esp firmware.bin, the app info sits somewhere, goes as raw deflate stream
        # the tx makes it into the zlib or gzip stream the receiver wants, so it gets what their trailers need
        pos = image.find(struct.pack('<I', OTA_APP_INFO_MAGIC))
        if pos < 0 or image.find(struct.pack('<I', OTA_APP_INFO_MAGIC), pos + 4) >= 0:
            fail('not an OTA image')
        magic, target_id, length, version = struct.unpack_from('<IIII', image, pos)
        if length != 0: fail('not an OTA image')
        deflate = zlib.compressobj(9, zlib.DEFLATED, -15)
        data = deflate.compress(image) + deflate.flush()
        flags, adler32, crc32 = OTA_FILE_FLAG_DEFLATE, zlib.adler32(image), zlib.crc32(image)
    else:
        # stm32, the app alone with length and crc filled in, goes as it is, the loader can't inflate
        image = stm32_app(image)
        if image is None: fail('not an OTA image')
        magic, target_id, length, version = struct.unpack_from('<IIII', image, OTA_APP_INFO_OFFSET)
        data = image
        flags, adler32, crc32 = 0, 0, 0

    header = struct.pack('<IIIIIIIII', OTA_FILE_MAGIC, target_id, version, flags, len(data), zlib.crc32(data),
                         len(image), adler32, crc32)
    header += struct.pack('<I', zlib.crc32(header))

    F = open(outname, mode='wb')
    F.write(header + data)
    F.close()
    print('image: target %08X' % target_id, 'version', version, 'length', len(image))
    print(outname, len(header) + len(data), 'bytes')


if __name__ == "__main__":
    main()
