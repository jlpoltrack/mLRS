'''
*******************************************************
 Copyright (c) MLRS project
 GPL3
 https://www.gnu.org/licenses/gpl-3.0.de.html
*******************************************************
 build_env_esp_wdt_reset.py
 PlatformIO extra script for ESP32 devices flashed via
 a bare USB-UART adapter without auto-reset
 Restarts the device with an RTC watchdog reset after
 flashing, on the classic ESP32 via the esptool stub
*******************************************************
'''
Import("env")
import serial
import struct
import time


# ESP32 RTC watchdog registers
RTC_CNTL_WDTCONFIG0_REG = 0x3FF4808C
RTC_CNTL_WDTCONFIG1_REG = 0x3FF48090
RTC_CNTL_WDTWPROTECT_REG = 0x3FF480A4


def stub_write_reg(s, addr, value):
    # esptool serial protocol, WRITE_REG (0x09), SLIP framed
    data = struct.pack('<IIII', addr, value, 0xFFFFFFFF, 0)
    pkt = struct.pack('<BBHI', 0x00, 0x09, len(data), 0) + data
    pkt = pkt.replace(b'\xdb', b'\xdb\xdd').replace(b'\xc0', b'\xdb\xdc')
    s.write(b'\xc0' + pkt + b'\xc0')

    deadline = time.time() + 0.5
    buf = b''
    while time.time() < deadline:
        buf += s.read(s.in_waiting or 1)
        if buf.count(b'\xc0') >= 2:
            return True
    return False


def wdt_reset(source, target, env):
    port = env.subst("$UPLOAD_PORT")
    baud = int(env.subst("$UPLOAD_SPEED"))
    print("======== RTC WATCHDOG RESET ========")
    print("  Port: %s @ %s" % (port, baud))

    s = serial.Serial()
    s.port = port
    s.baudrate = baud
    s.timeout = 0.05
    s.dtr = False
    s.rts = False
    s.open()
    s.read(s.in_waiting)  # flush

    if not stub_write_reg(s, RTC_CNTL_WDTWPROTECT_REG, 0x50D83AA1):  # unlock
        print("  WARNING: no response from stub, power cycle the device")
    stub_write_reg(s, RTC_CNTL_WDTCONFIG1_REG, 1000)  # short timeout
    stub_write_reg(s, RTC_CNTL_WDTCONFIG0_REG, 0xC0004800)  # enable, system reset

    s.close()
    print("======== RESET DONE ========")


# esptool can do the watchdog reset itself, except on the classic ESP32
mcu = env.BoardConfig().get("build.mcu", "esp32").lower()
after = "no_reset_stub" if mcu == "esp32" else "watchdog_reset"
env.Replace(UPLOADERFLAGS=[
    after if flag == "hard_reset" else flag for flag in env["UPLOADERFLAGS"]
])
if mcu == "esp32":
    env.AddPostAction("upload", wdt_reset)
