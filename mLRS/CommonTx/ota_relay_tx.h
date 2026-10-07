//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// OTA, Tx side
// relays packets between a host on the com port and the receiver's OTA loader
//*******************************************************
// The tx knows nothing about the OTA protocol besides the session id and the link, the host does it all.
// host -> tx: STX, len, packet[len], crc16     len = 0 ends the relay
// tx -> host: STX, len, response[len], crc16   len = 0 if the receiver did not respond
// crc16 is fmav_crc_calculate() over len and data
// The host may send the next packet while the tx is busy with the current one, it waits in the com rx buffer.
// Internal tx modules have no com port. The host comes in via the radio, which passes its usb through to the
// JR pin5 uart, starts the relay with a mBridge command in a CRSF frame, and the relay then uses that uart.
//*******************************************************
#ifndef OTA_RELAY_TX_H
#define OTA_RELAY_TX_H
#pragma once


#include "../Common/ota/ota_link.h"


#define OTA_RELAY_STX               0xA5
#define OTA_RELAY_HOST_TMO_MS       10000 // leave if the host goes silent
#define OTA_RELAY_TRANSMIT_TMO_MS   250
#define OTA_RELAY_RESPONSE_TMO_MS   400 // must cover a page erase on the receiver

#if defined ESP32 && defined JR_PIN5_FULL_DUPLEX
  #define USE_RX_OTA_VIA_JRPIN5
  // the uart is at one of the fast CRSF rates, at which the long relay frames get lost, the radio follows the host
  #define OTA_RELAY_JRPIN5_BAUDRATE 230400
#endif


class tTxOtaRelay
{
  public:
    void Run(tSerialBase* const _com, uint32_t sx_freq_reg, uint16_t _session_id);

    // the relay takes over the sx dio isr while it is running
    void DioIsr(void);
    volatile bool active;

    // the isr does spi, so the main code must lock it out while it does spi itself
#ifdef ESP32
    void lock(void) { sx_dio_init_exti_isroff(); }
    void unlock(void) { sx_dio_enable_exti_isr(); }
#else
    void lock(void) { NVIC_DisableIRQ(SX_DIO_EXTI_IRQn); }
    void unlock(void) { NVIC_EnableIRQ(SX_DIO_EXTI_IRQn); }
#endif

  private:
    bool host_receive(void);
    void host_send(void);
    void transact(void);
    uint16_t host_crc(void);
    bool wait_dio(uint16_t tmo_ms);

    tSerialBase* com;
    uint16_t session_id;

    volatile bool dio_fired;
    volatile uint32_t irq_status;

    uint8_t buf[OTA_PACKET_LEN_MAX + 8];
    uint8_t len;

    uint8_t state; // 0: wait for STX, 1: len, 2: data, 3,4: crc
    uint8_t pos;
    uint16_t crc;
};


void tTxOtaRelay::DioIsr(void)
{
    uint32_t irq = sx.GetAndClearIrqStatus(OTA_SX(IRQ_ALL));
    if (irq) { // can be a stale one
        irq_status = irq;
        dio_fired = true;
    }
}


bool tTxOtaRelay::wait_dio(uint16_t tmo_ms)
{
    uint32_t tstart_ms = millis32();
    while (!dio_fired) {
        if (millis32() - tstart_ms > tmo_ms) return false;
    }
    return true;
}


uint16_t tTxOtaRelay::host_crc(void)
{
    uint16_t crc16 = fmav_crc_calculate(&len, 1);
    fmav_crc_accumulate_buf(&crc16, buf, len);
    return crc16;
}


// returns true if a complete and valid frame is in buf
bool tTxOtaRelay::host_receive(void)
{
    while (com->available()) {
        uint8_t c = com->getc();
        switch (state) {
        case 0:
            if (c == OTA_RELAY_STX) state = 1;
            break;
        case 1:
            if (c > OTA_PACKET_LEN_MAX) { state = 0; break; }
            len = c;
            pos = 0;
            state = (len) ? 2 : 3;
            break;
        case 2:
            buf[pos++] = c;
            if (pos >= len) state = 3;
            break;
        case 3:
            crc = c;
            state = 4;
            break;
        case 4:
            crc |= (uint16_t)c << 8;
            state = 0;
            if (crc == host_crc()) return true;
            break;
        }
    }
    return false;
}


void tTxOtaRelay::host_send(void)
{
    uint16_t crc16 = host_crc();

    com->putc(OTA_RELAY_STX);
    com->putc(len);
    com->putbuf(buf, len);
    com->putc(crc16 & 0xFF);
    com->putc(crc16 >> 8);
}


// sends the packet in buf, and puts the response into buf, len = 0 if there is none
void tTxOtaRelay::transact(void)
{
    if (len >= OTA_PACKET_HEADER_LEN) {
        buf[1] = session_id & 0xFF;
        buf[2] = session_id >> 8;
    }

    len = ota_link_pack(buf, len, OTA_FSK_FRAME_LEN_TX);

    lock();
    ota_link_set_packet_len(len);
    dio_fired = false;
    sx.SendFrame(buf, len, OTA_RELAY_TRANSMIT_TMO_MS);
    unlock();
    wait_dio(OTA_RELAY_TRANSMIT_TMO_MS + 50);

    len = 0;

    lock();
    ota_link_set_packet_len((OTA_LINK_IS_FSK) ? OTA_FSK_FRAME_LEN_RX : 255);
    dio_fired = false;
    sx.SetToRx();
    unlock();
    bool received = wait_dio(OTA_RELAY_RESPONSE_TMO_MS);

    lock();
    if (received) len = ota_link_read(irq_status, buf, OTA_FSK_FRAME_LEN_RX);
    sx.SetToIdle();
    unlock();
}


// blocks until the host ends it or goes silent, the caller has to restart the controller afterwards
void tTxOtaRelay::Run(tSerialBase* const _com, uint32_t sx_freq_reg, uint16_t _session_id)
{
    com = _com;
    session_id = _session_id;
    if (!com) return;

    active = true;
    state = 0;

    lock();
    ota_link_start(sx_freq_reg);
    ota_link_set_packet_len(255);
    sx.SetToIdle();
    unlock();

    uint32_t tlast_ms = millis32();

    while (1) {
        if (!host_receive()) {
            if (millis32() - tlast_ms > OTA_RELAY_HOST_TMO_MS) break;
            continue;
        }
        tlast_ms = millis32();

        if (!len) break; // host says we are done

        transact();
        host_send();
#ifndef DEVICE_HAS_NO_LED
        led_red_toggle();
#endif
    }

    active = false;
}


#endif // OTA_RELAY_TX_H
