//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// OTA, Tx side
// bridges between the receiver and a host on the com port, see ota_protocol.h
//*******************************************************
// The tx knows nothing about the protocol besides the session id. What the receiver sends goes to the
// host, what the host sends goes to the receiver. The receiver repeats if something gets lost.
// tx -> host: STX, len, packet[len], crc16
// host -> tx: STX, len, packet[len], crc16     len = 0 ends the relay
// crc16 is fmav_crc_calculate() over len and packet
//*******************************************************
#ifndef OTA_RELAY_TX_H
#define OTA_RELAY_TX_H
#pragma once


#ifndef USE_RX_OTA

class tTxOtaRelay
{
  public:
    void Tick_ms(void) {}
    void Start(void) {}
    bool Ready(void) { return false; }
    void Run(tSerialBase* const com, uint32_t sx_freq_reg, uint16_t session_id) {}
};

#else

#include "../Common/ota/ota_link.h"


#define OTA_RELAY_STX               0xA5
#define OTA_RELAY_START_DELAY_MS    1000 // time for the rx to get the cmd and go into ota
#define OTA_RELAY_HOST_TMO_MS       10000 // leave if the host goes silent
#define OTA_RELAY_TRANSMIT_TMO_MS   250


class tTxOtaRelay
{
  public:
    void Tick_ms(void);
    void Start(void) { start_delay_ms = OTA_RELAY_START_DELAY_MS; }
    bool Ready(void) { return ready; }
    void Run(tSerialBase* const _com, uint32_t sx_freq_reg, uint16_t _session_id); // the caller has to restart the controller afterwards

  private:
    bool host_receive(void);
    void host_send(uint8_t* const data, uint8_t data_len);
    uint16_t host_crc(uint8_t* const data, uint8_t data_len);
    bool dio_is_set(void);
    void radio_receive(void);
    void radio_transmit(void);

    uint16_t start_delay_ms;
    bool ready;

    tSerialBase* com;
    uint16_t session_id;

    uint8_t buf[OTA_PACKET_LEN_MAX + 8]; // what comes from the host, goes to the receiver
    uint8_t len;
    uint8_t rx_buf[OTA_PACKET_LEN_MAX + 8]; // what comes from the receiver, separate as a host frame may be half in

    uint8_t state; // 0: wait for STX, 1: len, 2: data, 3,4: crc
    uint8_t pos;
    uint16_t crc;
};


void tTxOtaRelay::Tick_ms(void)
{
    if (!start_delay_ms) return;
    start_delay_ms--;
    if (!start_delay_ms) ready = true;
}


uint16_t tTxOtaRelay::host_crc(uint8_t* const data, uint8_t data_len)
{
    uint16_t crc16 = fmav_crc_calculate(&data_len, 1);
    fmav_crc_accumulate_buf(&crc16, data, data_len);
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
            if (crc == host_crc(buf, len)) return true;
            break;
        }
    }
    return false;
}


void tTxOtaRelay::host_send(uint8_t* const data, uint8_t data_len)
{
uint8_t frame[OTA_PACKET_LEN_MAX + 4];

    uint16_t crc16 = host_crc(data, data_len);

    frame[0] = OTA_RELAY_STX;
    frame[1] = data_len;
    memcpy(frame + 2, data, data_len);
    frame[data_len + 2] = crc16 & 0xFF;
    frame[data_len + 3] = crc16 >> 8;
    com->putbuf(frame, data_len + 4);
}


// dio is set for done, timeout and errors, the irq status is read only then
bool tTxOtaRelay::dio_is_set(void)
{
    return gpio_read_activehigh(SX_DIO);
}


void tTxOtaRelay::radio_receive(void)
{
    ota_link_set_packet_len(255);
    sx.SetToRx();
}


// sends the packet in buf to the receiver, and goes back to receive
void tTxOtaRelay::radio_transmit(void)
{
    ota_set_session_id(buf, session_id);
    ota_link_set_packet_len(len);
    sx.SendFrame(buf, len, OTA_RELAY_TRANSMIT_TMO_MS);

    uint32_t tstart_ms = millis32();
    while (!dio_is_set()) {
        if (millis32() - tstart_ms > OTA_RELAY_TRANSMIT_TMO_MS) break;
    }
    sx.GetAndClearIrqStatus(OTA_SX(IRQ_ALL));
    sx_dio_exti_isr_clearflag();

    radio_receive();
}


// blocks until the host ends it or goes silent
// the dio isr is off, dio is polled, the restart turns the isr on again
void tTxOtaRelay::Run(tSerialBase* const _com, uint32_t sx_freq_reg, uint16_t _session_id)
{
    ready = false;
    com = _com;
    session_id = _session_id;
    if (!com) return;

#ifdef ESP32
    sx_dio_init_exti_isroff();
#else
    NVIC_DisableIRQ(SX_DIO_EXTI_IRQn);
#endif

    state = 0;
    ota_link_start(sx_freq_reg);
    sx.SetRfPower_dbm(rfpower_list[0].dbm); // tx and rx sit next to each other, and long frames come back to back
    radio_receive();

    uint32_t tlast_ms = millis32();

    while (1) {
        if (dio_is_set()) {
            uint8_t rx_len = ota_link_read(sx.GetAndClearIrqStatus(OTA_SX(IRQ_ALL)), rx_buf);
            sx_dio_exti_isr_clearflag();
            if (rx_len && ota_session_id(rx_buf) == session_id) host_send(rx_buf, rx_len);
            radio_receive();
#ifndef DEVICE_HAS_NO_LED
            led_red_toggle();
#endif
        }

        if (!host_receive()) {
            if (millis32() - tlast_ms > OTA_RELAY_HOST_TMO_MS) break;
            continue;
        }
        tlast_ms = millis32();
        if (!len) break; // host says we are done
        if (len >= OTA_PACKET_HEADER_LEN) radio_transmit();
    }
}


#endif // USE_RX_OTA

#endif // OTA_RELAY_TX_H
