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
// The tx can also be the host by itself. It then sends an image which the radio sends to it from a file on
// its SD card, see ota_loader.h for the file. A lua script on the radio sends it in CRSF frames
//   0xEE, len, 0x81, 0x67, offset[3], n, data[n], crc8
// offset is the position in the data of the file. The script starts with the header of the file, as offset
// 0xFFFFFF, and then sends the data as far as the tx tells that it can take it.
// The tx answers with a mBridge RX_OTA_STATUS command whenever the radio lets it send.
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

#ifdef USE_RX_OTA_STREAM
#include "../Common/libs/fifo.h"

#define OTA_HOST_HELLO_RETRIES      20 // the receiver has to reboot into its loader first
#define OTA_HOST_RETRIES            10

typedef enum {
    OTA_HOST_OK = 0,
    OTA_HOST_ERR_NO_RECEIVER,
    OTA_HOST_ERR_NO_IMAGE, // it is for another receiver
    OTA_HOST_ERR_IMAGE, // damaged, or the receiver can't take it
    OTA_HOST_ERR_TRANSFER,
    OTA_HOST_ERR_REJECTED, // by the receiver
    OTA_HOST_ERR_NO_DATA, // the radio stopped to send
} OTA_HOST_RESULT_ENUM;

// = zlib crc32()
uint32_t ota_host_crc32(uint32_t crc, const uint8_t* data, uint32_t len)
{
    crc = ~crc;
    while (len--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++) crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
    }
    return ~crc;
}

#define OTA_STREAM_START            0xFFFFFF // offset of the frame with the header
#define OTA_STREAM_FIFO_SIZE        1024
#define OTA_STREAM_DATA_TMO_MS      3000 // the receiver leaves if nothing comes for some seconds
#define OTA_STREAM_DONE_LINGER_MS   500 // time to tell the radio the result
#define OTA_STREAM_STATUS_PERIOD_MS 20 // each one can cost a frame of the radio on a half duplex line

typedef enum {
    OTA_STREAM_STATE_IDLE = 0,
    OTA_STREAM_STATE_STARTING, // got the header, looking for the receiver
    OTA_STREAM_STATE_TRANSFER, // radio shall send the data
    OTA_STREAM_STATE_DONE,
} OTA_STREAM_STATE_ENUM;
#endif


class tTxOtaRelay
{
  public:
    void Run(tSerialBase* const _com, uint32_t sx_freq_reg, uint16_t _session_id);
#ifdef USE_RX_OTA_STREAM
    uint8_t RunStream(uint32_t sx_freq_reg, uint16_t _session_id);
    void StreamFrame(const uint8_t* payload, uint8_t payload_len); // is called in isr context
    bool StreamStartRequested(void);
    bool StreamActive(void) { return (stream_state == OTA_STREAM_STATE_TRANSFER); }
#endif

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
    void start(uint32_t sx_freq_reg);
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

#ifdef USE_RX_OTA_STREAM
    bool command(uint8_t cmd, const uint8_t* payload, uint8_t payload_len, uint8_t retries);
    bool data_read(uint32_t data_pos, uint8_t* data, uint32_t n);
    bool stream_read(uint32_t stream_pos, uint8_t* data, uint8_t n);
    uint8_t send_image(void);
    void pump(void);

    // what is transferred: head, data of the file, tail
    tOtaFileHeader header;
    uint8_t head[10];
    uint8_t head_len;
    uint8_t tail[8];
    uint8_t tail_len;

    bool stream_running; // RunStream() is at work

    tFifo<uint8_t,OTA_STREAM_FIFO_SIZE> stream_fifo; // filled in isr context
    volatile bool stream_start_request;
    volatile uint8_t stream_state;
    uint8_t stream_result;
    uint8_t stream_nack_seq;
    volatile uint32_t stream_rx_offset; // what we got from the radio
    uint32_t stream_gap_offset; // rx_offset for which the radio was told that data got lost
    uint32_t stream_read_pos; // what was taken out of the fifo
    uint32_t stream_data_crc32;
    uint32_t stream_status_tlast_ms;
    volatile uint8_t stream_crc_errors;
    volatile uint16_t stream_rejected;
#endif
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
#ifdef USE_RX_OTA_STREAM
        pump(); // the radio must be served all the time
#endif
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


void tTxOtaRelay::start(uint32_t sx_freq_reg)
{
    active = true;

    lock();
    ota_link_start(sx_freq_reg);
#ifndef DEVICE_HAS_SX127x
    ota_link_set_packet_len(255);
#endif
    sx.SetToIdle();
    unlock();
}


// blocks until the host ends it or goes silent, the caller has to restart the controller afterwards
void tTxOtaRelay::Run(tSerialBase* const _com, uint32_t sx_freq_reg, uint16_t _session_id)
{
    com = _com;
    session_id = _session_id;
    if (!com) return;

    state = 0;
    start(sx_freq_reg);

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


//-------------------------------------------------------
// The tx is the host, the image comes from the radio
//-------------------------------------------------------
#ifdef USE_RX_OTA_STREAM

static inline uint32_t ota_get_u32(const uint8_t* p) { return p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static inline void ota_put_u32(uint8_t* p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }


// returns true if the receiver responded to the command, buf then has cmd, session_id[2], status, payload
bool tTxOtaRelay::command(uint8_t cmd, const uint8_t* payload, uint8_t payload_len, uint8_t retries)
{
    while (retries--) {
        buf[0] = cmd; // the session id is filled in by transact()
        memcpy(buf + OTA_PACKET_HEADER_LEN, payload, payload_len);
        len = OTA_PACKET_HEADER_LEN + payload_len;
        transact();
#ifndef DEVICE_HAS_NO_LED
        led_red_toggle();
#endif
        if (len >= OTA_PACKET_HEADER_LEN + 3 && buf[0] == (cmd | OTA_CMD_RESPONSE)) return true;
    }
    return false;
}


// the data of the file, it comes only once and in order
bool tTxOtaRelay::data_read(uint32_t data_pos, uint8_t* data, uint32_t n)
{
    if (data_pos != stream_read_pos) return false;

    uint32_t tstart_ms = millis32();
    while (stream_fifo.Available() < n) {
        if (millis32() - tstart_ms > OTA_STREAM_DATA_TMO_MS) return false;
        pump();
    }

    for (uint32_t i = 0; i < n; i++) data[i] = stream_fifo.Get();
    stream_read_pos += n;
    stream_data_crc32 = ota_host_crc32(stream_data_crc32, data, n);
    return true;
}


bool tTxOtaRelay::stream_read(uint32_t stream_pos, uint8_t* data, uint8_t n)
{
    while (n) {
        if (stream_pos < head_len) {
            *data++ = head[stream_pos++];
            n--;
        } else
        if (stream_pos < head_len + header.data_length) {
            uint32_t data_pos = stream_pos - head_len;
            uint32_t data_n = (n < header.data_length - data_pos) ? n : header.data_length - data_pos;
            if (!data_read(data_pos, data, data_n)) return false;
            data += data_n;
            stream_pos += data_n;
            n -= data_n;
        } else {
            *data++ = tail[stream_pos++ - head_len - header.data_length];
            n--;
        }
    }
    return true;
}


uint8_t tTxOtaRelay::send_image(void)
{
    uint8_t payload[2 + OTA_BLOCK_SIZE];
    uint8_t* const response = buf + OTA_PACKET_HEADER_LEN; // status, payload

    // the receiver tells what it is and what it can do
    if (!command(OTA_CMD_HELLO, payload, 0, OTA_HOST_HELLO_RETRIES)) return OTA_HOST_ERR_NO_RECEIVER;
    if (len < OTA_PACKET_HEADER_LEN + 12 || response[0] != OTA_STATUS_OK) return OTA_HOST_ERR_NO_RECEIVER;
    uint8_t block_size = response[2];
    uint32_t target_id = ota_get_u32(response + 3);
    uint32_t app_size_max = ota_get_u32(response + 7);
    uint8_t rx_flags = response[11];
    if (response[1] != OTA_LOADER_VERSION || !block_size || block_size > OTA_BLOCK_SIZE) return OTA_HOST_ERR_NO_RECEIVER;

    if (header.target_id != target_id) return OTA_HOST_ERR_NO_IMAGE;
    if (header.image_length > app_size_max) return OTA_HOST_ERR_IMAGE;

    uint8_t flags = 0;
    head_len = tail_len = 0;
    if (header.flags & OTA_FILE_FLAG_DEFLATE) {
        if (rx_flags & OTA_FLAG_DEFLATE) { // zlib stream: cmf, flg, deflate stream, adler32 big endian
            flags = OTA_FLAG_DEFLATE;
            head[0] = 0x78; head[1] = 0xDA;
            head_len = 2;
            tail[0] = header.image_adler32 >> 24; tail[1] = header.image_adler32 >> 16;
            tail[2] = header.image_adler32 >> 8; tail[3] = header.image_adler32;
            tail_len = 4;
        } else
        if (rx_flags & OTA_FLAG_GZIP) { // gzip stream: header without options, deflate stream, crc32, length
            flags = OTA_FLAG_GZIP;
            memset(head, 0, 10);
            head[0] = 0x1F; head[1] = 0x8B; head[2] = 0x08; head[8] = 0x02; head[9] = 0xFF;
            head_len = 10;
            ota_put_u32(tail, header.image_crc32);
            ota_put_u32(tail + 4, header.image_length);
            tail_len = 8;
        } else {
            return OTA_HOST_ERR_IMAGE;
        }
    } else
    if (header.image_length != header.data_length) {
        return OTA_HOST_ERR_IMAGE;
    }
    uint32_t length = head_len + header.data_length + tail_len;

    ota_put_u32(payload, length);
    ota_put_u32(payload + 4, target_id);
    payload[8] = flags;
    ota_put_u32(payload + 9, header.image_length);
    if (!command(OTA_CMD_BEGIN, payload, 13, OTA_HOST_RETRIES)) return OTA_HOST_ERR_TRANSFER;
    if (response[0] != OTA_STATUS_OK) return OTA_HOST_ERR_REJECTED;

    stream_state = OTA_STREAM_STATE_TRANSFER; // the radio now sends the data

    // stop and wait, the receiver tells in each response which block it wants next
    // the receiver wants the crc32 of what was transferred at the end
    uint32_t stream_crc32 = 0;
    uint16_t block_num = (length + block_size - 1) / block_size;
    uint16_t block = 0;
    uint16_t block_loaded = UINT16_MAX; // the block which is in payload
    uint8_t n = 0;
    uint8_t fails = 0;
    while (block < block_num) {
        if (block != block_loaded) {
            if (block != (uint16_t)(block_loaded + 1)) return OTA_HOST_ERR_TRANSFER; // the data comes only once
            uint32_t stream_pos = (uint32_t)block * block_size;
            n = (length - stream_pos < block_size) ? length - stream_pos : block_size;
            payload[0] = block;
            payload[1] = block >> 8;
            if (!stream_read(stream_pos, payload + 2, n)) return OTA_HOST_ERR_NO_DATA;
            stream_crc32 = ota_host_crc32(stream_crc32, payload + 2, n);
            block_loaded = block;
        }
        if (!command(OTA_CMD_DATA, payload, 2 + n, 1)) {
            if (++fails > OTA_HOST_RETRIES) return OTA_HOST_ERR_TRANSFER;
            continue;
        }
        if (response[0] != OTA_STATUS_OK) return OTA_HOST_ERR_REJECTED;
        uint16_t next_block = response[1] | ((uint16_t)response[2] << 8);
        if (next_block == block + 1) {
            fails = 0;
        } else
        if (++fails > OTA_HOST_RETRIES) {
            return OTA_HOST_ERR_TRANSFER;
        }
        block = next_block;
    }

    // the data can't be checked before, as it comes while we go
    if (stream_data_crc32 != header.data_crc32) return OTA_HOST_ERR_IMAGE;

    // the receiver checks it, and reboots if it is ok
    ota_put_u32(payload, stream_crc32);
    if (!command(OTA_CMD_END, payload, 4, OTA_HOST_RETRIES)) return OTA_HOST_ERR_TRANSFER;
    if (response[0] != OTA_STATUS_OK) return OTA_HOST_ERR_REJECTED;

    return OTA_HOST_OK;
}


// a frame from the radio: offset[3], n, data[n]
void tTxOtaRelay::StreamFrame(const uint8_t* payload, uint8_t payload_len)
{
    if (!payload) { stream_crc_errors++; return; }
    if (payload_len < 4 || payload_len != 4 + payload[3]) return;
    uint32_t offset = payload[0] | ((uint32_t)payload[1] << 8) | ((uint32_t)payload[2] << 16);
    uint8_t n = payload[3];
    const uint8_t* data = payload + 4;

    if (offset == OTA_STREAM_START) {
        if (stream_state != OTA_STREAM_STATE_IDLE || n != sizeof(tOtaFileHeader)) return; // is repeated until we answer
        memcpy(&header, data, sizeof(tOtaFileHeader));
        if (header.magic != OTA_FILE_MAGIC || !header.data_length ||
            header.check != ota_host_crc32(0, (uint8_t*)&header, sizeof(tOtaFileHeader) - 4)) return;
        stream_fifo.Init();
        stream_rx_offset = 0;
        stream_gap_offset = UINT32_MAX;
        stream_nack_seq = 0;
        stream_crc_errors = 0;
        stream_rejected = 0;
        stream_state = OTA_STREAM_STATE_STARTING;
        stream_start_request = true;
        return;
    }

    if (stream_state != OTA_STREAM_STATE_TRANSFER) return;

    if (offset != stream_rx_offset) {
        // a frame got lost if it is ahead, tell it once, what is on its way comes also
        if (offset > stream_rx_offset && stream_gap_offset != stream_rx_offset) {
            stream_gap_offset = stream_rx_offset;
            stream_nack_seq++;
        }
        stream_rejected++;
        return;
    }
    if (!n || offset + n > header.data_length || !stream_fifo.HasSpace(n)) { stream_rejected++; return; }

    stream_fifo.PutBuf((void*)data, n);
    stream_rx_offset += n;
}


// true once when the radio has sent the header, the update is then to be started
bool tTxOtaRelay::StreamStartRequested(void)
{
    if (!stream_start_request) return false;
    stream_start_request = false;
    return true;
}


// serves the radio while we are busy with the receiver
void tTxOtaRelay::pump(void)
{
    if (!stream_running) return;

    crsf.Do();

    uint8_t task;
    if (!crsf.TelemetryUpdate(&task, 20)) return; // true when the radio has sent a frame, so we can send one

    uint32_t tnow_ms = millis32();
    if (tnow_ms - stream_status_tlast_ms < OTA_STREAM_STATUS_PERIOD_MS) return;
    stream_status_tlast_ms = tnow_ms;

    uint8_t frame[1 + MBRIDGE_CMD_RX_OTA_STATUS_LEN];
    tMBridgeRxOtaStatus* status = (tMBridgeRxOtaStatus*)(frame + 1);
    memset(frame, 0, sizeof(frame));
    frame[0] = MBRIDGE_COMMANDPACKET_STX + MBRIDGE_CMD_RX_OTA_STATUS;
    status->state = stream_state;
    status->result = stream_result;
    status->nack_seq = stream_nack_seq;
    status->crc_errors = stream_crc_errors;
    status->rejected = stream_rejected;
    status->rx_offset = stream_rx_offset;
    status->room = OTA_STREAM_FIFO_SIZE - 1 - stream_fifo.Available();
    crsf.SendMBridgeFrame(frame, sizeof(frame));
}


// blocks until the receiver has the image or it failed, the caller has to restart the controller afterwards
// StreamFrame() has taken the header before
uint8_t tTxOtaRelay::RunStream(uint32_t sx_freq_reg, uint16_t _session_id)
{
    session_id = _session_id;
    stream_running = true;
    stream_read_pos = 0;
    stream_data_crc32 = 0;
    stream_result = 0;
    start(sx_freq_reg);

    uint8_t result = send_image();

    stream_result = 1 + result;
    stream_state = OTA_STREAM_STATE_DONE;
    uint32_t tstart_ms = millis32();
    while (millis32() - tstart_ms < OTA_STREAM_DONE_LINGER_MS) pump();

    stream_state = OTA_STREAM_STATE_IDLE;
    stream_running = false;
    active = false;
    return result;
}

#endif // USE_RX_OTA_STREAM


#endif // OTA_RELAY_TX_H
