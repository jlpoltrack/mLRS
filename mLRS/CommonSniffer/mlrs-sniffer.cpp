//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// mLRS Listen-Only Sniffer
// 2026-03-08
//*******************************************************
// passive device that monitors OTA frames and outputs
// serial payloads over UART.
// never transmits — completely silent on the air.
//
// the Tx is the master clock. within each frame period:
//   1. Tx frame arrives → captured, rxclock.Reset(), re-enter RX
//   2. Rx response arrives → captured, now hop to next freq
//   3. if Rx response is missed, doPostReceive acts as fallback
//
// Rx-direction payloads (uplink) → serial port (UARTB, pins 21/20)
// Tx-direction payloads (downlink) → out port (UART, pin 19)


#define DBG_MAIN(x)
#define DBG_MAIN_SLIM(x)
#define DEBUG_ENABLED
#define FAIL_ENABLED


// irq priorities — same as Rx
#define CLOCK_IRQ_PRIORITY          10
#define UARTB_IRQ_PRIORITY          11 // serial
#define UART_IRQ_PRIORITY           12 // out pin
#define UARTF_IRQ_PRIORITY          11 // debug
#define SX_DIO_EXTI_IRQ_PRIORITY    13
#define SX2_DIO_EXTI_IRQ_PRIORITY   13
#define SWUART_TIM_IRQ_PRIORITY      9
#define FDCAN_IRQ_PRIORITY          14

#include "../Common/common_conf.h"
#include "../Common/common_types.h"

#if defined ESP8266 || defined ESP32

#include "../Common/hal/esp-glue.h"
#include "../modules/stm32ll-lib/src/stdstm32.h"
#include "../Common/esp-lib/esp-peripherals.h"
#include "../Common/esp-lib/esp-mcu.h"
#include "../Common/esp-lib/esp-stack.h"
#include "../Common/hal/hal.h"
#include "../Common/esp-lib/esp-delay.h"
#include "../Common/esp-lib/esp-eeprom.h"
#include "../Common/esp-lib/esp-spi.h"
#ifdef USE_SERIAL
#include "../Common/esp-lib/esp-uartb.h"
#endif
#include "../Common/esp-lib/esp-uart.h" // second UART (out port) for rx-direction payloads
#ifdef USE_DEBUG
#ifdef DEVICE_HAS_DEBUG_SWUART
#include "../Common/esp-lib/esp-uart-sw.h"
#else
#include "../Common/esp-lib/esp-uartf.h"
#endif
#endif
#include "../Common/hal/esp-timer.h"
#include "../Common/hal/esp-powerup.h"
#include "../Common/hal/esp-rxclock.h"

#else

#include "../Common/hal/glue.h"
#include "../modules/stm32ll-lib/src/stdstm32.h"
#include "../modules/stm32ll-lib/src/stdstm32-peripherals.h"
#include "../modules/stm32ll-lib/src/stdstm32-mcu.h"
#include "../modules/stm32ll-lib/src/stdstm32-dac.h"
#include "../modules/stm32ll-lib/src/stdstm32-stack.h"
#ifdef STM32WL
#include "../modules/stm32ll-lib/src/stdstm32-subghz.h"
#endif
#include "../Common/hal/hal.h"
#include "../modules/stm32ll-lib/src/stdstm32-delay.h"
#include "../modules/stm32ll-lib/src/stdstm32-eeprom.h"
#include "../modules/stm32ll-lib/src/stdstm32-spi.h"
#ifdef USE_SX2
#include "../modules/stm32ll-lib/src/stdstm32-spib.h"
#endif
#ifdef USE_SERIAL
#include "../modules/stm32ll-lib/src/stdstm32-uartb.h"
#endif
#include "../modules/stm32ll-lib/src/stdstm32-uart.h" // second UART (out port) for rx-direction payloads
#ifdef USE_DEBUG
#ifdef DEVICE_HAS_DEBUG_SWUART
#include "../modules/stm32ll-lib/src/stdstm32-uart-sw.h"
#else
#include "../modules/stm32ll-lib/src/stdstm32-uartf.h"
#endif
#endif
#ifdef USE_I2C
#include "../modules/stm32ll-lib/src/stdstm32-i2c.h"
#endif
#include "../Common/hal/timer.h"
#include "../CommonRx/powerup.h"
#include "../CommonRx/rxclock.h"

#endif //#if defined ESP8266 || defined ESP32

#include "../Common/sx-drivers/sx12xx.h"
#include "../Common/mavlink/fmav.h"
#include "../Common/mavlink/fmav_extension.h"
#ifdef USE_FEATURE_MAVLINKX
#include "../Common/thirdparty/mavlinkx.h"
#include "../Common/protocols/msp_protocol.h"
#include "../Common/thirdparty/mspx.h"
#endif
#include "../Common/setup.h"
#include "../Common/common.h"


//-------------------------------------------------------
// globals
//-------------------------------------------------------

tRxClock rxclock;
tPowerupCounter powerup;

// required by bind.h
void clock_reset(void) { rxclock.Reset(); }


//-------------------------------------------------------
// Init
//-------------------------------------------------------

void init_hw(void)
{
    __disable_irq();

    delay_init();
    systembootloader_init();
    timer_init();

    leds_init();
    button_init();

    serial.Init();

#ifdef DEVICE_HAS_OUT
    out_init_gpio();
    out_set_normal();
#endif
    uart_init_isroff();

    dbg.Init();

    setup_init();

    sx.Init();
    sx2.Init();

    __enable_irq();
}


//-------------------------------------------------------
// SX12xx ISR
//-------------------------------------------------------

volatile uint16_t irq_status;
volatile uint16_t irq2_status;

IRQHANDLER(
void SX_DIO_EXTI_IRQHandler(void)
{
    sx_dio_exti_isr_clearflag();
    irq_status = sx.GetAndClearIrqStatus(SX_IRQ_ALL);
    if (irq_status & SX_IRQ_RX_DONE) {
        uint16_t sync_word;
        sx.ReadBuffer(0, (uint8_t*)&sync_word, 2);
        if (sync_word != Config.FrameSyncWord) irq_status = 0;
    }
})
#ifdef USE_SX2
IRQHANDLER(
void SX2_DIO_EXTI_IRQHandler(void)
{
    sx2_dio_exti_isr_clearflag();
    irq2_status = sx2.GetAndClearIrqStatus(SX2_IRQ_ALL);
    if (irq2_status & SX2_IRQ_RX_DONE) {
        uint16_t sync_word;
        sx2.ReadBuffer(0, (uint8_t*)&sync_word, 2);
        if (sync_word != Config.FrameSyncWord) irq2_status = 0;
    }
})
#endif


//-------------------------------------------------------
// frame buffer
//-------------------------------------------------------

uint8_t sniffer_buf[FRAME_TX_RX_LEN];


//-------------------------------------------------------
// serial link mode decoder
//-------------------------------------------------------
// lightweight decoder that parses OTA payload bytes through
// the configured serial link mode (mavlinkX, mavlink, mspX,
// or transparent) and outputs standard protocol frames.

#define SNIFFER_BUF_SIZE  300 // larger than max mavlink frame (280)

typedef void (*tSnifferOutputFn)(uint8_t* buf, uint16_t len);

// output wrappers for the two serial ports
void sniffer_output_uart(uint8_t* buf, uint16_t len) { uart_putbuf(buf, len); }
void sniffer_output_serial(uint8_t* buf, uint16_t len) { serial.putbuf(buf, len); }

class tSnifferDecoder
{
  public:
    void Init(tSnifferOutputFn fn)
    {
        output_fn = fn;
        fmav_init();
        mav_status = {};
#ifdef USE_FEATURE_MAVLINKX
        fmavX_init();
        fmavX_config_compression((Config.Mode == MODE_19HZ || Config.Mode == MODE_19HZ_7X) ? 1 : 0);
        saved_fmavx = fmavx_status; // save clean state for this instance
        msp_status = {};
#endif
    }

    void Reset(void)
    {
        fmav_parse_reset(&mav_status);
#ifdef USE_FEATURE_MAVLINKX
        // reset mavlinkX compression state — stale bit-tracking from missed
        // frames causes the decompressor to produce garbage
        fmavX_init();
        fmavX_config_compression((Config.Mode == MODE_19HZ || Config.Mode == MODE_19HZ_7X) ? 1 : 0);
        saved_fmavx = fmavx_status; // save clean state for this instance
        msp_parse_reset(&msp_status);
#endif
    }

    // decode payload and output via the configured output function
    void PutBuf(uint8_t* const buf, uint8_t len)
    {
        if (!len) return;

#ifdef USE_FEATURE_MAVLINKX
        // swap in this instance's mavlinkX state (fmavx_status is a global
        // shared by all callers — each decoder needs its own copy)
        fmavx_status_t tmp = fmavx_status;
        fmavx_status = saved_fmavx;
#endif

        if (SERIAL_LINK_MODE_IS_MAVLINK(Setup.Rx.SerialLinkMode)) {
            for (uint8_t i = 0; i < len; i++) decode_mavlink(buf[i]);
        }
#ifdef USE_FEATURE_MAVLINKX
        else if (SERIAL_LINK_MODE_IS_MSP(Setup.Rx.SerialLinkMode)) {
            for (uint8_t i = 0; i < len; i++) decode_msp(buf[i]);
        }
#endif
        else {
            // transparent mode — raw passthrough
            output_fn(buf, len);
        }

#ifdef USE_FEATURE_MAVLINKX
        // swap out — save this instance's state, restore previous
        saved_fmavx = fmavx_status;
        fmavx_status = tmp;
#endif
    }

  private:
    tSnifferOutputFn output_fn;
    fmav_status_t mav_status;
    uint8_t mav_buf[SNIFFER_BUF_SIZE];
    fmav_message_t mav_msg;
#ifdef USE_FEATURE_MAVLINKX
    fmavx_status_t saved_fmavx; // per-instance mavlinkX state
    msp_status_t msp_status;
    msp_message_t msp_msg;
#endif
    uint8_t _buf[SNIFFER_BUF_SIZE];

    void decode_mavlink(char c)
    {
        fmav_result_t result;
#ifdef USE_FEATURE_MAVLINKX
        if (Setup.Rx.SerialLinkMode == SERIAL_LINK_MODE_MAVLINK_X) {
            fmavX_parse_and_checkX_to_frame_buf(&result, mav_buf, &mav_status, c);
        } else {
            fmav_parse_and_check_to_frame_buf(&result, mav_buf, &mav_status, c);
        }
#else
        fmav_parse_and_check_to_frame_buf(&result, mav_buf, &mav_status, c);
#endif
        if (result.res == FASTMAVLINK_PARSE_RESULT_OK) {
            fmav_frame_buf_to_msg(&mav_msg, &result, mav_buf);
            uint16_t frame_len = fmav_msg_to_frame_buf(_buf, &mav_msg);
            output_fn(_buf, frame_len);
        }
    }

#ifdef USE_FEATURE_MAVLINKX
    void decode_msp(char c)
    {
        if (msp_parseX_to_msg(&msp_msg, &msp_status, c)) {
            uint16_t frame_len = msp_msg_to_frame_buf(_buf, &msp_msg);
            output_fn(_buf, frame_len);
        }
    }
#endif
};

tSnifferDecoder decoder_tx; // Tx→Rx direction payloads → UART (P19)
tSnifferDecoder decoder_rx; // Rx→Tx direction payloads → serial (P21/P20)


uint8_t tx_seq_no_last;    // last seen Tx frame seq_no (3-bit, for ARQ dedup)
uint8_t rx_seq_no_last;    // last seen Rx frame seq_no (3-bit, for ARQ dedup)

//-------------------------------------------------------
// frame processing
//-------------------------------------------------------

// returns RX_STATUS and sets *got_tx true if it decoded as a Tx frame
uint8_t do_receive(bool* got_tx)
{
    sx.ReadBuffer(0, sniffer_buf, FRAME_TX_RX_LEN);

    *got_tx = false;

    // try as Tx frame (Tx→Rx direction)
    tTxFrame* txf = (tTxFrame*)sniffer_buf;
    uint8_t res = check_txframe(txf);
    if (res == CHECK_OK) {
        rxclock.Reset(); // sync timeout to Tx arrival
        *got_tx = true;
        // only feed payload if seq_no changed (skip ARQ retransmissions)
        if (txf->status.seq_no != tx_seq_no_last) {
            tx_seq_no_last = txf->status.seq_no;
            decoder_tx.PutBuf(txf->payload, txf->status.payload_len);
        }
        return RX_STATUS_VALID;
    }
    if (res == CHECK_ERROR_CRC) {
        rxclock.Reset();
        *got_tx = true;
        return RX_STATUS_CRC1_VALID;
    }

    // try as Rx frame (Rx→Tx direction)
    tRxFrame* rxf = (tRxFrame*)sniffer_buf;
    res = check_rxframe(rxf);
    if (res == CHECK_OK) {
        // only feed payload if seq_no changed (skip ARQ retransmissions)
        if (rxf->status.seq_no != rx_seq_no_last) {
            rx_seq_no_last = rxf->status.seq_no;
            decoder_rx.PutBuf(rxf->payload, rxf->status.payload_len);
        }
        return RX_STATUS_VALID;
    }

    return RX_STATUS_INVALID;
}


//##############################################################################################################
//*******************************************************
// MAIN routine
//*******************************************************

uint16_t tick_1hz;

uint8_t link_state;
uint8_t connect_state;
uint16_t connect_tmo_cnt;
uint8_t connect_sync_cnt;
uint8_t connect_listen_cnt;

uint8_t link_rx1_status;
bool got_tx_frame;         // true when Tx frame received this period, waiting for Rx
uint32_t tx_frame_tick;    // uwTick when the Tx frame was received

uint32_t tx_frames_cnt;
uint32_t rx_frames_cnt;


bool connected(void)
{
    return (connect_state == CONNECT_STATE_CONNECTED);
}


//-------------------------------------------------------
// connection state machine — called once per frame period
//-------------------------------------------------------

void do_post_receive_state(bool valid_frame_received)
{
    if (valid_frame_received) {
        switch (connect_state) {
        case CONNECT_STATE_LISTEN:
            connect_state = CONNECT_STATE_SYNC;
            connect_sync_cnt = 0;
            break;
        case CONNECT_STATE_SYNC:
            connect_sync_cnt++;
            if (connect_sync_cnt >= CONNECT_SYNC_CNT) {
                connect_state = CONNECT_STATE_CONNECTED;
            }
            break;
        }
        connect_tmo_cnt = CONNECT_TMO_SYSTICKS;
    }
}


// called when it's time to finish this period and hop to next freq
void do_hop(void)
{
    bool valid = (link_rx1_status > RX_STATUS_INVALID);
    do_post_receive_state(valid);

    sx.SetToIdle();
    sx2.SetToIdle();
    link_state = LINK_STATE_RECEIVE;
}


void main_loop(void)
{
INITCONTROLLER_ONCE
    stack_check_init();
RESTARTCONTROLLER
    init_hw();
    DBG_MAIN(dbg.puts("\n\n\nSniffer\n\n");)

    // setup_init() has run so Config is valid
    serial.SetBaudRate(Config.SerialBaudrate);
    uart_setprotocol(Config.SerialBaudrate, XUART_PARITY_NO, UART_STOPBIT_1);

    leds.Init();

    // start up sx
    if (!sx.isOk()) { FAILALWAYS(BLINK_RD_GR_OFF, "Sx not ok"); }
    if (!sx2.isOk()) { FAILALWAYS(BLINK_GR_RD_OFF, "Sx2 not ok"); }
    irq_status = irq2_status = 0;
    IF_SX(sx.StartUp(&Config.Sx));
    IF_SX2(sx2.StartUp(&Config.Sx2));

    fhss.Init(&Config.Fhss, &Config.Fhss2);
    fhss.Start();

    decoder_tx.Init(sniffer_output_uart);
    decoder_rx.Init(sniffer_output_serial);

    sx.SetRfFrequency(fhss.GetCurrFreq());
    sx2.SetRfFrequency(fhss.GetCurrFreq2());

    link_state = LINK_STATE_RECEIVE;
    connect_state = CONNECT_STATE_LISTEN;
    connect_tmo_cnt = 0;
    connect_sync_cnt = 0;
    connect_listen_cnt = 0;
    link_rx1_status = RX_STATUS_NONE;
    got_tx_frame = false;
    tx_frame_tick = 0;
    tx_seq_no_last = 0xFF; // invalid — ensures first frame always passes dedup
    rx_seq_no_last = 0xFF;

    tx_frames_cnt = 0;
    rx_frames_cnt = 0;

    rxclock.Init(2 * Config.frame_rate_ms);

    tick_1hz = 0;
    resetSysTask();
INITCONTROLLER_END

    //-- SysTask handling (runs every 1 ms)

    if (doSysTask()) {

        if (connect_tmo_cnt) {
            connect_tmo_cnt--;
        }

        leds.Tick_ms(connected());

        // software timer: got Tx frame but Rx frame hasn't arrived
        // after half a frame period — Rx was missed, hop now
        if (link_state == LINK_STATE_RECEIVE_WAIT &&
            got_tx_frame &&
            (uwTick - tx_frame_tick) >= (Config.frame_rate_ms / 2)) {
            decoder_tx.Reset();
            decoder_rx.Reset();
            do_hop();
        }

        DECc(tick_1hz, SYSTICK_DELAY_MS(1000));

        if (!tick_1hz) {
            dbg.puts("sniff: tx=");
            dbg.puts(u16toBCD_s(tx_frames_cnt));
            dbg.puts(" rx=");
            dbg.puts(u16toBCD_s(rx_frames_cnt));
            dbg.puts(" ");
            dbg.puts(connected() ? "CONN" : (connect_state == CONNECT_STATE_SYNC) ? "SYNC" : "LIST");
            dbg.puts("\n");
            tx_frames_cnt = 0;
            rx_frames_cnt = 0;
        }
    }

    //-- SX handling: enter RX mode

    switch (link_state) {
    case LINK_STATE_RECEIVE:
        // hop only when synced/connected — in LISTEN we hop via connect_listen_cnt
        if (connect_state >= CONNECT_STATE_SYNC) {
            fhss.HopToNext();
        }
        sx.SetRfFrequency(fhss.GetCurrFreq());
        sx2.SetRfFrequency(fhss.GetCurrFreq2());
        IF_ANTENNA1(sx.SetToRx());
        IF_ANTENNA2(sx2.SetToRx());
        link_state = LINK_STATE_RECEIVE_WAIT;
        link_rx1_status = RX_STATUS_NONE;
        got_tx_frame = false;
        tx_frame_tick = 0;
        doPostReceive = false; // discard stale timeout from previous cycle
        irq_status = irq2_status = 0;
        break;
    }//end of switch(link_state)

    //-- SX handling: process RX_DONE
    //
    // Tx frame → decode, re-enter RX (wait for Rx frame)
    // Rx frame → decode, hop immediately (period is over)

IF_SX(
    if (irq_status) {
        if (link_state == LINK_STATE_RECEIVE_WAIT) {
            if (irq_status & SX_IRQ_RX_DONE) {
                irq_status = 0;

                bool is_tx = false;
                uint8_t rx_status = do_receive(&is_tx);

                // track best status for sync purposes
                if (rx_status > link_rx1_status) {
                    link_rx1_status = rx_status;
                }

                if (rx_status == RX_STATUS_VALID && is_tx) {
                    // Tx frame — record it, re-enter RX for the Rx frame
                    tx_frames_cnt++;
                    got_tx_frame = true;
                    tx_frame_tick = uwTick;
                    if (!tx_frame_tick) tx_frame_tick = 1; // avoid 0 sentinel
                    IF_ANTENNA1(sx.SetToRx());
                    IF_ANTENNA2(sx2.SetToRx());
                    irq_status = 0;
                } else if (rx_status == RX_STATUS_VALID) {
                    // Rx frame — period is over, hop now
                    rx_frames_cnt++;
                    do_hop();
                } else {
                    // invalid frame — re-enter RX
                    IF_ANTENNA1(sx.SetToRx());
                    IF_ANTENNA2(sx2.SetToRx());
                    irq_status = 0;
                }
            }
        }

        if (irq_status) {
            // unexpected irq — recover
            irq_status = 0;
            link_state = LINK_STATE_RECEIVE;
            link_rx1_status = RX_STATUS_NONE;
        }
    }//end of if(irq_status)
);

    //-- doPostReceive: long-term timeout for completely missed frames
    //
    // fires at 2× frame_rate_ms after the last Tx frame (via rxclock.Reset),
    // or free-running during LISTEN scanning.
    // the "got Tx, missed Rx" case is handled by the software timer above.

    if (doPostReceive) {
        doPostReceive = false;

        // if we already got a Tx frame this period, this is a stale
        // rxclock event from a previous period — ignore it
        if (got_tx_frame) {
            return;
        }

        // reset parser state so partial parses don't carry across gaps
        decoder_tx.Reset();
        decoder_rx.Reset();

        // LISTEN: hop slowly through frequencies
        if (connect_state == CONNECT_STATE_LISTEN) {
            connect_listen_cnt++;
            if (connect_listen_cnt >= CONNECT_LISTEN_HOP_CNT) {
                fhss.HopToNext();
                connect_listen_cnt = 0;
            }
            sx.SetToIdle();
            sx2.SetToIdle();
            link_state = LINK_STATE_RECEIVE;
            return;
        }

        // SYNC/CONNECTED: timeout — drop to listen
        if ((connect_state >= CONNECT_STATE_SYNC) && !connect_tmo_cnt) {
            connect_state = CONNECT_STATE_LISTEN;
            connect_listen_cnt = 0;
            sx.SetToIdle();
            sx2.SetToIdle();
            link_state = LINK_STATE_RECEIVE;
            return;
        }

        // SYNC/CONNECTED: missed Rx frame — hop
        if (connect_state >= CONNECT_STATE_SYNC) {
            do_hop();
        }

        return;
    }//end of if(doPostReceive)

}//end of main_loop
