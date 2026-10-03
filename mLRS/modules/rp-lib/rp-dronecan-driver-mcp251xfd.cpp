//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// DroneCAN Driver for RP2040/RP2350 using a MCP2518FD/MCP251863
// external CAN FD controller on SPI, for use with libcanard
//*******************************************************
// implements the same dc_hal_* API as rp-dronecan-driver.cpp.
// all SPI traffic is done on core 0 in dc_hal_poll_core0(), which
// has to be called as fast as possible. the radio core (core 1) only
// touches the ring buffers. no interrupt is used, the level of the
// chip's INT pin (the main one, not INT0 or INT1) tells if there are
// frames to read, so there is no SPI traffic when there is nothing to do.
// expects a 40 MHz crystal.
//*******************************************************
#if (defined ARDUINO_ARCH_RP2040 || defined ARDUINO_ARCH_RP2350) && defined DEVICE_HAS_DRONECAN_FD

#include <string.h>
#include <stdint.h>
#include <hardware/spi.h>
#include <hardware/gpio.h>
#include <hardware/timer.h>
#include <hardware/sync.h>

#include "rp-dronecan-driver.h"

#if !CANARD_ENABLE_CANFD
#error CANARD_ENABLE_CANFD must be enabled for the MCP251xFD driver !
#endif

// same as on STM32
#define DC_HAL_ACCEPTANCE_FILTERS_NUM_MAX  8


//-------------------------------------------------------
// MCP251xFD registers
//-------------------------------------------------------

#define MCP_CMD_RESET             0x00
#define MCP_CMD_WRITE             0x20
#define MCP_CMD_READ_CRC          0xB0

#define MCP_REG_CON               0x000
#define MCP_REG_NBTCFG            0x004
#define MCP_REG_DBTCFG            0x008
#define MCP_REG_TDC               0x00C
#define MCP_REG_INT               0x01C
#define MCP_REG_FIFOCON(n)        (0x050 + 12 * (n)) // n = 1...31
#define MCP_REG_FIFOSTA(n)        (0x054 + 12 * (n)) // is followed by FIFOUA
#define MCP_REG_FLTCON(n)         (0x1D0 + (n))
#define MCP_REG_FLTOBJ(n)         (0x1F0 + 8 * (n))
#define MCP_REG_MASK(n)           (0x1F4 + 8 * (n))
#define MCP_REG_OSC               0xE00
#define MCP_REG_ECCCON            0xE0C

#define MCP_RAM_ADDR              0x400
#define MCP_RAM_SIZE              2048

#define MCP_CON_ISOCRCEN          0x00000020
#define MCP_CON_RTXAT             0x00010000 // FIFOCON.TXAT is used
#define MCP_CON_PXEDIS            0x00000040
#define MCP_CON_OPMOD_SHIFT       21
#define MCP_CON_REQOP_SHIFT       24
#define MCP_CON_TXBWS_2BITS       0x10000000 // pause between own frames, as TransmitPause on STM32

#define MCP_OPMODE_NORMAL_FD      0 // mixed, receives both classic CAN and CAN FD frames
#define MCP_OPMODE_CONFIG         4
#define MCP_OPMODE_NORMAL_CAN20   6

// bit timings for 40 MHz, prescaler 1
// nominal 90% sample point as on STM32/ArduPilot, data 80% (STM32 has 75%, not possible with 10 tq)
#define MCP_NBTCFG_1MBPS          0x00220303 // 1 + 35 + 4 tq, sjw 4
#define MCP_DBTCFG_4MBPS          0x00060101 // 1 + 7 + 2 tq, sjw 2, same data bit rate as on STM32G4
#define MCP_TDC_4MBPS             0x00020700 // auto, TDCO = 7

#define MCP_INT_RXIE              0x00020000


#define MCP_OSC_OSCRDY            0x00000400

#define MCP_ECCCON_ECCEN          0x01

#define MCP_FIFOCON_TFNRFNIE      0x00000001
#define MCP_FIFOCON_TXEN          0x00000080
#define MCP_FIFOCON_UINC          0x00000100
#define MCP_FIFOCON_TXREQ         0x00000200
#define MCP_FIFOCON_FRESET        0x00000400
#define MCP_FIFOCON_TXAT_3        0x00200000 // three retransmission attempts
#define MCP_FIFOCON_FSIZE_SHIFT   24
#define MCP_FIFOCON_PLSIZE_64     0xE0000000

#define MCP_FIFOSTA_TFNRFNIF      0x01 // tx fifo not full, rx fifo not empty
#define MCP_FIFOSTA_TFERFFIF      0x04 // tx fifo empty, rx fifo full
#define MCP_FIFOSTA_RXOVIF        0x08
#define MCP_FIFOSTA_TXATIF        0x10

#define MCP_FLTCON_FLTEN          0x80
#define MCP_FLTOBJ_EXIDE          0x40000000 // match only extended ids
#define MCP_MASK_MIDE             0x40000000

#define MCP_OBJ_SID_MASK          0x000007FF
#define MCP_OBJ_EID_MASK          0x0003FFFF
#define MCP_OBJ_EID_SHIFT         11
#define MCP_OBJ_DLC_MASK          0x0000000F
#define MCP_OBJ_IDE               0x00000010
#define MCP_OBJ_RTR               0x00000020
#define MCP_OBJ_BRS               0x00000040
#define MCP_OBJ_FDF               0x00000080

#define MCP_OBJ_HEADER_SIZE       8
#define MCP_OBJ_SIZE              (MCP_OBJ_HEADER_SIZE + CANARD_CANFD_FRAME_MAX_DATA_LEN)

// chip fifos, 8 + 10 + 10 objects of 72 bytes take 2016 of the 2048 bytes ram
// the two rx fifos correspond to FIFO0, FIFO1 on STM32
#define MCP_TX_FIFO               1
#define MCP_TX_FIFO_SIZE          8
#define MCP_RX_FIFO0              2
#define MCP_RX_FIFO1              3
#define MCP_RX_FIFO_SIZE          10


//-------------------------------------------------------
// ring buffers
//-------------------------------------------------------
// RX: dc_hal_poll_core0 (core 0) writes, core 1 reads
// TX: core 1 writes, dc_hal_poll_core0 (core 0) reads
// both are single-producer single-consumer across cores, see rp-dronecan-driver.cpp
// for why the index updates are fenced

#define DC_HAL_RXBUFSIZE          32
#define DC_HAL_RXBUFSIZEMASK      (DC_HAL_RXBUFSIZE - 1)
#define DC_HAL_TXBUFSIZE          32
#define DC_HAL_TXBUFSIZEMASK      (DC_HAL_TXBUFSIZE - 1)

typedef struct
{
    CanardCANFrame frames[DC_HAL_RXBUFSIZE];
    volatile uint16_t writepos;
    volatile uint16_t readpos;
} tDcHalRxBuf;

typedef struct
{
    CanardCANFrame frames[DC_HAL_TXBUFSIZE];
    volatile uint16_t writepos;
    volatile uint16_t readpos;
} tDcHalTxBuf;

static tDcHalRxBuf dc_hal_rxbuf;
static tDcHalTxBuf dc_hal_txbuf;

static tDcHalStatistics dc_hal_stats;

static tDcHalMcpConfig dc_hal_config;
static uint8_t dc_hal_opmode; // the normal mode the chip is put into
static bool dc_hal_opmode_reached;
static uint32_t dc_hal_check_tlast_us;

// written by core 1, read by core 0 on start and on a filter request
static tDcHalAcceptanceFilterConfiguration dc_hal_filters[DC_HAL_ACCEPTANCE_FILTERS_NUM_MAX];
static uint8_t dc_hal_filter_num;
static volatile bool dc_hal_filter_request = false;

static volatile bool dc_hal_fd_frame_detected; // set once a FD frame has been seen, as on STM32
static spi_inst_t* dc_hal_spi;

// inter-core signaling: core 1 writes the start request, once, core 0 writes all other states
typedef enum
{
    DC_HAL_STATE_IDLE = 0,
    DC_HAL_STATE_START_REQUEST,
    DC_HAL_STATE_RUNNING,
    DC_HAL_STATE_FAILED,
} DC_HAL_STATE_ENUM;

static volatile uint8_t dc_hal_state = DC_HAL_STATE_IDLE;


static const uint8_t dlc_to_len[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

static uint8_t dlc_from_data_len(uint8_t data_len)
{
    for (uint8_t dlc = 0; dlc < 16; dlc++) {
        if (dlc_to_len[dlc] >= data_len) return dlc;
    }
    return 15;
}

// 29 bit id to the SID, EID layout of the chip's message and filter objects
static uint32_t mcp_obj_from_id(uint32_t id)
{
    id &= CANARD_CAN_EXT_ID_MASK;
    return (id >> 18) | ((id & MCP_OBJ_EID_MASK) << MCP_OBJ_EID_SHIFT);
}


//-------------------------------------------------------
// spi access, core 0 only
//-------------------------------------------------------

static void mcp_write(uint16_t addr, const void* data, uint16_t len)
{
    uint8_t cmd[2] = { (uint8_t)(MCP_CMD_WRITE | (addr >> 8)), (uint8_t)addr };
    gpio_put(dc_hal_config.cs_pin, 0);
    spi_write_blocking(dc_hal_spi, cmd, 2);
    spi_write_blocking(dc_hal_spi, (const uint8_t*)data, len);
    gpio_put(dc_hal_config.cs_pin, 1);
}

// poly 0x8005, msb first, the chip starts with 0xFFFF on nCS low
static uint16_t mcp_crc16_table[256];

static void mcp_crc16_init(void)
{
    for (uint16_t i = 0; i < 256; i++) {
        uint16_t crc = i << 8;
        for (uint8_t n = 0; n < 8; n++) crc = (crc & 0x8000) ? (crc << 1) ^ 0x8005 : (crc << 1);
        mcp_crc16_table[i] = crc;
    }
}

static uint16_t mcp_crc16(uint16_t crc, const uint8_t* data, uint16_t len)
{
    while (len--) crc = (crc << 8) ^ mcp_crc16_table[(crc >> 8) ^ *data++];
    return crc;
}

// all reads are crc checked, errata: a plain read can return wrong data for CiCON, CiFIFOSTAm
// and others. len must be a multiple of 4 for ram
// returns false if the crc does not match even with retries
static bool mcp_read(uint16_t addr, void* data, uint8_t len)
{
    // the length field is in bytes for registers, and in words for ram
    uint8_t n = (addr >= MCP_RAM_ADDR && addr < MCP_RAM_ADDR + MCP_RAM_SIZE) ? len / 4 : len;
    uint8_t cmd[3] = { (uint8_t)(MCP_CMD_READ_CRC | (addr >> 8)), (uint8_t)addr, n };
    for (uint8_t retry = 0; retry < 3; retry++) {
        uint8_t crc[2];
        gpio_put(dc_hal_config.cs_pin, 0);
        spi_write_blocking(dc_hal_spi, cmd, 3);
        spi_read_blocking(dc_hal_spi, 0, (uint8_t*)data, len);
        spi_read_blocking(dc_hal_spi, 0, crc, 2);
        gpio_put(dc_hal_config.cs_pin, 1);
        uint16_t crc16 = mcp_crc16(mcp_crc16(0xFFFF, cmd, 3), (uint8_t*)data, len);
        if (crc16 == (((uint16_t)crc[0] << 8) | crc[1])) return true;
    }
    dc_hal_stats.parse_error_count++;
    return false;
}

// registers are little endian, as is the rp
// reads as all ones on a crc failure, which no caller takes for a valid value
static uint32_t mcp_read32(uint16_t addr)
{
    uint32_t value;
    if (!mcp_read(addr, &value, 4)) return UINT32_MAX;
    return value;
}

static void mcp_write32(uint16_t addr, uint32_t value)
{
    mcp_write(addr, &value, 4);
}

static void mcp_write8(uint16_t addr, uint8_t value)
{
    mcp_write(addr, &value, 1);
}

static void mcp_reset(void)
{
    uint8_t cmd[2] = { MCP_CMD_RESET, 0 };
    gpio_put(dc_hal_config.cs_pin, 0);
    spi_write_blocking(dc_hal_spi, cmd, 2);
    gpio_put(dc_hal_config.cs_pin, 1);
}

static uint8_t mcp_opmode(void)
{
    return (mcp_read32(MCP_REG_CON) >> MCP_CON_OPMOD_SHIFT) & 0x07;
}


//-------------------------------------------------------
// chip configuration, core 0 only
//-------------------------------------------------------

// mirrors STM32: only extended ids, frames matching no filter are rejected
static void mcp_apply_filters(void)
{
    for (uint8_t n = 0; n < DC_HAL_ACCEPTANCE_FILTERS_NUM_MAX; n++) {
        mcp_write8(MCP_REG_FLTCON(n), 0); // a filter must be disabled to be changed
        if (n >= dc_hal_filter_num) continue;

        uint8_t fifo;
        if (dc_hal_filters[n].rx_fifo == DC_HAL_RX_FIFO0) {
            fifo = MCP_RX_FIFO0;
        } else
        if (dc_hal_filters[n].rx_fifo == DC_HAL_RX_FIFO1) {
            fifo = MCP_RX_FIFO1;
        } else {
            fifo = ((n & 0x01) == 0) ? MCP_RX_FIFO0 : MCP_RX_FIFO1;
        }
        mcp_write32(MCP_REG_FLTOBJ(n), mcp_obj_from_id(dc_hal_filters[n].id) | MCP_FLTOBJ_EXIDE);
        mcp_write32(MCP_REG_MASK(n), mcp_obj_from_id(dc_hal_filters[n].mask) | MCP_MASK_MIDE);
        mcp_write8(MCP_REG_FLTCON(n), MCP_FLTCON_FLTEN | fifo);
    }
}


// returns false if no chip is found
static bool mcp_configure(void)
{
    dc_hal_check_tlast_us = time_us_32();

    // the chip has to be in configuration mode to be reset
    mcp_write8(MCP_REG_CON + 3, MCP_OPMODE_CONFIG);
    uint32_t tstart_us = time_us_32();
    while (mcp_opmode() != MCP_OPMODE_CONFIG) {
        if ((time_us_32() - tstart_us) > 2000) break;
    }
    mcp_reset();

    // wait for the oscillator, takes about 3 ms. a floating miso reads as 0 or 0xFF
    tstart_us = time_us_32();
    while (1) {
        uint32_t osc = mcp_read32(MCP_REG_OSC);
        if ((osc & MCP_OSC_OSCRDY) && osc != UINT32_MAX) break;
        if ((time_us_32() - tstart_us) > 20000) return false;
    }
    if (mcp_opmode() != MCP_OPMODE_CONFIG) return false;

    // ecc needs the ram to be initialized
    uint8_t zeros[64] = {};
    for (uint16_t n = 0; n < MCP_RAM_SIZE; n += sizeof(zeros)) {
        mcp_write(MCP_RAM_ADDR + n, zeros, sizeof(zeros));
    }
    mcp_write8(MCP_REG_ECCCON, MCP_ECCCON_ECCEN);

    // no tx queue, no tx event fifo, iso crc, limited retransmissions
    mcp_write32(MCP_REG_CON, MCP_CON_TXBWS_2BITS |
        ((uint32_t)MCP_OPMODE_CONFIG << MCP_CON_REQOP_SHIFT) | MCP_CON_RTXAT | MCP_CON_PXEDIS | MCP_CON_ISOCRCEN);

    mcp_write32(MCP_REG_NBTCFG, MCP_NBTCFG_1MBPS);
    mcp_write32(MCP_REG_DBTCFG, MCP_DBTCFG_4MBPS);
    mcp_write32(MCP_REG_TDC, MCP_TDC_4MBPS);

    mcp_write32(MCP_REG_FIFOCON(MCP_TX_FIFO),
        MCP_FIFOCON_PLSIZE_64 | ((uint32_t)(MCP_TX_FIFO_SIZE - 1) << MCP_FIFOCON_FSIZE_SHIFT) |
        MCP_FIFOCON_TXAT_3 | MCP_FIFOCON_TXEN);

    // INT pin is low as long as a rx fifo is not empty
    uint32_t fifocon = MCP_FIFOCON_PLSIZE_64 | ((uint32_t)(MCP_RX_FIFO_SIZE - 1) << MCP_FIFOCON_FSIZE_SHIFT) |
        MCP_FIFOCON_TFNRFNIE;
    mcp_write32(MCP_REG_FIFOCON(MCP_RX_FIFO0), fifocon);
    mcp_write32(MCP_REG_FIFOCON(MCP_RX_FIFO1), fifocon);
    mcp_write32(MCP_REG_INT, MCP_INT_RXIE);

    mcp_apply_filters();

    // the chip enters the mode by itself as soon as it sees the bus idle
    dc_hal_opmode = (dc_hal_config.canfd) ? MCP_OPMODE_NORMAL_FD : MCP_OPMODE_NORMAL_CAN20;
    dc_hal_opmode_reached = false;
    mcp_write8(MCP_REG_CON + 3, (MCP_CON_TXBWS_2BITS >> 24) | dc_hal_opmode);

    return true;
}


//-------------------------------------------------------
// rx, core 0 loop
//-------------------------------------------------------

static void mcp_receive(uint8_t fifo, uint32_t sta, uint32_t ua)
{
    if (sta & MCP_FIFOSTA_RXOVIF) {
        dc_hal_stats.rx_overflow_count++;
        mcp_write8(MCP_REG_FIFOSTA(fifo), 0); // clears the flag
    }
    if (!(sta & MCP_FIFOSTA_TFNRFNIF)) return; // fifo empty

    // read header and the first 8 bytes, which is all there is for classic frames
    union {
        uint32_t r[2];
        uint8_t buf[MCP_OBJ_SIZE];
    } obj;
    uint16_t addr = MCP_RAM_ADDR + (ua & 0x0FFF);
    if (!mcp_read(addr, obj.buf, MCP_OBJ_HEADER_SIZE + 8)) return; // frame stays in the fifo

    bool fdf = (obj.r[1] & MCP_OBJ_FDF) != 0;
    uint8_t data_len = dlc_to_len[obj.r[1] & MCP_OBJ_DLC_MASK];
    // as on STM32, drop remote frames and classic frames with DLC > 8
    // the chip's filters cannot match on RTR, so this is done here
    bool drop = !fdf && ((obj.r[1] & MCP_OBJ_RTR) || data_len > 8);
    if (!drop && data_len > 8) {
        if (!mcp_read(addr + MCP_OBJ_HEADER_SIZE + 8, obj.buf + MCP_OBJ_HEADER_SIZE + 8, data_len - 8)) return;
    }
    mcp_write8(MCP_REG_FIFOCON(fifo) + 1, MCP_FIFOCON_UINC >> 8);
    if (drop) {
        dc_hal_stats.parse_error_count++;
        return;
    }
    if (fdf) dc_hal_fd_frame_detected = true;

    uint16_t wp = dc_hal_rxbuf.writepos;
    uint16_t next = (wp + 1) & DC_HAL_RXBUFSIZEMASK;
    if (next == dc_hal_rxbuf.readpos) {
        dc_hal_stats.rx_overflow_count++;
        return;
    }

    // filters only pass extended ids
    CanardCANFrame* frame = &dc_hal_rxbuf.frames[wp];
    frame->id = ((obj.r[0] & MCP_OBJ_SID_MASK) << 18) | ((obj.r[0] >> MCP_OBJ_EID_SHIFT) & MCP_OBJ_EID_MASK);
    frame->id |= CANARD_CAN_FRAME_EFF;
    memset(frame->data, 0, sizeof(frame->data));
    memcpy(frame->data, obj.buf + MCP_OBJ_HEADER_SIZE, data_len);
    frame->data_len = data_len;
    frame->iface_id = 0; // libcanard silently drops frames whose iface_id differs from the rx state's
    frame->canfd = fdf;

    dc_hal_stats.rx_total++;

    __mem_fence_release(); // slot must be visible to core 1 before writepos is
    dc_hal_rxbuf.writepos = next;
}


static void mcp_poll_rx(void)
{
    // bounded, so that tx gets its turn also with a saturated bus
    for (uint8_t n = 0; n < MCP_RX_FIFO_SIZE; n++) {
        if (gpio_get(dc_hal_config.int_pin)) return; // high, both rx fifos are empty

        // FIFOSTA, FIFOUA of rx fifo 0, FIFOCON, FIFOSTA, FIFOUA of rx fifo 1
        uint32_t regs[5];
        if (!mcp_read(MCP_REG_FIFOSTA(MCP_RX_FIFO0), regs, sizeof(regs))) return;
        mcp_receive(MCP_RX_FIFO0, regs[0], regs[1]);
        mcp_receive(MCP_RX_FIFO1, regs[3], regs[4]);
    }
}


static bool mcp_start(void)
{
    mcp_crc16_init();

    dc_hal_spi = (dc_hal_config.spi_num == 1) ? spi1 : spi0;

    gpio_init(dc_hal_config.cs_pin);
    gpio_put(dc_hal_config.cs_pin, 1);
    gpio_set_dir(dc_hal_config.cs_pin, GPIO_OUT);

    spi_init(dc_hal_spi, dc_hal_config.spi_hz); // 8 bits, mode 0, msb first
    gpio_set_function(dc_hal_config.sck_pin, GPIO_FUNC_SPI);
    gpio_set_function(dc_hal_config.mosi_pin, GPIO_FUNC_SPI);
    gpio_set_function(dc_hal_config.miso_pin, GPIO_FUNC_SPI);

    gpio_init(dc_hal_config.int_pin); // input
    gpio_pull_up(dc_hal_config.int_pin);

    return mcp_configure();
}


//-------------------------------------------------------
// tx, core 0 loop
//-------------------------------------------------------

static void mcp_poll_tx(void)
{
    while (dc_hal_txbuf.readpos != dc_hal_txbuf.writepos) {
        __mem_fence_acquire(); // do not read the slot before the writepos that published it

        // FIFOUA is not valid before the chip has left configuration mode
        if (!dc_hal_opmode_reached) {
            dc_hal_opmode_reached = (mcp_opmode() == dc_hal_opmode);
            if (!dc_hal_opmode_reached) return;
        }

        // FIFOCON, FIFOSTA, FIFOUA
        uint32_t regs[3];
        bool ok = mcp_read(MCP_REG_FIFOCON(MCP_TX_FIFO), regs, sizeof(regs));
        if (!ok) return;
        const uint32_t* sta_ua = &regs[1];

        // the chip gave up on a frame after three attempts, and has stopped sending. drop all
        // frames pending in the chip, similar to the abort on error on STM32
        if ((sta_ua[0] & MCP_FIFOSTA_TXATIF) ||
            (!(regs[0] & MCP_FIFOCON_TXREQ) && !(sta_ua[0] & MCP_FIFOSTA_TFERFFIF))) {
            mcp_write8(MCP_REG_FIFOCON(MCP_TX_FIFO) + 1, MCP_FIFOCON_FRESET >> 8);
            uint32_t tstart_us = time_us_32();
            while (mcp_read32(MCP_REG_FIFOCON(MCP_TX_FIFO)) & MCP_FIFOCON_FRESET) {
                if ((time_us_32() - tstart_us) > 1000) break;
            }
            mcp_write8(MCP_REG_FIFOSTA(MCP_TX_FIFO), 0); // clears the flag
            dc_hal_stats.tx_abort_count++;
            return; // continue next poll
        }

        if (!(sta_ua[0] & MCP_FIFOSTA_TFNRFNIF)) return; // chip tx fifo full, retry next poll

        uint16_t rp = dc_hal_txbuf.readpos;
        const CanardCANFrame* frame = &dc_hal_txbuf.frames[rp];

        union {
            uint32_t t[2];
            uint8_t buf[MCP_OBJ_SIZE];
        } obj = {};
        uint8_t dlc = dlc_from_data_len(frame->data_len);
        obj.t[0] = mcp_obj_from_id(frame->id);
        obj.t[1] = dlc | MCP_OBJ_IDE;
        if (frame->canfd) {
            obj.t[1] |= MCP_OBJ_FDF | MCP_OBJ_BRS;
        }
        memcpy(obj.buf + MCP_OBJ_HEADER_SIZE, frame->data, frame->data_len);

        // ram is written in whole words
        uint16_t len = MCP_OBJ_HEADER_SIZE + ((dlc_to_len[dlc] + 3) & ~3);
        mcp_write(MCP_RAM_ADDR + (sta_ua[1] & 0x0FFF), obj.buf, len);
        mcp_write8(MCP_REG_FIFOCON(MCP_TX_FIFO) + 1, (MCP_FIFOCON_UINC | MCP_FIFOCON_TXREQ) >> 8);

        dc_hal_stats.tx_attempt++;
        dc_hal_stats.tx_total++;

        __mem_fence_release(); // slot is handed to the chip, only now may core 1 see it as free
        dc_hal_txbuf.readpos = (rp + 1) & DC_HAL_TXBUFSIZEMASK;
    }
}


//-------------------------------------------------------
// core 0 poll — called from loop() on core 0
//-------------------------------------------------------

void dc_hal_poll_core0(void)
{
    if (dc_hal_state == DC_HAL_STATE_START_REQUEST) {
        dc_hal_state = (mcp_start()) ? DC_HAL_STATE_RUNNING : DC_HAL_STATE_FAILED;
        return;
    }
    if (dc_hal_state == DC_HAL_STATE_IDLE) return;

    // every 100 ms: configure the chip anew if it was not found or has left normal mode.
    // it leaves it on a system error, e.g. a ram ecc error, and then ignores all tx
    // requests, and a brown out resets it. costs about 10 us
    uint32_t tnow_us = time_us_32();
    if ((tnow_us - dc_hal_check_tlast_us) > 100000) {
        dc_hal_check_tlast_us = tnow_us;
        if (dc_hal_state == DC_HAL_STATE_FAILED || mcp_opmode() != dc_hal_opmode) {
            dc_hal_stats.parse_error_count++;
            dc_hal_state = (mcp_configure()) ? DC_HAL_STATE_RUNNING : DC_HAL_STATE_FAILED;
        }
    }
    if (dc_hal_state != DC_HAL_STATE_RUNNING) return;

    if (dc_hal_filter_request) {
        __mem_fence_acquire(); // do not read the filters before the request that published them
        mcp_apply_filters();
        dc_hal_filter_request = false;
    }

    mcp_poll_rx();
    mcp_poll_tx();

    dc_hal_stats.error_sum_count = dc_hal_stats.rx_overflow_count + dc_hal_stats.parse_error_count +
                                   dc_hal_stats.tx_abort_count;
}


//-------------------------------------------------------
// dc_hal API, called from core 1
//-------------------------------------------------------

void dc_hal_set_mcp_config(const tDcHalMcpConfig* const config)
{
    dc_hal_config = *config;
}


int16_t dc_hal_init(
    DC_HAL_CAN_ENUM can_instance,
    const tDcHalCanTimings* const timings,
    const DC_HAL_IFACE_MODE_ENUM iface_mode)
{
    (void)can_instance;
    (void)timings; // bit timings are fixed
    (void)iface_mode;

    // called once, a controller restart is a reboot on rp
    memset(&dc_hal_stats, 0, sizeof(dc_hal_stats));
    memset(&dc_hal_rxbuf, 0, sizeof(dc_hal_rxbuf));
    memset(&dc_hal_txbuf, 0, sizeof(dc_hal_txbuf));
    dc_hal_fd_frame_detected = false;

    // as on STM32, accept all into fifo 0 until dc_hal_config_acceptance_filters() is called
    memset(dc_hal_filters, 0, sizeof(dc_hal_filters));
    dc_hal_filters[0].rx_fifo = DC_HAL_RX_FIFO0;
    dc_hal_filter_num = 1;

    return 0;
}


int16_t dc_hal_start(void)
{
    // signal core 0 to do the chip configuration, so that all spi access is on core 0
    dc_hal_state = DC_HAL_STATE_START_REQUEST;

    // typically takes < 15 ms, time out instead of freezing the radio core
    uint32_t tstart_us = time_us_32();
    while (dc_hal_state == DC_HAL_STATE_START_REQUEST) {
        if ((time_us_32() - tstart_us) > 200000) break;
    }

    return (dc_hal_state == DC_HAL_STATE_RUNNING) ? 0 : -DC_HAL_ERROR_CAN_START;
}


uint8_t dc_hal_is_canfd(void)
{
    return (dc_hal_fd_frame_detected) ? 1 : 0; // only ever becomes true in canfd mode
}


// queues frame into TX ring buffer for core 0
int16_t dc_hal_transmit(const CanardCANFrame* const frame, uint32_t tnow_ms)
{
    (void)tnow_ms;

    if (frame == NULL) {
        return -DC_HAL_ERROR_INVALID_ARGUMENT;
    }

    if (frame->id & CANARD_CAN_FRAME_ERR) {
        return -DC_HAL_ERROR_UNSUPPORTED_FRAME_FORMAT;
    }
    if (frame->id & CANARD_CAN_FRAME_RTR) { // DroneCAN does not use REMOTE frames
        return -DC_HAL_ERROR_UNSUPPORTED_FRAME_FORMAT;
    }
    if (!(frame->id & CANARD_CAN_FRAME_EFF)) { // DroneCAN does not use STD ID, uses only EXT frames
        return -DC_HAL_ERROR_UNSUPPORTED_FRAME_FORMAT;
    }

    uint8_t max_data_len = (dc_hal_config.canfd && frame->canfd) ? CANARD_CANFD_FRAME_MAX_DATA_LEN : CANARD_CAN_FRAME_MAX_DATA_LEN;
    if ((frame->canfd && !dc_hal_config.canfd) || frame->data_len > max_data_len) {
        return -DC_HAL_ERROR_UNSUPPORTED_FRAME_FORMAT;
    }

    uint16_t wp = dc_hal_txbuf.writepos;
    uint16_t next = (wp + 1) & DC_HAL_TXBUFSIZEMASK;
    if (next == dc_hal_txbuf.readpos) {
        return 0; // tx fifo full, postpone
    }

    dc_hal_txbuf.frames[wp] = *frame;

    __mem_fence_release(); // slot must be visible to core 0 before writepos is
    dc_hal_txbuf.writepos = next;
    return 1;
}


int16_t dc_hal_receive(CanardCANFrame* const frame)
{
    if (frame == NULL) {
        return -DC_HAL_ERROR_INVALID_ARGUMENT;
    }

    if (dc_hal_rxbuf.writepos == dc_hal_rxbuf.readpos) {
        return 0; // fifo empty
    }
    __mem_fence_acquire(); // do not read the slot before the writepos that published it

    uint16_t rp = dc_hal_rxbuf.readpos;
    *frame = dc_hal_rxbuf.frames[rp];

    __mem_fence_release(); // slot is read out, only now may core 0 see it as free
    dc_hal_rxbuf.readpos = (rp + 1) & DC_HAL_RXBUFSIZEMASK;

    return 1;
}


// emptying is done consumer-side by advancing readpos, which is safe against the writer on core 0
void dc_hal_rx_flush(void)
{
    dc_hal_rxbuf.readpos = dc_hal_rxbuf.writepos;
}


int16_t dc_hal_enable_isr(void)
{
    return 0; // no-op, no interrupt is used
}


// num_filter_configs = 0 rejects all frames
int16_t dc_hal_config_acceptance_filters(
    const tDcHalAcceptanceFilterConfiguration* const filter_configs,
    const uint8_t num_filter_configs)
{
    if ((filter_configs == NULL) || (num_filter_configs > DC_HAL_ACCEPTANCE_FILTERS_NUM_MAX)) {
        return -DC_HAL_ERROR_INVALID_ARGUMENT;
    }

    memcpy(dc_hal_filters, filter_configs, num_filter_configs * sizeof(filter_configs[0]));
    dc_hal_filter_num = num_filter_configs;

    // signal core 0 to write them to the chip, and wait so they are not changed meanwhile
    __mem_fence_release();
    dc_hal_filter_request = true;
    if (dc_hal_state != DC_HAL_STATE_RUNNING) return 0; // are applied once it is running
    uint32_t tstart_us = time_us_32();
    while (dc_hal_filter_request) {
        if ((time_us_32() - tstart_us) > 100000) return -DC_HAL_ERROR_CAN_CONFIG_FILTER; // core 0 loop not running
    }

    return 0;
}


// returns cached stats (updated by core 0)
tDcHalStatistics dc_hal_get_stats(void)
{
    return dc_hal_stats;
}


#endif // (ARDUINO_ARCH_RP2040 || ARDUINO_ARCH_RP2350) && DEVICE_HAS_DRONECAN_FD
