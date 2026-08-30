//*******************************************************
// STM32 DroneCAN Library
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// DroneCAN Driver Library for STM32 using HAL
// for use with libcanard
//*******************************************************
// This is the STM32C5 version. The C5's FDCAN peripheral is the same Bosch M_CAN
// as the G4's and H5's, so the register level parts of this file are a copy of
// stm32-dronecan-driver-h5.c. Keep the three in sync, but mind these differences:
// - the C5 ships ST's HAL2 driver package, where the whole FDCAN API was renamed
//   and restructured: a handle carries an instance *enum* rather than a pointer,
//   config went from HAL_FDCAN_Init(&h) with an Init struct to HAL_FDCAN_Init() +
//   HAL_FDCAN_SetConfig(), and the Tx header is a bitfield union instead of a
//   struct of pre-shifted register values. So this driver keeps its own
//   fdcan_regs pointer for all direct register work, and uses HAL2 only for
//   config, filters, start and transmit.
// - the C552 has FDCAN1 only, the FDCAN2 sections are inactive
// The FDCAN bus and kernel clocks are set up by can_init() in stdstm32-can.h, as on
// the other families.
//*******************************************************
#if defined STM32C552xx

#include "stm32c5xx_hal.h"

#if defined USE_HAL_FDCAN_MODULE && (USE_HAL_FDCAN_MODULE == 1)
// HAL2's umbrella header pulls in only hal_def and a couple of LL headers, every module
// header has to be included by hand
#include "stm32c5xx_hal_fdcan.h"
#include "stm32c5xx_ll_bus.h"
#include "stm32-dronecan-driver.h"
#include <string.h>

// HAL2 renamed the CMSIS register helpers, alias the classic names the shared code uses
#ifndef READ_REG
  #define READ_REG(reg)          STM32_READ_REG(reg)
  #define CLEAR_BIT(reg, bit)    STM32_CLEAR_BIT((reg), (bit))
#endif


#define DC_HAL_ACCEPTANCE_FILTERS_NUM_MAX  8 // must be equal to SRAMCAN_FLE_NBR, for extended ids

// the three Tx buffers, HAL2 takes a bitmask of buffer indices
#define DC_HAL_TX_BUFFERS_ALL  (0x00000001U | 0x00000002U | 0x00000004U)

// HAL2 dropped the IS_FDCAN_xxx range macros, so check against our own limits
#define DC_IS_NOMINAL_PRESCALER(x)  (((x) >= DC_HAL_PRESCALER_MIN) && ((x) <= DC_HAL_PRESCALER_MAX))
#define DC_IS_NOMINAL_TSEG1(x)      (((x) >= DC_HAL_NOMINAL_BS1_MIN) && ((x) <= DC_HAL_NOMINAL_BS1_MAX))
#define DC_IS_NOMINAL_TSEG2(x)      (((x) >= DC_HAL_NOMINAL_BS2_MIN) && ((x) <= DC_HAL_NOMINAL_BS2_MAX))
#define DC_IS_NOMINAL_SJW(x)        (((x) >= DC_HAL_NOMINAL_SJW_MIN) && ((x) <= DC_HAL_NOMINAL_SJW_MAX))

// we can use IS_FDCAN_NOMINAL_PRESCALER(), IS_FDCAN_NOMINAL_TSEG1(), IS_FDCAN_NOMINAL_TSEG2(), IS_FDCAN_NOMINAL_SJW() !
#define DC_HAL_PRESCALER_MIN        1
#define DC_HAL_PRESCALER_MAX        512

#define DC_HAL_NOMINAL_BS1_MIN      1     // datasheet: bit_time must be 4 .. 81 tq
#define DC_HAL_NOMINAL_BS1_MAX      256
#define DC_HAL_NOMINAL_BS2_MIN      1
#define DC_HAL_NOMINAL_BS2_MAX      128
#define DC_HAL_NOMINAL_SJW_MIN      1
#define DC_HAL_NOMINAL_SJW_MAX      128

#define DC_HAL_DATA_PRESCALER_MIN   1
#define DC_HAL_DATA_PRESCALER_MAX   32
#define DC_HAL_DATA_BS1_MIN         1
#define DC_HAL_DATA_BS1_MAX         32
#define DC_HAL_DATA_BS2_MIN         1
#define DC_HAL_DATA_BS2_MAX         16
#define DC_HAL_DATA_SJW_MIN         1
#define DC_HAL_DATA_SJW_MAX         16    // datasheet: must always be smaller than BS2


static tDcHalStatistics dc_hal_stats = {};

static uint8_t abort_tx_on_error;

static hal_fdcan_handle_t hfdcan;
// HAL2's handle holds an instance enum, not a pointer, so keep our own pointer for the
// direct register accesses below. NULL means the iface was never opened.
static FDCAN_GlobalTypeDef* fdcan_regs = NULL;

static uint32_t tx_tlast_ms = 0; // for TX timeout error counting in dc_hal_transmit()
static uint8_t was_bo = 0; // for counting bus off errors only when state changes

static uint8_t fd_mode_enabled; // set when the iface was opened with data_timings != NULL
static uint8_t fd_frame_detected; // set once a FD frame has been seen


//-------------------------------------------------------
// Helper
//-------------------------------------------------------

static void _process_error_status(void)
{
    if (fdcan_regs == NULL) return; // iface was never opened, dc_hal_init() failed or was not called

//    FDCAN_ProtocolStatusTypeDef protocolStatus;
//    HAL_FDCAN_GetProtocolStatus(&hfdcan, &protocolStatus);
//    if (protocolStatus.BusOff) {
//        CLEAR_BIT(fdcan_regs->CCCR, FDCAN_CCCR_INIT);
//    }
    uint32_t psr = READ_REG(fdcan_regs->PSR); // read the protocol status register

    if ((psr & FDCAN_PSR_BO) != 0) { // is bus off
        CLEAR_BIT(fdcan_regs->CCCR, FDCAN_CCCR_INIT);
        HAL_FDCAN_ReqAbortOfTxBuffer(&hfdcan, DC_HAL_TX_BUFFERS_ALL);
        if (!was_bo) dc_hal_stats.bo_count++; // BO, bus off, only count when toggled from bus on to bus off
        was_bo = 1;
    } else {
        was_bo = 0;
    }

    uint32_t lec = (psr & FDCAN_PSR_LEC) >> FDCAN_PSR_LEC_Pos; // Last Error Code
    uint32_t dlec = (psr & FDCAN_PSR_DLEC) >> FDCAN_PSR_DLEC_Pos; // Data Last Error Code
    if ((lec != HAL_FDCAN_PROTOCOL_ERROR_NONE && lec != HAL_FDCAN_PROTOCOL_ERROR_NO_CHANGE) ||
        (dlec != HAL_FDCAN_PROTOCOL_ERROR_NONE && dlec != HAL_FDCAN_PROTOCOL_ERROR_NO_CHANGE)) {
        // no write to PSR here: LEC and DLEC are cleared to "no change" by the read of PSR
        // above, and the C5 header correctly declares PSR read only (the G4 and H5 headers
        // have it as __IO, so the G4/H5 drivers write it, which is a no-op on the hardware)
        if (abort_tx_on_error) {
            HAL_FDCAN_ReqAbortOfTxBuffer(&hfdcan, DC_HAL_TX_BUFFERS_ALL);
        }
        if (lec != HAL_FDCAN_PROTOCOL_ERROR_NONE && lec != HAL_FDCAN_PROTOCOL_ERROR_NO_CHANGE) {
            dc_hal_stats.lec_count++; // LEC, Last Error Code
            if (lec == HAL_FDCAN_PROTOCOL_ERROR_ACK) dc_hal_stats.lec_ack_count++;
        }
        // CAN FD diagnostics
        if (dlec != HAL_FDCAN_PROTOCOL_ERROR_NONE && dlec != HAL_FDCAN_PROTOCOL_ERROR_NO_CHANGE) {
            dc_hal_stats.dlec_count++; // DLEC, Data-phase Last Error Code
            switch (dlec) {
                case HAL_FDCAN_PROTOCOL_ERROR_STUFF: dc_hal_stats.dlec_stuff_count++; break;
                case HAL_FDCAN_PROTOCOL_ERROR_FORM: dc_hal_stats.dlec_form_count++; break;
                case HAL_FDCAN_PROTOCOL_ERROR_ACK: dc_hal_stats.dlec_ack_count++; break;
                case HAL_FDCAN_PROTOCOL_ERROR_BIT1: dc_hal_stats.dlec_bit1_count++; break;
                case HAL_FDCAN_PROTOCOL_ERROR_BIT0: dc_hal_stats.dlec_bit0_count++; break;
                case HAL_FDCAN_PROTOCOL_ERROR_CRC: dc_hal_stats.dlec_crc_count++; break;
            }
        }
    }

    if ((psr & FDCAN_PSR_PXE) != 0) { // protocol exception event occurred, happens when fc is not yet doing CAN
        // PXE is likewise cleared by the read of PSR above
        dc_hal_stats.pxe_count++; // PXE, Protocol Exception Event
    }

    uint32_t ecr = READ_REG(fdcan_regs->ECR); // read the error counter register, read clears CEL
    dc_hal_stats.tec_count = (ecr & FDCAN_ECR_TEC) >> FDCAN_ECR_TEC_Pos; // TEC, Transmit Error Counter
    dc_hal_stats.rec_count = (ecr & FDCAN_ECR_REC) >> FDCAN_ECR_REC_Pos; // REC, Receive Error Counter
    if ((ecr & FDCAN_ECR_CEL) != 0) { // Can Error Logging
        dc_hal_stats.cel_count += (ecr & FDCAN_ECR_CEL) >> FDCAN_ECR_CEL_Pos; // CEL, Can Error Logging
    }

    // record some more
    if (lec != HAL_FDCAN_PROTOCOL_ERROR_NO_CHANGE) dc_hal_stats.last_lec = lec;
    dc_hal_stats.last_psr = psr;
    dc_hal_stats.last_ecr = ecr;
    dc_hal_stats.last_cccr = READ_REG(fdcan_regs->CCCR);
    // CAN FD
    if (dlec != HAL_FDCAN_PROTOCOL_ERROR_NO_CHANGE) dc_hal_stats.last_dlec = dlec;
}


// convert DLC field to data length in bytes (with CAN FD encoding)
static const uint8_t dlc_to_len[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

uint8_t _data_len_from_dlc(uint8_t dlc)
{
    return (dlc < 16) ? dlc_to_len[dlc] : 64;
}


// convert data length in bytes to DLC field (with CAN FD encoding)
uint8_t _dlc_from_data_len(uint8_t data_len)
{
    if (data_len > 64) data_len = 64; // should not happen, but play it safe
    if (data_len <= 8) return data_len;
    if (data_len <= 12) return 9;
    if (data_len <= 16) return 10;
    if (data_len <= 20) return 11;
    if (data_len <= 24) return 12;
    if (data_len <= 32) return 13;
    if (data_len <= 48) return 14;
    return 15; // 49-64 bytes
}


//-------------------------------------------------------
// Init
//-------------------------------------------------------

int16_t dc_hal_init(
    DC_HAL_CAN_ENUM can_instance,
    const tDcHalCanTimings* const timings,
    const tDcHalCanDataTimings* const data_timings,
    const DC_HAL_IFACE_MODE_ENUM iface_mode)
{
    if ((iface_mode != DC_HAL_IFACE_MODE_NORMAL) &&
        (iface_mode != DC_HAL_IFACE_MODE_SILENT) &&
        (iface_mode != DC_HAL_IFACE_MODE_AUTOMATIC_TX_ABORT_ON_ERROR)) {
        return -DC_HAL_ERROR_INVALID_ARGUMENT;
    }

    if ((timings == NULL) ||
        !DC_IS_NOMINAL_PRESCALER(timings->bit_rate_prescaler) ||
        !DC_IS_NOMINAL_TSEG1(timings->bit_segment_1) ||
        !DC_IS_NOMINAL_TSEG2(timings->bit_segment_2) ||
        !DC_IS_NOMINAL_SJW(timings->sync_jump_width)) {
        return -DC_HAL_ERROR_INVALID_ARGUMENT;
    }

    memset(&dc_hal_stats, 0, sizeof(dc_hal_stats));
    abort_tx_on_error = (iface_mode == DC_HAL_IFACE_MODE_AUTOMATIC_TX_ABORT_ON_ERROR);

    fd_mode_enabled = (data_timings != NULL); // nullptr doesn't work, as undeclared
    fd_frame_detected = 0;

    // the C552 has FDCAN1 only, can_instance is accepted but ignored
    (void)can_instance;

    if (HAL_FDCAN_Init(&hfdcan, HAL_FDCAN1) != HAL_OK) {
        return -DC_HAL_ERROR_CAN_INIT;
    }
    fdcan_regs = FDCAN1;

    hal_fdcan_config_t cfg = {0};

    cfg.mode = HAL_FDCAN_MODE_NORMAL;

    cfg.auto_retransmission = HAL_FDCAN_AUTO_RETRANSMISSION_ENABLE; // ArduPilot has it enabled, so also do
    cfg.transmit_pause = HAL_FDCAN_TRANSMIT_PAUSE_ENABLE; // insert pause between TX frames to reduce bus contention
    cfg.protocol_exception = HAL_FDCAN_PROTOCOL_EXCEPTION_DISABLE; // treat exceptions as form errors

    cfg.nominal_bit_timing.nominal_prescaler = timings->bit_rate_prescaler;
    cfg.nominal_bit_timing.nominal_time_seg1 = timings->bit_segment_1;
    cfg.nominal_bit_timing.nominal_time_seg2 = timings->bit_segment_2;
    cfg.nominal_bit_timing.nominal_jump_width = timings->sync_jump_width;

    if (fd_mode_enabled) {
        // enable FD frame format with BRS - allows receiving both classic CAN and CAN FD frames
        cfg.frame_format = HAL_FDCAN_FRAME_FORMAT_FD_BRS;
        cfg.data_bit_timing.data_prescaler = data_timings->bit_rate_prescaler;
        cfg.data_bit_timing.data_time_seg1 = data_timings->bit_segment_1;
        cfg.data_bit_timing.data_time_seg2 = data_timings->bit_segment_2;
        cfg.data_bit_timing.data_jump_width = data_timings->sync_jump_width;
    } else {
        cfg.frame_format = HAL_FDCAN_FRAME_FORMAT_CLASSIC_CAN;
        cfg.data_bit_timing.data_prescaler = 1; // irrelevant if frame_format is not FD_BRS
        cfg.data_bit_timing.data_time_seg1 = 1;
        cfg.data_bit_timing.data_time_seg2 = 1;
        cfg.data_bit_timing.data_jump_width = 1;
    }

    cfg.std_filters_nbr = 0; // these size the message RAM filter blocks
    cfg.ext_filters_nbr = DC_HAL_ACCEPTANCE_FILTERS_NUM_MAX;

    cfg.tx_fifo_queue_mode = HAL_FDCAN_TX_MODE_FIFO;

    if (HAL_FDCAN_SetConfig(&hfdcan, &cfg) != HAL_OK) {
        fdcan_regs = NULL;
        return -DC_HAL_ERROR_CAN_INIT;
    }

    if (fd_mode_enabled && data_timings->tdco > 0) { // Transceiver Delay Compensation for CANFD
        hal_fdcan_tx_delay_comp_config_t tdc = {0};
        tdc.tx_delay_comp_offset = data_timings->tdco;
        tdc.tx_delay_comp_win_length = 0;
        HAL_FDCAN_SetConfigTxDelayCompensation(&hfdcan, &tdc);
        HAL_FDCAN_EnableTxDelayCompensation(&hfdcan);
    }

    // configure reception filter
    //   follow libcanard's default filter setup in spirit
    //   here it is really needed since the FDCAN RAM was already set up for max num filters
    // at least one filter must be enabled for receive to work
    hal_fdcan_filter_t sFilterConfig = {0};
    sFilterConfig.id_type = HAL_FDCAN_ID_EXTENDED;
    sFilterConfig.filter_type = HAL_FDCAN_FILTER_TYPE_CLASSIC; // filter_id1 = id, filter_id2 = mask
    sFilterConfig.filter_id1 = 0;
    sFilterConfig.filter_id2 = 0;

    for (uint8_t n = 0; n < DC_HAL_ACCEPTANCE_FILTERS_NUM_MAX; n++) {
        sFilterConfig.filter_config = (n == 0) ? HAL_FDCAN_FILTER_TO_RX_FIFO_0 : HAL_FDCAN_FILTER_DISABLE;
        sFilterConfig.filter_index = n;
        if (HAL_FDCAN_SetFilter(&hfdcan, &sFilterConfig) != HAL_OK) {
            return -DC_HAL_ERROR_CAN_CONFIG_FILTER;
        }
    }

    // configure global filter
    //  reject non matching frames with STD and EXT ID
    //  filter all remote frames with STD and EXT ID
    hal_fdcan_global_filter_config_t gfilter = {0};
    gfilter.acceptance_non_matching_std = HAL_FDCAN_NO_MATCH_REJECT;
    gfilter.acceptance_non_matching_ext = HAL_FDCAN_NO_MATCH_REJECT;
    gfilter.acceptance_remote_std = HAL_FDCAN_REMOTE_REJECT;
    gfilter.acceptance_remote_ext = HAL_FDCAN_REMOTE_REJECT;
    if (HAL_FDCAN_SetGlobalFilter(&hfdcan, &gfilter) != HAL_OK) {
        return -DC_HAL_ERROR_CAN_CONFIG_GLOBAL_FILTER;
    }

    return 0;
}


int16_t dc_hal_start(void)
{
    if (fdcan_regs == NULL) { return -DC_HAL_ERROR_CAN_START; } // iface was never opened

    if (HAL_FDCAN_Start(&hfdcan) != HAL_OK) { return -DC_HAL_ERROR_CAN_START; }

    return 0;
}


uint8_t dc_hal_is_canfd(void)
{
    return fd_frame_detected; // only ever becomes true if also fd_mode_enabled is true
}


//-------------------------------------------------------
// Transmit
//-------------------------------------------------------

int16_t dc_hal_transmit(const CanardCANFrame* const frame, uint32_t tnow_ms)
{
    if (fdcan_regs == NULL) { return -DC_HAL_ERROR_CAN_ADD_TX_MESSAGE; } // iface was never opened

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

    if (!fd_mode_enabled && frame->data_len > 8) { // don't do FD if not enabled
        return -DC_HAL_ERROR_UNSUPPORTED_FRAME_FORMAT;
    }

    _process_error_status();

    // thx to the TxFiFo in the C5 we can do the crude method and just put the message into the fifo if there is space
    // check for space in fifo
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan) == HAL_FDCAN_TX_FIFO_FREE_LEVEL_0) {
        if (tx_tlast_ms > 0 && (tnow_ms - tx_tlast_ms) > 10) {
            dc_hal_stats.tffl_count++; // TFFL, Tx Fifo Free Level
        }
        return 0; // no space, postpone
    }
    // this check is done in HAL_FDCAN_AddMessageToTxFifoQ(), so better do it here too
    if ((fdcan_regs->TXFQS & FDCAN_TXFQS_TFQF) != 0) {
        if (tx_tlast_ms > 0 && (tnow_ms - tx_tlast_ms) > 10) {
            dc_hal_stats.tfqf_count++; // TFQF, Tx Fifo Queue Full
        }
        return 0; // tx fifo queue full, postpone
    }

    // HAL2's Tx header is a bitfield union written straight into the message RAM word,
    // so every field is the plain code here, nothing is pre-shifted
    hal_fdcan_tx_header_t pTxHeader;
    pTxHeader.d64 = 0;
    pTxHeader.b.error_state_indicator = HAL_FDCAN_ERROR_STATE_IND_ACTIVE;
    if (fd_mode_enabled && frame->canfd) {
        pTxHeader.b.bit_rate_switch = HAL_FDCAN_BIT_RATE_SWITCH_ON;
        pTxHeader.b.frame_format = HAL_FDCAN_HEADER_FRAME_FORMAT_FD_CAN;
    } else {
        pTxHeader.b.bit_rate_switch = HAL_FDCAN_BIT_RATE_SWITCH_OFF;
        pTxHeader.b.frame_format = HAL_FDCAN_HEADER_FRAME_FORMAT_CAN;
    }
    pTxHeader.b.event_fifo_control = HAL_FDCAN_TX_EVENTS_FIFO_DISCARD;
    pTxHeader.b.message_marker = 0;

    pTxHeader.b.identifier = (frame->id & CANARD_CAN_EXT_ID_MASK);
    pTxHeader.b.identifier_type = HAL_FDCAN_ID_EXTENDED;
    pTxHeader.b.frame_type = HAL_FDCAN_FRAME_DATA;

    pTxHeader.b.data_length = (hal_fdcan_data_length_code_t)_dlc_from_data_len(frame->data_len);

    if (HAL_FDCAN_ReqTransmitMsgFromFIFOQ(&hfdcan, &pTxHeader, frame->data) != HAL_OK) {
        return -DC_HAL_ERROR_CAN_ADD_TX_MESSAGE;
    }

    tx_tlast_ms = tnow_ms;
    dc_hal_stats.transmitted_frame_count++;

    return 1;
}


//-------------------------------------------------------
// Receive
//-------------------------------------------------------
#ifndef DRONECAN_USE_RX_ISR
//-- Polling
#error CAN polling not supported, DRONECAN_USE_RX_ISR must be defined !
#else
//-- ISR

#define DC_FDCAN_RX_FIFO_ELEMENT_SIZE  (18U * 4U) // Rx FIFO 0/1 element size in bytes, 2 words + 64 bytes = 18*4, equals SRAMCAN_RFx_SIZE


typedef struct
{
    uint32_t r0;
    uint32_t r1;
    union {
        uint8_t data[CANARD_CANFD_FRAME_MAX_DATA_LEN];
        uint32_t data_32[CANARD_CANFD_FRAME_MAX_DATA_LEN / 4]; // CANARD_CANFD_FRAME_MAX_DATA_LEN should be divisible by 4
    };
} tDcRxFifoElement;


typedef enum // see table 400 in datasheet, unfortunately defined in stm32h5xx_hal_fdcan.c and not in .h file
{
    DC_RX_FIFO_R0_XTD_BIT     = 0x40000000U, // extended identifier, equals FDCAN_ELEMENT_MASK_XTD
    DC_RX_FIFO_R0_RTR_BIT     = 0x20000000U, // remote transmission request, equals FDCAN_ELEMENT_MASK_RTR
    DC_RX_FIFO_R0_EXTID_MASK  = 0x1FFFFFFFU, // identifier, equals FDCAN_ELEMENT_MASK_EXTID, CANARD_CAN_EXT_ID_MASK
    DC_RX_FIFO_R1_FDF_BIT     = 0x00200000U, // FD format, equals FDCAN_ELEMENT_MASK_FDF
    DC_RX_FIFO_R1_BRS_BIT     = 0x00100000U, // bit rate switch, equals FDCAN_ELEMENT_MASK_BRS
    DC_RX_FIFO_R1_DLC_MASK    = 0x000F0000U, // data length code, equals FDCAN_ELEMENT_MASK_DLC, FDCAN_DLC_BYTES_64
} DC_RX_FIFO_ELEMENT_ENUM;


#define DRONECAN_RXFRAMEBUFSIZEMASK  (DRONECAN_RXFRAMEBUFSIZE - 1)


volatile tDcRxFifoElement dronecan_rxbuf[DRONECAN_RXFRAMEBUFSIZE];
volatile uint16_t dronecan_rxwritepos; // pos at which the last frame was stored
volatile uint16_t dronecan_rxreadpos; // pos at which the next frame is to be fetched


void _dc_hal_receive_isr(uint32_t* RxAddress)
{
    uint32_t r0 = *RxAddress;
    if ((r0 & DC_RX_FIFO_R0_XTD_BIT) == 0) { // DroneCAN uses only EXT frames, so this should be an error
        dc_hal_stats.isr_xtd_count++;
        return;
    }
    if ((r0 & DC_RX_FIFO_R0_RTR_BIT) != 0) { // DroneCAN does not use RTR frames, so this should be an error
        dc_hal_stats.isr_rtr_count++;
        return;
    }

    RxAddress++;
    uint32_t r1 = *RxAddress;
    bool is_fd_frame = ((r1 & DC_RX_FIFO_R1_FDF_BIT) != 0);
    if (!fd_mode_enabled) {
        if (is_fd_frame) {
            dc_hal_stats.isr_fdf_count++;
            return;
        }
        if ((r1 & DC_RX_FIFO_R1_BRS_BIT) != 0) {
            dc_hal_stats.isr_brs_count++;
            return;
        }
    } else {
        if (is_fd_frame) fd_frame_detected = 1; // report that one FD frame has been seen
    }

    // for classic CAN, reject frames with DLC > 8
    // compare the extracted DLC, not the masked register field against FDCAN_DLC_BYTES_8,
    // as the meaning of FDCAN_DLC_BYTES_x differs between the G4 and H5 HAL generations
    if (!is_fd_frame && (((r1 & DC_RX_FIFO_R1_DLC_MASK) >> 16) > 8)) {
        dc_hal_stats.isr_dlc_count++;
        return;
    }

    uint16_t next = (dronecan_rxwritepos + 1) & DRONECAN_RXFRAMEBUFSIZEMASK;
    if (dronecan_rxreadpos != next) { // fifo not full
        dronecan_rxwritepos = next;

        dronecan_rxbuf[next].r0 = r0;
        dronecan_rxbuf[next].r1 = r1;
        RxAddress++;
        // copy all data bytes based on actual DLC (H5 message RAM supports 64 bytes)
        uint32_t dlc = (r1 & DC_RX_FIFO_R1_DLC_MASK) >> 16;
        uint8_t data_len = _data_len_from_dlc(dlc);
        uint8_t word_len = (data_len + 3) / 4; // round up to full words
        for (uint8_t i = 0; i < word_len; i++) {
            dronecan_rxbuf[next].data_32[i] = *RxAddress;
            RxAddress++;
        }

        // fill can reach DRONECAN_RXFRAMEBUFSIZE - 1 at most, one slot is always kept free
        uint16_t fill = (next - dronecan_rxreadpos) & DRONECAN_RXFRAMEBUFSIZEMASK;
        if (fill > dc_hal_stats.rx_fifo_peak) dc_hal_stats.rx_fifo_peak = fill;

    } else {
        dc_hal_stats.rx_overflow_count++; // rx frame buffer overflow
        dc_hal_stats.rx_fifo_peak = DRONECAN_RXFRAMEBUFSIZE; // full
    }
}


void _dc_hal_isr_handler(void)
{
    //HAL_FDCAN_IRQHandler(&hfdcan); HAL_FDCAN_GetRxMessage();
    // copy the part relevant to us
    // flags:
    //   FDCAN_IR_RF0L           // Rx FIFO 0 message lost
    //   FDCAN_IR_RF0F           // Rx FIFO 0 full
    //   FDCAN_IR_RF0N           // New message written to Rx FIFO 0
    // more descriptive defines are in the HAL, like FDCAN_FLAG_RX_FIFO0_NEW_MESSAGE
    // #define FDCAN_RX_FIFO0_MASK (FDCAN_IR_RF0L | FDCAN_IR_RF0F | FDCAN_IR_RF0N)
    // there are also analogous defines
    //   FDCAN_IE_RF0LE          // Rx FIFO 0 message lost
    //   FDCAN_IE_RF0FE          // Rx FIFO 0 full
    //   FDCAN_IE_RF0NE          // New message written to Rx FIFO 0
    // for which there are also more descriptive defines in the HAL, like FDCAN_IT_RX_FIFO0_NEW_MESSAGE

    uint32_t RxFifo0ITs = fdcan_regs->IR & (FDCAN_IR_RF0N | FDCAN_IR_RF0F | FDCAN_IR_RF0L); // __HAL_FDCAN_GET_FLAG(), equals FDCAN_RX_FIFO0_MASK
    RxFifo0ITs &= fdcan_regs->IE; // __HAL_FDCAN_GET_IT_SOURCE()
    uint32_t RxFifo1ITs = fdcan_regs->IR & (FDCAN_IR_RF1N | FDCAN_IR_RF1F | FDCAN_IR_RF1L); // equals FDCAN_RX_FIFO1_MASK
    RxFifo1ITs &= fdcan_regs->IE;

    if (RxFifo0ITs != 0) {
        fdcan_regs->IR = RxFifo0ITs; // clear Rx FIFO0 flags

        if ((RxFifo0ITs & (FDCAN_IR_RF0N | FDCAN_IR_RF0F)) != 0) { // do it also if Rx FIFO 0 full
            // HAL_FDCAN_GetRxMessage()
            while ((fdcan_regs->RXF0S & FDCAN_RXF0S_F0FL) != 0) { // Rx FIFO 0 not empty
                // calculate Rx FIFO 0 element address
                uint32_t GetIndex = ((fdcan_regs->RXF0S & FDCAN_RXF0S_F0GI) >> FDCAN_RXF0S_F0GI_Pos);
                uint32_t* RxAddress = (uint32_t*)(hfdcan.msg_ram.rx_fifo0_start_addr + (GetIndex * DC_FDCAN_RX_FIFO_ELEMENT_SIZE));
                _dc_hal_receive_isr(RxAddress);
                // acknowledge the Rx FIFO 0 that the oldest element is read so that it increments the GetIndex
                fdcan_regs->RXF0A = GetIndex;
            }
        }

        if ((RxFifo0ITs & FDCAN_IR_RF0F) != 0) {
            dc_hal_stats.isr_rf0f_count++; // RF0F, Rx Fifo 0 Full
        }
        if ((RxFifo0ITs & FDCAN_IR_RF0L) != 0) {
            dc_hal_stats.isr_rf0l_count++; // RF0L, Rx Fifo 0 Message Lost
        }
    }

    if (RxFifo1ITs != 0) {
        fdcan_regs->IR = RxFifo1ITs; // clear Rx FIFO1 flags

        if ((RxFifo1ITs & (FDCAN_IR_RF1N | FDCAN_IR_RF1F)) != 0) { // do it also if Rx FIFO 1 full
            while ((fdcan_regs->RXF1S & FDCAN_RXF1S_F1FL) != 0) { // Rx FIFO 1 not empty
                // calculate Rx FIFO 1 element address
                uint32_t GetIndex = ((fdcan_regs->RXF1S & FDCAN_RXF1S_F1GI) >> FDCAN_RXF1S_F1GI_Pos);
                uint32_t* RxAddress = (uint32_t*)(hfdcan.msg_ram.rx_fifo1_start_addr + (GetIndex * DC_FDCAN_RX_FIFO_ELEMENT_SIZE));
                _dc_hal_receive_isr(RxAddress);
                // acknowledge the Rx FIFO 1 that the oldest element is read so that it increments the GetIndex
                fdcan_regs->RXF1A = GetIndex;
            }
        }

        if ((RxFifo1ITs & FDCAN_IR_RF1F) != 0) {
            dc_hal_stats.isr_rf1f_count++; // RF1F, Rx Fifo 1 Full
        }
        if ((RxFifo1ITs & FDCAN_IR_RF1L) != 0) {
            dc_hal_stats.isr_rf1l_count++; // RF1L, Rx Fifo 1 Message Lost
        }
    }

    // EOL: Error Logging Overflow                -> Bit and Line Error
    // WDI: Watchdog Interrupt                    -> Protocol Error
    // PEA: Protocol Error in Arbitration Phase   -> Protocol Error
    // PED: Protocol Error in Data Phase          -> Protocol Error
    // ARA: Access to Reserved Address            -> Protocol Error
    uint32_t Errors = fdcan_regs->IR & (FDCAN_IR_ELO | FDCAN_IR_WDI | FDCAN_IR_PEA | FDCAN_IR_PED | FDCAN_IR_ARA);
    Errors &= fdcan_regs->IE;
    // EP: Error Passive                          -> Bit and Line Error
    // EW: Warning Status                         -> Protocol Error
    // BO: Bus_Off Status                         -> Protocol Error
    uint32_t ErrorStatusITs = fdcan_regs->IR & (FDCAN_IR_EP | FDCAN_IR_EW | FDCAN_IR_BO);
    ErrorStatusITs &= fdcan_regs->IE;

    if (Errors != 0) {
        fdcan_regs->IR = Errors; // clear the Error flags
        dc_hal_stats.isr_errors_count++;
    }
    if (ErrorStatusITs != 0) {
        fdcan_regs->IR = ErrorStatusITs; // clear the Error flags
        dc_hal_stats.isr_errorstatus_count++;
    }
}


// is already C context, not C++ !

#ifdef FDCAN2
void FDCAN2_IT0_IRQHandler(void)
{
    if (fdcan_regs == FDCAN2) _dc_hal_isr_handler();
}
#endif

void FDCAN1_IT0_IRQHandler(void)
{
    if (fdcan_regs == FDCAN1) _dc_hal_isr_handler();
}


//-- API

int16_t dc_hal_enable_isr(void)
{
    if (fdcan_regs == NULL) { return -DC_HAL_ERROR_ISR_CONFIG; } // iface was never opened

    dronecan_rxwritepos = 0;
    dronecan_rxreadpos = 0;
    memset(&dc_hal_stats, 0, sizeof(dc_hal_stats));
    fd_frame_detected = 0;

    // HAL2 doc: by default all interrupts are assigned to line 0, so no
    // HAL_FDCAN_SetInterruptGroupsToLine() is needed here
    if (HAL_FDCAN_EnableInterrupts(
            &hfdcan,
            HAL_FDCAN_IT_RX_FIFO_0_NEW_MSG | HAL_FDCAN_IT_RX_FIFO_0_FULL |
            HAL_FDCAN_IT_RX_FIFO_1_NEW_MSG | HAL_FDCAN_IT_RX_FIFO_1_FULL |
            HAL_FDCAN_IT_BUS_OFF) != HAL_OK) {
        return -DC_HAL_ERROR_ISR_CONFIG;
    }

    if (HAL_FDCAN_EnableInterruptLines(&hfdcan, HAL_FDCAN_IT_LINE_0) != HAL_OK) {
        return -DC_HAL_ERROR_ISR_CONFIG;
    }

    fdcan_regs->IR = 0xFFFFFFFF; // clear all flags by writing 1 to them

#ifdef FDCAN2
    if (fdcan_regs == FDCAN2) {
        NVIC_SetPriority(FDCAN2_IT0_IRQn, DRONECAN_IRQ_PRIORITY);
        NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
    } else
#endif
    if (fdcan_regs == FDCAN1) {
        NVIC_SetPriority(FDCAN1_IT0_IRQn, DRONECAN_IRQ_PRIORITY);
        NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    }

    return 0;
}


int16_t dc_hal_receive(CanardCANFrame* const frame)
{
    if (fdcan_regs == NULL) { return 0; } // iface was never opened

    if (frame == NULL) {
        return -DC_HAL_ERROR_INVALID_ARGUMENT;
    }

    _process_error_status();

    if (dronecan_rxwritepos == dronecan_rxreadpos) {
        return 0; // fifo empty
    }

    uint16_t rxreadpos = (dronecan_rxreadpos + 1) & DRONECAN_RXFRAMEBUFSIZEMASK;
    dronecan_rxreadpos = rxreadpos;

    frame->id = (dronecan_rxbuf[rxreadpos].r0 & DC_RX_FIFO_R0_EXTID_MASK) >> 0;
    frame->id |= CANARD_CAN_FRAME_EFF;

    // convert DLC to actual byte count
    uint32_t dlc = (dronecan_rxbuf[rxreadpos].r1 & DC_RX_FIFO_R1_DLC_MASK) >> 16;
    frame->data_len = _data_len_from_dlc(dlc);
    if (frame->data_len > CANARD_CANFD_FRAME_MAX_DATA_LEN) frame->data_len = CANARD_CANFD_FRAME_MAX_DATA_LEN; // should not happen, but play it safe

    // copy data bytes, and zero-fill
    for (uint8_t n = 0; n < CANARD_CANFD_FRAME_MAX_DATA_LEN; n++) {
        frame->data[n] = (n < frame->data_len) ? dronecan_rxbuf[rxreadpos].data[n] : 0;
    }

    frame->iface_id = 0;

    frame->canfd = ((dronecan_rxbuf[rxreadpos].r1 & DC_RX_FIFO_R1_FDF_BIT) != 0);

    dc_hal_stats.received_frame_count++;

    return 1;
}


void dc_hal_rx_flush(void)
{
    if (fdcan_regs == NULL) return; // iface was never opened

    dronecan_rxwritepos = 0;
    dronecan_rxreadpos = 0;
    dc_hal_stats.rx_overflow_count = 0;
}


#endif // DRONECAN_USE_RX_ISR


//-------------------------------------------------------
// Filter
//-------------------------------------------------------

// num_filter_configs = 0 rejects all frames
int16_t dc_hal_config_acceptance_filters(
    const tDcHalAcceptanceFilterConfiguration* const filter_configs,
    const uint8_t num_filter_configs)
{
    if (fdcan_regs == NULL) { return -DC_HAL_ERROR_CAN_CONFIG_FILTER; } // iface was never opened

    if ((filter_configs == NULL) || (num_filter_configs > DC_HAL_ACCEPTANCE_FILTERS_NUM_MAX)) {
        return -DC_HAL_ERROR_INVALID_ARGUMENT;
    }

    hal_fdcan_filter_t sFilterConfig = {0};
    sFilterConfig.id_type = HAL_FDCAN_ID_EXTENDED;
    sFilterConfig.filter_type = HAL_FDCAN_FILTER_TYPE_CLASSIC; // filter_id1 = id, filter_id2 = mask
    sFilterConfig.filter_config = HAL_FDCAN_FILTER_TO_RX_FIFO_0;
    sFilterConfig.filter_index = 0;
    sFilterConfig.filter_id1 = 0;
    sFilterConfig.filter_id2 = 0;

    for (uint8_t n = 0; n < num_filter_configs; n++) {
        if (filter_configs[n].rx_fifo == DC_HAL_RX_FIFO0) {
            sFilterConfig.filter_config = HAL_FDCAN_FILTER_TO_RX_FIFO_0;
        } else
        if (filter_configs[n].rx_fifo == DC_HAL_RX_FIFO1) {
            sFilterConfig.filter_config = HAL_FDCAN_FILTER_TO_RX_FIFO_1;
        } else {
            sFilterConfig.filter_config = ((n & 0x01) == 0) ? HAL_FDCAN_FILTER_TO_RX_FIFO_0 : HAL_FDCAN_FILTER_TO_RX_FIFO_1;
        }
        sFilterConfig.filter_index = n;
        sFilterConfig.filter_id1 = (filter_configs[n].id & CANARD_CAN_EXT_ID_MASK);
        sFilterConfig.filter_id2 = (filter_configs[n].mask & CANARD_CAN_EXT_ID_MASK);
        if (HAL_FDCAN_SetFilter(&hfdcan, &sFilterConfig) != HAL_OK) { return -DC_HAL_ERROR_CAN_CONFIG_FILTER; }
    }

    // fill remaining filters with default
    sFilterConfig.filter_id1 = 0;
    sFilterConfig.filter_id2 = 0;
    for (uint8_t n = num_filter_configs; n < DC_HAL_ACCEPTANCE_FILTERS_NUM_MAX; n++) {
        sFilterConfig.filter_config = HAL_FDCAN_FILTER_DISABLE;
        sFilterConfig.filter_index = n;
        if (HAL_FDCAN_SetFilter(&hfdcan, &sFilterConfig) != HAL_OK) { return -DC_HAL_ERROR_CAN_CONFIG_FILTER; }
    }

    return 0;
}


//-------------------------------------------------------
// More Helper
//-------------------------------------------------------

tDcHalStatistics dc_hal_get_stats(void)
{
    _process_error_status(); // to ensure it is latest
    dc_hal_stats.error_sum_count =
        dc_hal_stats.bo_count + 
        dc_hal_stats.lec_count + dc_hal_stats.dlec_count +
        dc_hal_stats.pxe_count + dc_hal_stats.cel_count;
#ifdef DRONECAN_USE_RX_ISR
    dc_hal_stats.error_sum_count += dc_hal_stats.rx_overflow_count;
    dc_hal_stats.error_sum_count += dc_hal_stats.isr_xtd_count;
    dc_hal_stats.error_sum_count += dc_hal_stats.isr_rtr_count;
    dc_hal_stats.error_sum_count += dc_hal_stats.isr_fdf_count;
    dc_hal_stats.error_sum_count += dc_hal_stats.isr_brs_count;
    dc_hal_stats.error_sum_count += dc_hal_stats.isr_dlc_count;
    dc_hal_stats.error_sum_count += dc_hal_stats.isr_rf0l_count;
    dc_hal_stats.error_sum_count += dc_hal_stats.isr_rf0f_count;
    dc_hal_stats.error_sum_count += dc_hal_stats.isr_rf1l_count;
    dc_hal_stats.error_sum_count += dc_hal_stats.isr_rf1f_count;
    dc_hal_stats.error_sum_count += dc_hal_stats.isr_errors_count;
    dc_hal_stats.error_sum_count += dc_hal_stats.isr_errorstatus_count;
#endif
    dc_hal_stats.error_sum_count += dc_hal_stats.tffl_count;
    dc_hal_stats.error_sum_count += dc_hal_stats.tfqf_count;
    return dc_hal_stats;
}


const char* dc_hal_psr_lec_to_str(uint32_t psr)
{
    uint32_t lec = (psr & FDCAN_PSR_LEC) >> FDCAN_PSR_LEC_Pos; // Last Error Code
    switch (lec) {
        case 0: return "ok";
        case 1: return "STUFF";
        case 2: return "FORM";
        case 3: return "ACK";
        case 4: return "BIT1";
        case 5: return "BIT0";
        case 6: return "CRC";
        case 7: return "NC"; // No Change
        default: return "??";
    }
}


const char* dc_hal_psr_act_to_str(uint32_t psr)
{
    uint32_t act = (psr & FDCAN_PSR_ACT) >> FDCAN_PSR_ACT_Pos; // Activity
    switch (act) {
        case 0: return "Sync";  // Synchronizing
        case 1: return "Idle";  // Idle
        case 2: return "Rx";    // Receiver
        case 3: return "Tx";    // Transmitter
        default: return "??";
    }
}


int16_t dc_hal_compute_timings(
    const uint32_t peripheral_clock_rate,
    const uint32_t target_bit_rate,
    tDcHalCanTimings* const timings)
{
    if (target_bit_rate != 1000000) {
        return -DC_HAL_ERROR_UNSUPPORTED_BIT_RATE;
    }

    // general rule:
    // tq = peripheral_clock_rate / bit_rate / prescaler
    // BS1 = SP * tq - 1, where SP is e.g. 3/4 = 75% or 7/8 = 87.5%
    // BS2 = tq - 1 - BS1 = (1 - SP) * tq
    // -> SP = (1 + BS1)/(1 + BS1 + BS2)

    // Note: ChatGPT was very clear on that one should use 80 MHz clock for FDCAN
    // we used 170 MHz before for classic CAN, but 80 MHz is said to be just better

    // timings by ArduPilot (confirmed by ChatGPT, by JLP, by myself using AP's code explicitly)
    // 10 tq
    // prescaler 8
    // BS1 = 8
    // BS2 = 1
    // SJW = 1
    // -> SP = 90.0%
    // Note: this is somewhat weird. AP cites a source that says that 8 tq would be optimal,
    // which can be achieved with prescaler 10, BS1 = 6, BS2 = 1, SJW = 1, -> SP = 7/8 = 87.5%.
    // It also would give SP 87.5% which ChatGPT says is industry standard. ??
#if 1
    // the C5 runs FDCAN at 80 MHz, PSIK = PSI 160 MHz / 2, see stdstm32-can.h
    if (peripheral_clock_rate == 80000000) { // 80 MHz
        timings->bit_rate_prescaler = 8;
        timings->bit_segment_1 = 8;
        timings->bit_segment_2 = 1;
        timings->sync_jump_width = 1;
    } else if (peripheral_clock_rate == 160000000) { // 160 MHz // NOT PREFFRED, but not terrible
        timings->bit_rate_prescaler = 16;
        timings->bit_segment_1 = 8;
        timings->bit_segment_2 = 1;
        timings->sync_jump_width = 1;
    } else if (peripheral_clock_rate == 170000000) { // 170 MHz // NOT PREFFRED
        timings->bit_rate_prescaler = 17;
        timings->bit_segment_1 = 8;
        timings->bit_segment_2 = 1;
        timings->sync_jump_width = 1;
    } else {
        return -DC_HAL_ERROR_UNSUPPORTED_CLOCK_FREQUENCY;
    }
#endif

    // timings generated by phryniszak for 75%
    // legacy: this is what we used before with 170 MHz FDCAN clock
#if 0
    if (peripheral_clock_rate == 170000000) { // 170 MHz
        timings->bit_rate_prescaler = 1;
        timings->bit_segment_1 = 127;
        timings->bit_segment_2 = 42; // -> SP = 0.75294 %
        timings->sync_jump_width = 42;
    } else if (peripheral_clock_rate == 160000000) { // 160 MHz
        timings->bit_rate_prescaler = 1;
        timings->bit_segment_1 = 119;
        timings->bit_segment_2 = 40; // -> SP = 0.75 %
        timings->sync_jump_width = 40;
    } else if (peripheral_clock_rate == 80000000) { // 80 MHz
        timings->bit_rate_prescaler = 1;
        timings->bit_segment_1 = 59;
        timings->bit_segment_2 = 20; // -> SP = 0.75 %
        timings->sync_jump_width = 20;
    } else {
        return -DC_HAL_ERROR_UNSUPPORTED_CLOCK_FREQUENCY;
    }
#endif
    // timings generated by phryniszak for 87.5%
#if 0
    if (peripheral_clock_rate == 170000000) { // 170 MHz
        timings->bit_rate_prescaler = 1;
        timings->bit_segment_1 = 147;
        timings->bit_segment_2 = 22;
        timings->sync_jump_width = 21;
    } else {
        return -DC_HAL_ERROR_UNSUPPORTED_CLOCK_FREQUENCY;
    }
#endif

    // let's do a check
    // datasheet:
    //   tq = prescaler * 1/f_clk
    //   bit_time = (1 + BS1 + BS2) * tq
    // =>
    //   bit_time = (1 + BS1 + BS2) * prescaler * 1/f_clk
    // we want bit_time = 1/1000000
    // =>
    //   (1 + BS1 + BS2) * prescaler = f_clk / 1000000
    const uint32_t bit_time = (1 + timings->bit_segment_1 + timings->bit_segment_2) * timings->bit_rate_prescaler;
    const uint32_t f_clk_MHz = peripheral_clock_rate / 1000000;
    if (bit_time != f_clk_MHz) {
        return -DC_HAL_ERROR_TIMING;
    }

    return 0;
}


int16_t dc_hal_compute_data_timings(
    const uint32_t peripheral_clock_rate,
    const uint32_t target_data_bit_rate,
    tDcHalCanDataTimings* const data_timings)
{
    if (peripheral_clock_rate != 80000000) {
        return -DC_HAL_ERROR_UNSUPPORTED_CLOCK_FREQUENCY;
    }

    // ArduPilot table for 80 MHz, ardupilot/libraries/AP_HAL_ChibiOS/CANFDIface.cpp
    if (target_data_bit_rate == 1000000) {
        data_timings->bit_rate_prescaler = 4;
        data_timings->bit_segment_1 = 14;
        data_timings->bit_segment_2 = 5;
        data_timings->sync_jump_width = 5;
    } else if (target_data_bit_rate == 2000000) {
        data_timings->bit_rate_prescaler = 2;
        data_timings->bit_segment_1 = 14;
        data_timings->bit_segment_2 = 5;
        data_timings->sync_jump_width = 5;
    } else if (target_data_bit_rate == 4000000) {
        data_timings->bit_rate_prescaler = 1;
        data_timings->bit_segment_1 = 14;
        data_timings->bit_segment_2 = 5;
        data_timings->sync_jump_width = 5;
    } else if (target_data_bit_rate == 5000000) {
        data_timings->bit_rate_prescaler = 1;
        data_timings->bit_segment_1 = 11;
        data_timings->bit_segment_2 = 4;
        data_timings->sync_jump_width = 4;
    } else if (target_data_bit_rate == 8000000) {
        data_timings->bit_rate_prescaler = 1;
        data_timings->bit_segment_1 = 6;
        data_timings->bit_segment_2 = 3;
        data_timings->sync_jump_width = 3;
    } else {
        return -DC_HAL_ERROR_UNSUPPORTED_BIT_RATE;
    }

    // let's do a check
    const uint32_t bit_time = (1 + data_timings->bit_segment_1 + data_timings->bit_segment_2) * data_timings->bit_rate_prescaler;
    const uint32_t expected = peripheral_clock_rate / target_data_bit_rate;
    if (bit_time != expected) {
        return -DC_HAL_ERROR_TIMING;
    }

    // transceiver delay compensation (TDC) is only needed for fast data prescalers
    // RM0492: TDC is intended for data prescaler 1 or 2, slower ones (prescaler > 2) do not need it
    // TDC offset is in FDCAN clock cycles (= 125 ns @ 80 MHz)
    // ArduPilot's value is 10 (tuned for a 120 ns MCP2557FD-class transceiver)
    data_timings->tdco = (data_timings->bit_rate_prescaler <= 2) ? 10 : 0;

    return 0;
}





/*
https://phryniszak.github.io/stm32g-fdcan/
https://github.com/phryniszak/stm32g-fdcan?tab=readme-ov-file
nominal:  TSeg1 = 127,  Tseg2 = 42, SJW = 42, Prescale = 1     75%
data:     TSeg1 = 7,    Tseg2 = 2,  SJW = 2, Prescale = 17     80%

https://kvaser.com/support/calculators/can-fd-bit-timing-calculator/
170 MHz, 500 ppm, 100ns
1MB/s 1MB/s
for ca 75%
nominal:  TSeg1 = 85 + 42 = 127,  Tseg2 = 42, SJW = 43, Prescale = 1
data:     TSeg1 = 126,            Tseg2 = 43, SJW = 43, Prescale = 1

STM32G4 examples
https://github.com/STMicroelectronics/STM32CubeG4/tree/master/Projects/STM32G474E-EVAL/Examples/FDCAN

https://github.com/pierremolinaro/acanfd-stm32

https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_ChibiOS/CANFDIface.cpp

https://github.com/am32-firmware/AM32/pull/36/files

https://github.com/ARMmbed/mbed-os/pull/13565/files
*/
/*
good source on CAN errors
https://www.csselectronics.com/pages/can-bus-errors-intro-tutorial
*/


#endif // USE_HAL_FDCAN_MODULE
#endif // STM32C552xx
