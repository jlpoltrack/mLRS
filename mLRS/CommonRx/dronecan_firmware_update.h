//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// DroneCAN Firmware Update
// state machine for firmware update over CAN using
// DroneCAN BeginFirmwareUpdate + file.Read protocol
//*******************************************************
#ifndef DRONECAN_FIRMWARE_UPDATE_H
#define DRONECAN_FIRMWARE_UPDATE_H
#pragma once

#ifdef DEVICE_HAS_DRONECAN

#ifdef ESP32
#include "../Common/esp-lib/esp-ota.h"
#elif defined STM32G4
#include "../Common/hal/stm32/stm32-ota.h"
#endif


//-------------------------------------------------------
// firmware update state machine
//-------------------------------------------------------

#define FWUPDATE_READ_TIMEOUT_MS      2000  // timeout waiting for file.Read response
#define FWUPDATE_READ_MAX_RETRIES     5     // max retries per chunk before abort


typedef enum {
    FWUPDATE_STATE_IDLE = 0,
    FWUPDATE_STATE_STARTING,
    FWUPDATE_STATE_DOWNLOADING,
    FWUPDATE_STATE_FINALIZING,
} fwupdate_state_e;


class tDroneCanFirmwareUpdate
{
  public:

    void Init(void)
    {
        state = FWUPDATE_STATE_IDLE;
        source_node_id = 0;
        file_offset = 0;
        last_requested_offset = 0;
        read_transfer_id = 0;
        retry_count = 0;
        last_request_ms = 0;
        memset(&file_path, 0, sizeof(file_path));
    }

    bool IsActive(void) { return (state != FWUPDATE_STATE_IDLE); }
    fwupdate_state_e State(void) { return state; }

    //-- BeginFirmwareUpdate service handler

    // called from dronecan_on_transfer_received when a BeginFirmwareUpdate request arrives.
    // sends the response (accept/reject) and starts the update process.
    void HandleBeginFirmwareUpdateRequest(
        CanardInstance* ins,
        CanardRxTransfer* transfer,
        uint8_t* buf,
        uint16_t buf_size)
    {
        if (state != FWUPDATE_STATE_IDLE) {
            // already in progress, reject
            _send_begin_firmware_update_response(ins, transfer, buf, buf_size,
                UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_IN_PROGRESS);
            return;
        }

        // decode the request
        struct uavcan_protocol_file_BeginFirmwareUpdateRequest req;
        if (uavcan_protocol_file_BeginFirmwareUpdateRequest_decode(transfer, &req)) {
            // decode error, reject
            _send_begin_firmware_update_response(ins, transfer, buf, buf_size,
                UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_UNKNOWN);
            return;
        }

        // store source node id — if zero, use the sender's node id (per spec)
        source_node_id = req.source_node_id;
        if (source_node_id == 0) {
            source_node_id = transfer->source_node_id;
        }

        // store the file path for later file.Read requests
        memcpy(&file_path, &req.image_file_remote_path, sizeof(file_path));

        // accept the request
        _send_begin_firmware_update_response(ins, transfer, buf, buf_size,
            UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_OK);

        // transition to starting state
        state = FWUPDATE_STATE_STARTING;
        file_offset = 0;
        retry_count = 0;
        last_request_ms = 0;
        chunk_count = 0;
    }


    //-- file.Read response handler

    // called from dronecan_on_transfer_received when a file.Read response arrives
    void HandleFileReadResponse(
        CanardRxTransfer* transfer)
    {
        if (state != FWUPDATE_STATE_DOWNLOADING) return;

        struct uavcan_protocol_file_ReadResponse res;
        if (uavcan_protocol_file_ReadResponse_decode(transfer, &res)) {
            // decode error, will retry via timeout
            return;
        }

        // check for file server error
        if (res.error.value != UAVCAN_PROTOCOL_FILE_ERROR_OK) {
            _abort();
            return;
        }

        // guard against duplicate/stale responses. file.Read responses carry
        // no offset, so we track what we requested. if file_offset has already
        // advanced past the last-requested offset, this is a late duplicate
        // from a timed-out request — discard it.
        if (file_offset != last_requested_offset) {
            return;
        }

        // accommodate file servers that re-send the first chunk's data.
        // after chunk 0 is written, its first 8 bytes (vector table header) are
        // stored. any subsequent response whose first 8 bytes match is a
        // duplicate and gets discarded. the vector table SP + reset are unique
        // per firmware build, so false positives are essentially impossible.
        if (chunk_count > 0 && res.data.len >= 8) {
            uint64_t this_dword = 0;
            memcpy(&this_dword, res.data.data, 8);
            if (this_dword == first_chunk_dword) {
                return;
            }
        }

        // write the received data to OTA partition
        if (res.data.len > 0) {
            if (!_ota_write_chunk(res.data.data, res.data.len)) {
                _abort();
                return;
            }
        }

        // after writing chunk 0, store its header for duplicate detection
        if (chunk_count == 0 && res.data.len >= 8) {
            memcpy(&first_chunk_dword, res.data.data, 8);
        }

        file_offset += res.data.len;
        retry_count = 0;
        chunk_count++;

        // check for end of file — data.len < 256 means last chunk
        if (res.data.len < 256) {
            state = FWUPDATE_STATE_FINALIZING;
        } else {
            // request next chunk
            last_request_ms = 0; // force immediate re-request in tick
        }
    }


    //-- tick function, called from Tick_ms

    // drives the state machine: starts OTA, sends requests, handles timeouts.
    // canard, buf, buf_size are passed in from the owning tRxDroneCan.
    void Tick_ms(
        CanardInstance* canard,
        uint8_t* buf,
        uint16_t buf_size,
        uint32_t tnow_ms)
    {
        switch (state) {

        case FWUPDATE_STATE_STARTING:
            // initialize the OTA partition
            if (!_ota_begin()) {
                _abort();
                return;
            }
            state = FWUPDATE_STATE_DOWNLOADING;
            last_request_ms = 0; // send first request immediately
            // fall through to DOWNLOADING
            // no break

        case FWUPDATE_STATE_DOWNLOADING:
            // send file.Read request if needed (initial or retry on timeout)
            if (last_request_ms == 0 || (tnow_ms - last_request_ms) >= FWUPDATE_READ_TIMEOUT_MS) {
                if (last_request_ms != 0) {
                    retry_count++;
                    if (retry_count > FWUPDATE_READ_MAX_RETRIES) {
                        _abort();
                        return;
                    }
                }
                _send_file_read_request(canard, buf, buf_size);
                last_request_ms = tnow_ms;
            }
            break;

        case FWUPDATE_STATE_FINALIZING:
            // finalize OTA — does not return on success
            if (!_ota_finish()) {
                _abort();
            }
            // if we get here, finish() failed
            break;

        case FWUPDATE_STATE_IDLE:
            break;
        }
    }

  private:

    fwupdate_state_e state;
    uint8_t source_node_id;
    uint64_t file_offset;           // current read offset into the firmware image
    uint64_t last_requested_offset; // offset used in the last file.Read request
    uint8_t read_transfer_id;       // transfer id for file.Read requests
    uint8_t retry_count;
    uint32_t last_request_ms;
    struct uavcan_protocol_file_Path file_path;
    uint32_t chunk_count;
    uint64_t first_chunk_dword;    // first 8 bytes of chunk 0 for duplicate detection

#ifdef ESP32
    tEspOta ota;
#elif defined STM32G4
    tStm32Ota ota;
#endif


    void _send_begin_firmware_update_response(
        CanardInstance* ins,
        CanardRxTransfer* transfer,
        uint8_t* buf,
        uint16_t buf_size,
        uint8_t error_code)
    {
        struct uavcan_protocol_file_BeginFirmwareUpdateResponse res;
        res.error = error_code;
        res.optional_error_message.len = 0;

        uint16_t len = uavcan_protocol_file_BeginFirmwareUpdateResponse_encode(&res, buf);

        canardRequestOrRespond(
            ins,
            transfer->source_node_id,
            UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_SIGNATURE,
            UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ID,
            &transfer->transfer_id,
            transfer->priority,
            CanardResponse,
            buf,
            len);
    }


    void _send_file_read_request(
        CanardInstance* canard,
        uint8_t* buf,
        uint16_t buf_size)
    {
        struct uavcan_protocol_file_ReadRequest req;
        req.offset = file_offset;
        last_requested_offset = file_offset;
        memcpy(&req.path, &file_path, sizeof(req.path));

        uint16_t len = uavcan_protocol_file_ReadRequest_encode(&req, buf);

        canardRequestOrRespond(
            canard,
            source_node_id,
            UAVCAN_PROTOCOL_FILE_READ_REQUEST_SIGNATURE,
            UAVCAN_PROTOCOL_FILE_READ_REQUEST_ID,
            &read_transfer_id,
            CANARD_TRANSFER_PRIORITY_HIGH,
            CanardRequest,
            buf,
            len);
    }


    bool _ota_begin(void) { return ota.ota_hal_begin(); }

    bool _ota_write_chunk(const uint8_t* data, uint16_t len) { return ota.ota_hal_write_chunk(data, len); }

    bool _ota_finish(void) { return ota.ota_hal_finish(); }

    void _abort(void)
    {
        ota.ota_hal_abort();
        state = FWUPDATE_STATE_IDLE;
    }
};


#else // !DEVICE_HAS_DRONECAN

// stub when DroneCAN is not enabled
class tDroneCanFirmwareUpdate
{
  public:
    void Init(void) {}
    bool IsActive(void) { return false; }
};

#endif // DEVICE_HAS_DRONECAN

#endif // DRONECAN_FIRMWARE_UPDATE_H
