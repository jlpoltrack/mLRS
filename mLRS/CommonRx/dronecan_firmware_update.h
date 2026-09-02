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

// firmware update requires an OTA hal, which currently only RP has
#if defined ARDUINO_ARCH_RP2040 || defined ARDUINO_ARCH_RP2350
#define DRONECAN_HAS_FIRMWARE_UPDATE
#endif

#endif // DEVICE_HAS_DRONECAN


#ifdef DRONECAN_HAS_FIRMWARE_UPDATE

#include "../modules/rp-lib/rp-ota.h"
#include "../Common/dronecan/out/include/uavcan.protocol.file.BeginFirmwareUpdate.h"
#include "../Common/dronecan/out/include/uavcan.protocol.file.Read.h"
#include "../Common/dronecan/out/include/uavcan.protocol.file.Error.h"
#include "../Common/dronecan/out/include/uavcan.protocol.file.Path.h"


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
        read_transfer_id = 0;
        pending_transfer_id = 0;
        awaiting_response = false;
        retry_count = 0;
        last_request_ms = 0;
        memset(&file_path, 0, sizeof(file_path));
        _clear_stats();
    }

    bool IsActive(void) { return (state != FWUPDATE_STATE_IDLE); }
    fwupdate_state_e State(void) { return state; }

    // stats — read from 1 Hz debug tick, no dbg calls inside callbacks
    uint16_t stats_chunks;
    uint16_t stats_errors;      // decode + server + write errors
    uint32_t stats_bytes;
    uint16_t stats_retries;
    uint16_t stats_stale;       // responses not matching the outstanding request, normal after a timeout

    //-- BeginFirmwareUpdate service handler

    // called from dronecan_on_transfer_received when a BeginFirmwareUpdate request arrives.
    // sends the response (accept/reject) and starts the update process.
    // returns true only when this request was accepted and started an update — not merely
    // "an update is active", which would also be true for a request rejected as in-progress
    // NOTE: no dbg output here — called from canardHandleRxFrame callback
    bool HandleBeginFirmwareUpdateRequest(
        CanardInstance* ins,
        CanardRxTransfer* transfer,
        uint8_t* buf,
        uint16_t buf_size)
    {
        if (state != FWUPDATE_STATE_IDLE) {
            // already in progress, reject
            _send_begin_firmware_update_response(ins, transfer, buf, buf_size,
                UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_IN_PROGRESS);
            return false;
        }

        // decode the request
        struct uavcan_protocol_file_BeginFirmwareUpdateRequest req;
        if (uavcan_protocol_file_BeginFirmwareUpdateRequest_decode(transfer, &req)) {
            // decode error, reject
            _send_begin_firmware_update_response(ins, transfer, buf, buf_size,
                UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_UNKNOWN);
            return false;
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
        awaiting_response = false;
        retry_count = 0;
        last_request_ms = 0;
        _clear_stats();
        return true;
    }


    //-- file.Read response handler

    // called from dronecan_on_transfer_received when a file.Read response arrives.
    // NOTE: no dbg output here — called from canardHandleRxFrame callback
    void HandleFileReadResponse(
        CanardRxTransfer* transfer)
    {
        if (state != FWUPDATE_STATE_DOWNLOADING) return;

        // only accept the response to the request that is currently outstanding. a service
        // response carries the transfer id of its request unaltered, so this is what tells
        // apart a response that arrives after its timeout from the response to the retry.
        // without it both get written and the chunk lands in the image twice, shifting
        // everything after it. the latch additionally drops a second copy of the response
        // we already consumed, which the transfer id alone would let through
        // transfer ids on the wire are 5 bit; canard happens to keep read_transfer_id in
        // 0..31 too (incrementTransferID wraps at 32), the mask makes us not depend on that
        if (!awaiting_response || transfer->transfer_id != (pending_transfer_id & 31)) {
            stats_stale++;
            return;
        }
        if (transfer->source_node_id != source_node_id) { // only from the node we asked
            stats_stale++;
            return;
        }
        awaiting_response = false;

        struct uavcan_protocol_file_ReadResponse res;
        if (uavcan_protocol_file_ReadResponse_decode(transfer, &res)) {
            stats_errors++;
            return;
        }

        // check for file server error
        if (res.error.value != UAVCAN_PROTOCOL_FILE_ERROR_OK) {
            stats_errors++;
            _abort();
            return;
        }

        // write the received data to flash
        if (res.data.len > 0) {
            if (!ota.ota_hal_write_chunk(res.data.data, res.data.len)) {
                stats_errors++;
                _abort();
                return;
            }
        }

        file_offset += res.data.len;
        stats_chunks++;
        stats_bytes = (uint32_t)file_offset;
        retry_count = 0;

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
            // initialize the OTA
            if (!ota.ota_hal_begin()) {
                stats_errors++;
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
                    stats_retries++;
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
            if (!ota.ota_hal_finish()) {
                stats_errors++;
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
    uint8_t read_transfer_id;       // transfer id for file.Read requests
    uint8_t pending_transfer_id;    // transfer id of the outstanding request, echoed by the response
    bool awaiting_response;         // cleared once a response for it has been consumed
    uint8_t retry_count;
    uint32_t last_request_ms;
    struct uavcan_protocol_file_Path file_path;

    tRpOta ota;


    void _clear_stats(void)
    {
        stats_chunks = 0;
        stats_errors = 0;
        stats_bytes = 0;
        stats_retries = 0;
        stats_stale = 0;
    }


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

        bool canfd = dc_hal_is_canfd();
        uint16_t len = uavcan_protocol_file_BeginFirmwareUpdateResponse_encode(&res, buf, !canfd);

        canardRequestOrRespond(
            ins,
            transfer->source_node_id,
            UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_SIGNATURE,
            UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ID,
            &transfer->transfer_id,
            transfer->priority,
            CanardResponse,
            buf,
            len,
            canfd);
    }


    void _send_file_read_request(
        CanardInstance* canard,
        uint8_t* buf,
        uint16_t buf_size)
    {
        struct uavcan_protocol_file_ReadRequest req;
        req.offset = file_offset;
        memcpy(&req.path, &file_path, sizeof(req.path));

        bool canfd = dc_hal_is_canfd();
        uint16_t len = uavcan_protocol_file_ReadRequest_encode(&req, buf, !canfd);

        // capture before the send: canard stamps the current read_transfer_id on the request
        // and only then increments it, and only if the enqueue succeeded
        pending_transfer_id = read_transfer_id;
        awaiting_response = true;

        canardRequestOrRespond(
            canard,
            source_node_id,
            UAVCAN_PROTOCOL_FILE_READ_REQUEST_SIGNATURE,
            UAVCAN_PROTOCOL_FILE_READ_REQUEST_ID,
            &read_transfer_id,
            CANARD_TRANSFER_PRIORITY_HIGH,
            CanardRequest,
            buf,
            len,
            canfd);
    }


    void _abort(void)
    {
        ota.ota_hal_abort();
        state = FWUPDATE_STATE_IDLE;
    }
};


#else // !DRONECAN_HAS_FIRMWARE_UPDATE

// stub when DroneCAN is not enabled, or when the platform has no OTA hal (all but RP)
class tDroneCanFirmwareUpdate
{
  public:
    void Init(void) {}
    bool IsActive(void) { return false; }
};

#endif // DRONECAN_HAS_FIRMWARE_UPDATE

#endif // DRONECAN_FIRMWARE_UPDATE_H
