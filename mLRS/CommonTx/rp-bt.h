//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// Bluetooth wireless bridge for CYW43439 (Pico 2 W)
// BT classic SPP via arduino-pico SerialBT, BLE via btstack Nordic UART Service
// included by rp-wifi.h when built with PIO_FRAMEWORK_ARDUINO_ENABLE_BLUETOOTH,
// btstack is brought up by the rpipico2w variant boot on the shared cyw43 async context
// all functions run on Core 0, data crosses cores via wifi.tx_fifo/wifi.rx_fifo
//*******************************************************
#ifndef RP_BT_H
#define RP_BT_H
#pragma once

#include <SerialBT.h>
#include <cyw43_wrappers.h> // __lockBluetooth()/__unlockBluetooth(), btstack runs on the async context IRQ
#include <btstack.h>
#include "ble/gatt-service/nordic_spp_service_server.h"


static uint8_t bt_chunk_buf[512]; // bounds the blocking time of a single SPP write


//-------------------------------------------------------
//-- BT classic SPP
//-------------------------------------------------------

static void bt_spp_init(const char* name)
{
    SerialBT.setFIFOSize(4096); // RX buffer, matches the crosscore FIFO size
    SerialBT.setName(name);
    SerialBT.begin();
}


static void bt_spp_do(void)
{
    // drain TX FIFO -> SPP (SerialBT.write blocks until btstack accepted the chunk)
    uint16_t len = wifi.tx_fifo.AvailableCount();
    if (len > 0) {
        if (SerialBT.availableForWrite() > 0) { // a client is connected
            if (len > sizeof(bt_chunk_buf)) len = sizeof(bt_chunk_buf);
            wifi.tx_fifo.GetBuf((char*)bt_chunk_buf, len);
            SerialBT.write(bt_chunk_buf, len);
        } else {
            wifi.tx_fifo.Flush(); // no client, discard
        }
    }

    // SPP -> RX FIFO
    int avail = SerialBT.available();
    while (avail-- > 0) {
        int c = SerialBT.read();
        if (c < 0) break;
        wifi.rx_fifo.Put((char)c);
    }
}


//-------------------------------------------------------
//-- BLE, Nordic UART Service
//-------------------------------------------------------
// GATT DB is built at runtime with att_db_util, UUIDs match the esp wireless bridge
// so GCS BLE clients work with either. RX data and connection state arrive in btstack
// context (async context IRQ on Core 0), the TX pump uses the can-send-now pattern.

// NUS UUIDs 6E40000x-B5A3-F393-E0A9-E50E24DCCA9E, big endian as btstack expects
static const uint8_t ble_nus_service_uuid[16] = {
    0x6E, 0x40, 0x00, 0x01, 0xB5, 0xA3, 0xF3, 0x93, 0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E };
static const uint8_t ble_nus_rx_uuid[16] = {
    0x6E, 0x40, 0x00, 0x02, 0xB5, 0xA3, 0xF3, 0x93, 0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E };
static const uint8_t ble_nus_tx_uuid[16] = {
    0x6E, 0x40, 0x00, 0x03, 0xB5, 0xA3, 0xF3, 0x93, 0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E };

static volatile hci_con_handle_t ble_con_handle = HCI_CON_HANDLE_INVALID;
static volatile bool ble_connected = false;  // client connected AND notifications enabled
static volatile bool ble_send_requested = false;

static uint8_t ble_send_buf[512];

// adv data buffers must stay valid, btstack does not copy them
static uint8_t ble_adv_data[31];
static uint8_t ble_adv_data_len;
static uint8_t ble_scan_resp_data[31];
static uint8_t ble_scan_resp_data_len;


// nordic spp service events + RX data, called from btstack context
static void ble_nordic_spp_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size)
{
    switch (packet_type) {
    case HCI_EVENT_PACKET:
        if (hci_event_packet_get_type(packet) != HCI_EVENT_GATTSERVICE_META) break;
        switch (hci_event_gattservice_meta_get_subevent_code(packet)) {
        case GATTSERVICE_SUBEVENT_SPP_SERVICE_CONNECTED:
            ble_con_handle = gattservice_subevent_spp_service_connected_get_con_handle(packet);
            ble_connected = true;
            break;
        case GATTSERVICE_SUBEVENT_SPP_SERVICE_DISCONNECTED:
            ble_con_handle = HCI_CON_HANDLE_INVALID;
            ble_connected = false;
            break;
        }
        break;
    case RFCOMM_DATA_PACKET: // nordic spp delivers writes to the RX characteristic with this type
        wifi.rx_fifo.PutBuf((char*)packet, size);
        break;
    }
}


// ATT events, called from btstack context
static void ble_att_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size)
{
    if (packet_type != HCI_EVENT_PACKET) return;
    if (hci_event_packet_get_type(packet) != ATT_EVENT_CAN_SEND_NOW) return;

    if (!ble_connected) { ble_send_requested = false; return; }

    uint16_t len = wifi.tx_fifo.AvailableCount();
    if (len == 0) { ble_send_requested = false; return; }

    uint16_t mtu = att_server_get_mtu(ble_con_handle);
    uint16_t max_len = (mtu > 3) ? mtu - 3 : 20; // payload per notification
    if (len > max_len) len = max_len;
    if (len > sizeof(ble_send_buf)) len = sizeof(ble_send_buf);

    wifi.tx_fifo.GetBuf((char*)ble_send_buf, len);
    nordic_spp_service_server_send(ble_con_handle, ble_send_buf, len);

    if (wifi.tx_fifo.Available()) {
        att_server_request_can_send_now_event(ble_con_handle); // more pending
    } else {
        ble_send_requested = false;
    }
}


static void ble_init(const char* name)
{
    __lockBluetooth();

    l2cap_init();
    sm_init();

    // GATT DB: GAP service with device name, NUS service
    att_db_util_init();
    att_db_util_add_service_uuid16(GAP_SERVICE_UUID);
    att_db_util_add_characteristic_uuid16(GAP_DEVICE_NAME_UUID,
        ATT_PROPERTY_READ, ATT_SECURITY_NONE, ATT_SECURITY_NONE, (uint8_t*)name, strlen(name));
    att_db_util_add_service_uuid128(ble_nus_service_uuid);
    att_db_util_add_characteristic_uuid128(ble_nus_rx_uuid,
        ATT_PROPERTY_WRITE | ATT_PROPERTY_WRITE_WITHOUT_RESPONSE | ATT_PROPERTY_DYNAMIC,
        ATT_SECURITY_NONE, ATT_SECURITY_NONE, nullptr, 0);
    att_db_util_add_characteristic_uuid128(ble_nus_tx_uuid,
        ATT_PROPERTY_NOTIFY | ATT_PROPERTY_DYNAMIC, // NOTIFY adds the client config descriptor
        ATT_SECURITY_NONE, ATT_SECURITY_NONE, nullptr, 0);

    att_server_init(att_db_util_get_address(), nullptr, nullptr);
    att_server_register_packet_handler(ble_att_packet_handler);
    nordic_spp_service_server_init(ble_nordic_spp_packet_handler);

    // advertising: flags + NUS UUID, complete name in the scan response
    uint8_t pos = 0;
    ble_adv_data[pos++] = 2; ble_adv_data[pos++] = BLUETOOTH_DATA_TYPE_FLAGS; ble_adv_data[pos++] = 0x06;
    ble_adv_data[pos++] = 17; ble_adv_data[pos++] = BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS;
    for (uint8_t i = 0; i < 16; i++) ble_adv_data[pos++] = ble_nus_service_uuid[15 - i]; // little endian
    ble_adv_data_len = pos;

    uint8_t name_len = strlen(name);
    if (name_len > 29) name_len = 29;
    pos = 0;
    ble_scan_resp_data[pos++] = name_len + 1;
    ble_scan_resp_data[pos++] = BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME;
    memcpy(&ble_scan_resp_data[pos], name, name_len); pos += name_len;
    ble_scan_resp_data_len = pos;

    bd_addr_t null_addr = {};
    gap_advertisements_set_params(0x0030, 0x0060, 0, 0, null_addr, 0x07, 0x00);
    gap_advertisements_set_data(ble_adv_data_len, ble_adv_data);
    gap_scan_response_set_data(ble_scan_resp_data_len, ble_scan_resp_data);
    gap_advertisements_enable(1); // stays enabled, btstack re-advertises after a disconnect

    hci_power_control(HCI_POWER_ON);

    __unlockBluetooth();
}


static void ble_do(void)
{
    if (!ble_connected) {
        if (wifi.tx_fifo.Available()) wifi.tx_fifo.Flush(); // no client, discard
        return;
    }

    if (wifi.tx_fifo.Available() && !ble_send_requested) {
        ble_send_requested = true;
        __lockBluetooth();
        att_server_request_can_send_now_event(ble_con_handle);
        __unlockBluetooth();
    }
}


//-------------------------------------------------------
//-- protocol dispatch, called from tTxWifiNative on Core 0
//-------------------------------------------------------

void bt_init(uint8_t protocol, const char* name)
{
    if (protocol == WIFI_PROTOCOL_BLE) ble_init(name);
    else bt_spp_init(name);
}


void bt_do(uint8_t protocol)
{
    if (protocol == WIFI_PROTOCOL_BLE) ble_do();
    else bt_spp_do();
}


#endif // RP_BT_H
