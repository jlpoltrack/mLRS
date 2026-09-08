//*******************************************************
// mLRS Wireless Bridge for RP2040/RP2350 (Pico W family)
// Copyright (c) www.olliw.eu, OlliW, OlliW42
// License: GPL v3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// Basic but effective & reliable transparent WiFi or Bluetooth <-> serial bridge.
// Minimizes wireless traffic while respecting latency by better packeting algorithm.
//*******************************************************
// 8. Sep. 2026
//*********************************************************/
// This is the port of esp/mlrs-wireless-bridge to boards with a CYW43439 radio,
// i.e. Raspberry Pi Pico W and Pico 2 W. It mirrors the ESP sketch closely, so
// changes can be carried over between the two.
// NOTES:
// - ArduinoIDE 2.3.2, arduino-pico (Raspberry Pi Pico/RP2040/RP2350) by Earle Philhower 4.x
// Dependencies:
// You need to have in File->Preferences->Additional Board managers URLs
// - https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
// Install
// - Boards Manager: Raspberry Pi Pico/RP2040/RP2350 by Earle F. Philhower, III
// Tools menu settings
// - Board: "Raspberry Pi Pico W" or "Raspberry Pi Pico 2W"
// - IP/Bluetooth Stack: MUST be "IPv4 + Bluetooth", the sketch always builds BT and BLE in
// - Country: leave at Worldwide, the WiFi channels the sketch uses are allowed everywhere

/*
Definitions:
- "module" refers to the physical hardware
- "board" refers to the board you need to select in the Arduino IDE menu Tools->Board

For more details on the modules see mlrs-wireless-bridge-boards.h

Differences to the ESP32 version:
- no ESP-NOW: that is an Espressif protocol, the CYW43439 cannot do it, AT+PROTOCOL=6 is
  answered with KO
- AT mode has no RESET line: a Pico W cannot be reset by the Tx module, and it is not
  flashed through the module either, so only the GPIO0 line is used
- AT mode settings are stored in the EEPROM emulation, not in ESP32 Preferences
- no WiFi channel 13: the CLM blob of the Pico W has no channels 12-13, so only 1, 6 and 11
  are available, and AT+WIFICHANNEL=13 is answered with KO
- flashing is via USB (BOOTSEL) or SWD, not via serial passthrough

Connections (Tx module <-> Pico W / Pico 2 W):
- Tx module TX  -> GP1 (RX0, = Serial1 RX)
- Tx module RX  <- GP0 (TX0, = Serial1 TX)
  (with USE_SERIAL2 the comm port is Serial2, i.e. GP9 = RX1 and GP8 = TX1)
- USB is never used for communication, it is always the debug output
- GND           <-> GND, mandatory, the two boards must share ground
- 5V            -> VSYS (pin 39), the Pico's onboard regulator makes 3.3V from it
  ATTENTION: when VSYS is fed, do not also plug in the USB port, unless the module's 5V
  is diode-ored into VSYS (a Schottky to VSYS is the standard way)
- AT mode line  -> GP2, only needed if GPIO0_IO is enabled, the Tx module drives it
  low to enter AT mode and high for normal operation. The pin is a plain input without
  pull, so leave GPIO0_IO commented out if the line is not wired, else a floating GP2
  will make the bridge randomly fall into AT mode.
- RUN (pin 30)  NOT connected. Unlike an ESP backpack, the Pico is neither reset nor
  flashed by the Tx module: AT+RESTART is done in software (watchdog reboot), and
  firmware goes in via USB (BOOTSEL) or SWD. Leave RUN open.
- 3V3 (pin 36) is an output, never feed it from the Tx module.
- the Pico's IOs are 3.3V and are NOT 5V tolerant, level shift a 5V serial

Troubleshooting:
- Compile error "Set Tools->IP/Bluetooth Stack to ..." or "This library needs Bluetooth
  enabled": set Tools->IP/Bluetooth Stack to "IPv4 + Bluetooth"
- No serial data: check GND is shared, and note the Pico W has 3.3V IOs and is NOT 5V tolerant
*/


//-------------------------------------------------------
// User configuration
//-------------------------------------------------------

// Module
// uncomment what you want, you must select one (and only one)
// (you also need to set the board in the Arduino IDE accordingly)
//#define MODULE_RP_PICO_W                      // board: Raspberry Pi Pico W
#define MODULE_RP_PICO_2W                     // board: Raspberry Pi Pico 2W

// Serial level
// uncomment, if you need inverted serial, e.g. for a FrSky R9M
// (the RP UART inverts in hardware, so this works on any pin)
//#define USE_SERIAL_INVERTED

// Wireless protocol
//     0 = WiFi TCP
//     1 = WiFi UDP
//     2 = Wifi UDPSTA
//     3 = Bluetooth
//     4 = Wifi UDPCl
//     5 = BLE
// Note: If GPIO0_IO is defined, then this only sets the default protocol
#define WIRELESS_PROTOCOL  1

// GPIO0 usage
// uncomment if your Tx module drives a line to the Pico which signals AT mode (aka AT mode)
// GP2 is used on both the Pico W and the Pico 2 W, see Connections above
//#define GPIO0_IO  2 // = GP2


//**********************//
//*** WiFi settings ***//

// For TCP, UDP (only for these two)
// ssid = "" results in a default name, like "mLRS-13427 AP UDP"
// password = "" makes it an open AP
String ssid = ""; // "mLRS AP"; // Wifi name
String password = ""; // "thisisgreat"; // WiFi password (min 8 chars)

IPAddress ip(192, 168, 4, 55); // connect to this IP // MissionPlanner default is 127.0.0.1, so enter 192.168.4.55 in MP

int port_tcp = 5760; // connect to this port per TCP // MissionPlanner default is 5760
int port_udp = 14550; // connect to this port per UDP // MissionPlanner default is 14550

// For UDPSTA, UDPCl (only for these two)
// for UDPSTA setting network_ssid = "" results in
// - a default name, like "mLRS-13427 STA UDP"
// - a default password which includes the mLRS bindphrase, like "mLRS-mlrs.0"
// for UDPCl both strings MUST be set to what your Wifi network requires
String network_ssid = ""; // name of your WiFi network
String network_password = ""; // password to access your WiFi network (min 8 chars)

IPAddress ip_udpcl(192, 168, 0, 164); // your network's IP (only for UDPCl) // MissionPlanner default is 127.0.0.1, so enter your home's IP in MP

int port_udpcl = 14550; // listens to this port per UDPCl (only for UDPCl) // MissionPlanner default is 14550

// Bind phrase, used for the default UDPSTA password
String bindphrase = "mlrs.0";

// WiFi channel (only for TCP, UDP)
// choose 1, 6, 11
// Note: unlike the ESP version there is no channel 13, the CLM blob of the Pico W has no
// channels 12-13 in any regulatory domain. Channels 1-11 are allowed in every domain.
#define WIFI_CHANNEL  6

// WiFi power (for all TCP, UDP, UDPSTA, UDPCl)
// this sets the power level for the WiFi protocols, in dBm, via the CYW43 "qtxpower" iovar
// Note: If GPIO0_IO is defined, this sets the power for the medium power option.
#define WIFI_POWER  5 // 0 is about the lowest which still works, 20 is the max


//**************************//
//*** Bluetooth settings ***//

// bluetooth_device_name = "" results in a default name, like "mLRS-13427 BT"
String bluetooth_device_name = ""; // name of your Bluetooth device as it will be seen by your operating system


//**************************//
//*** BLE settings ***//

// ble_device_name = "" results in a default name, like "mLRS-13427 BLE"
String ble_device_name = ""; // name of your BLE device as it will be seen by your operating system


//**************************//
//*** General settings ***//

// Baudrate
#define BAUD_RATE  115200

// Serial port usage
// debug output is always on the USB Serial, it is never used for communication
// comment for default behavior, which is Serial1 (GP0/GP1) for communication
//#define USE_SERIAL2 // use Serial2 (GP8/GP9) for communication


//-------------------------------------------------------
// Version
//-------------------------------------------------------

#define VERSION_STR  "v1.3.09" // to not get version salad, use what the current mLRS version is at the time


//-------------------------------------------------------
// Module details
//-------------------------------------------------------

#include "mlrs-wireless-bridge-boards.h"


//-------------------------------------------------------
// Includes
//-------------------------------------------------------

#if defined GPIO0_IO && (WIRELESS_PROTOCOL != 4) // UDPCl cannot be set via Tx module, so force it
    #define USE_AT_MODE
#endif

#if (WIRELESS_PROTOCOL < 0) || (WIRELESS_PROTOCOL > 5)
    #error WIRELESS_PROTOCOL must be 0...5, ESP-NOW (6) is not available on a CYW43439.
#endif
#if !defined ENABLE_CLASSIC || !defined ENABLE_BLE
    #error Set Tools->IP/Bluetooth Stack to "IPv4 + Bluetooth"!
#endif

#if defined USE_AT_MODE || (WIRELESS_PROTOCOL == 0)
    #define USE_WIRELESS_PROTOCOL_TCP
#endif
#if defined USE_AT_MODE || (WIRELESS_PROTOCOL == 1)
    #define USE_WIRELESS_PROTOCOL_UDP
#endif
#if defined USE_AT_MODE || (WIRELESS_PROTOCOL == 2)
    #define USE_WIRELESS_PROTOCOL_UDPSTA
#endif
#if defined USE_AT_MODE || (WIRELESS_PROTOCOL == 4)
    #define USE_WIRELESS_PROTOCOL_UDPCL
#endif
#if defined USE_AT_MODE || (WIRELESS_PROTOCOL == 3)
    #define USE_WIRELESS_PROTOCOL_BLUETOOTH
#endif
#if defined USE_AT_MODE || (WIRELESS_PROTOCOL == 5)
    #define USE_WIRELESS_PROTOCOL_BLE
#endif

#if !defined ARDUINO_ARCH_RP2040
    #error Must be a RP2040 or RP2350 board with a CYW43439, e.g. Raspberry Pi Pico W.
#endif

#include <WiFi.h>
#include <pico/cyw43_arch.h>
#include <pico/unique_id.h>

#if defined USE_WIRELESS_PROTOCOL_BLUETOOTH
    #include <SerialBT.h>
#endif
#if defined USE_WIRELESS_PROTOCOL_BLE
    #include <BluetoothLock.h> // scoped async context lock, btstack runs on the async context IRQ
    #include <btstack.h>
    #include "ble/gatt-service/nordic_spp_service_server.h"
#endif


//-------------------------------------------------------
// Internals
//-------------------------------------------------------

// TCP, UDP, UDPCl
IPAddress ip_gateway(0, 0, 0, 0);
IPAddress netmask(255, 255, 255, 0);
IPAddress ip_dns(0, 0, 0, 0);
// TCP
#ifdef USE_WIRELESS_PROTOCOL_TCP
WiFiServer server(port_tcp);
WiFiClient client;
#endif
// UDP, UDPSTA, UDPCl
#if defined USE_WIRELESS_PROTOCOL_UDP || defined USE_WIRELESS_PROTOCOL_UDPSTA || defined USE_WIRELESS_PROTOCOL_UDPCL
WiFiUDP udp;
#endif

typedef enum {
    WIRELESS_PROTOCOL_TCP = 0,
    WIRELESS_PROTOCOL_UDP = 1,
    WIRELESS_PROTOCOL_UDPSTA = 2,
    WIRELESS_PROTOCOL_BT = 3,
    WIRELESS_PROTOCOL_UDPCl = 4,
    WIRELESS_PROTOCOL_BLE = 5,
    WIRELESS_PROTOCOL_ESPNOW = 6, // not available on a CYW43439, only here to keep the AT commands aligned
} WIRELESS_PROTOCOL_ENUM;

typedef enum {
    WIFIPOWER_LOW = 0,
    WIFIPOWER_MED,
    WIFIPOWER_MAX,
} WIFIPOWER_ENUM;

#define PROTOCOL_DEFAULT  WIRELESS_PROTOCOL
#define BAUDRATE_DEFAULT  BAUD_RATE
#define WIFICHANNEL_DEFAULT  WIFI_CHANNEL
#define WIFIPOWER_DEFAULT  WIFIPOWER_MED

#define G_PROTOCOL_STR  "protocol"
int g_protocol = PROTOCOL_DEFAULT;
#define G_BAUDRATE_STR  "baudrate"
int g_baudrate = BAUDRATE_DEFAULT;
#define G_WIFICHANNEL_STR  "wifichannel"
int g_wifichannel = WIFICHANNEL_DEFAULT;
#define G_WIFIPOWER_STR  "wifipower"
int g_wifipower = WIFIPOWER_DEFAULT;
#define G_BINDPHRASE_STR  "bindphrase"
String g_bindphrase = bindphrase;
#define G_PASSWORD_STR  "password"
String g_password = "";
#define G_NETWORK_SSID_STR  "network_ssid"
String g_network_ssid = "";

uint16_t device_id = 0; // is going to be set by tWifiHandler::Init()
String device_name = "";
String device_name_STAUDP;
String device_password = "";

#ifdef USE_AT_MODE
#include "mlrs-wireless-bridge-preferences.h"
tPreferences preferences;
#include "mlrs-wireless-bridge-at-mode.h"
AtMode at_mode;
#endif

bool led_state;
unsigned long led_tlast_ms;
bool is_connected;
unsigned long is_connected_tlast_ms;


void serialFlushRx(void)
{
    while (SERIAL.available() > 0) { SERIAL.read(); }
}


// SerialUART has no bulk read(), so do it byte wise, it never blocks
int serialReadBuf(uint8_t* buf, int maxlen)
{
    int cnt = 0;
    while (cnt < maxlen) {
        int c = SERIAL.read();
        if (c < 0) break;
        buf[cnt++] = (uint8_t)c;
    }
    return cnt;
}


//-------------------------------------------------------
// CYW43 helpers
//-------------------------------------------------------

// Set TX power via the "qtxpower" iovar (value in quarter-dBm).
// The Arduino WiFi class has no API for it. Must be called after the interface is up.
void cyw43_set_txpower(int8_t dbm, int itf)
{
    uint8_t buf[9 + 4];
    memcpy(buf, "qtxpower\x00", 9);
    uint32_t q = (uint32_t)(dbm * 4);
    buf[9]  = q & 0xff;
    buf[10] = (q >> 8) & 0xff;
    buf[11] = (q >> 16) & 0xff;
    buf[12] = (q >> 24) & 0xff;
    cyw43_ioctl(&cyw43_state, CYW43_IOCTL_SET_VAR, sizeof(buf), buf, itf);
}


// dBm mapping mirrors setup_wifipower() in esp/mlrs-wireless-bridge
void setup_wifipower(int itf)
{
    static const int8_t power_dbm[] = { 0, WIFI_POWER, 20 }; // WIFIPOWER_LOW, _MED, _MAX
    int8_t dbm = (g_wifipower < (int)sizeof(power_dbm)) ? power_dbm[g_wifipower] : WIFI_POWER;
    cyw43_set_txpower(dbm, itf);
}


//-------------------------------------------------------
// BLE, Nordic UART Service
//-------------------------------------------------------
// The arduino-pico core has no BLE serial class, so the GATT server is built at runtime
// with btstack's att_db_util. UUIDs match those of the ESP wireless bridge, so a GCS
// works with either. btstack callbacks run in the async context IRQ on core 0, hence the
// data is handed over in fifos and btstack calls from loop() are wrapped in BluetoothLock.
#ifdef USE_WIRELESS_PROTOCOL_BLE

#define BLE_FIFO_SIZE  2048 // must be a power of 2

class tBleFifo {
  public:
    void Init(void) { head = tail = 0; }

    void Put(const uint8_t* data, int len) {
        for (int i = 0; i < len; i++) {
            uint16_t next = (head + 1) & (BLE_FIFO_SIZE - 1);
            if (next == tail) break; // fifo full, drop
            buf[head] = data[i];
            head = next;
        }
    }

    int Get(uint8_t* data, int maxlen) {
        int cnt = 0;
        while (tail != head && cnt < maxlen) {
            data[cnt++] = buf[tail];
            tail = (tail + 1) & (BLE_FIFO_SIZE - 1);
        }
        return cnt;
    }

    int Available(void) { return (head - tail) & (BLE_FIFO_SIZE - 1); }
    void Flush(void) { tail = head; }

    uint8_t buf[BLE_FIFO_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
};

tBleFifo ble_tx_fifo; // serial -> BLE
tBleFifo ble_rx_fifo; // BLE -> serial

// NUS UUIDs 6E40000x-B5A3-F393-E0A9-E50E24DCCA9E, big endian as btstack expects
static const uint8_t ble_nus_service_uuid[16] = {
    0x6E, 0x40, 0x00, 0x01, 0xB5, 0xA3, 0xF3, 0x93, 0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E };
static const uint8_t ble_nus_rx_uuid[16] = {
    0x6E, 0x40, 0x00, 0x02, 0xB5, 0xA3, 0xF3, 0x93, 0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E };
static const uint8_t ble_nus_tx_uuid[16] = {
    0x6E, 0x40, 0x00, 0x03, 0xB5, 0xA3, 0xF3, 0x93, 0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E };

volatile hci_con_handle_t ble_con_handle = HCI_CON_HANDLE_INVALID;
volatile bool ble_device_connected = false; // client connected AND notifications enabled
volatile bool ble_send_requested = false;

uint8_t ble_send_buf[512];
char ble_gap_name[32];

// adv data buffers must stay valid, btstack does not copy them
uint8_t ble_adv_data[31];
uint8_t ble_adv_data_len;
uint8_t ble_scan_resp_data[31];
uint8_t ble_scan_resp_data_len;


// nordic spp service events + RX data, called from btstack context
void ble_nordic_spp_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size)
{
    switch (packet_type) {
    case HCI_EVENT_PACKET:
        if (hci_event_packet_get_type(packet) != HCI_EVENT_GATTSERVICE_META) break;
        switch (hci_event_gattservice_meta_get_subevent_code(packet)) {
        case GATTSERVICE_SUBEVENT_SPP_SERVICE_CONNECTED:
            ble_con_handle = gattservice_subevent_spp_service_connected_get_con_handle(packet);
            ble_device_connected = true;
            break;
        case GATTSERVICE_SUBEVENT_SPP_SERVICE_DISCONNECTED:
            ble_con_handle = HCI_CON_HANDLE_INVALID;
            ble_device_connected = false;
            ble_send_requested = false; // a pending can-send-now will never arrive now
            break;
        }
        break;
    case RFCOMM_DATA_PACKET: // nordic spp delivers writes to the RX characteristic with this type
        ble_rx_fifo.Put(packet, size);
        break;
    }
}


// ATT events, called from btstack context
void ble_att_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size)
{
    if (packet_type != HCI_EVENT_PACKET) return;
    if (hci_event_packet_get_type(packet) != ATT_EVENT_CAN_SEND_NOW) return;

    if (!ble_device_connected) { ble_send_requested = false; return; }

    int len = ble_tx_fifo.Available();
    if (len == 0) { ble_send_requested = false; return; }

    uint16_t mtu = att_server_get_mtu(ble_con_handle);
    int max_len = (mtu > 3) ? mtu - 3 : 20; // payload per notification
    if (len > max_len) len = max_len;
    if (len > (int)sizeof(ble_send_buf)) len = sizeof(ble_send_buf);

    ble_tx_fifo.Get(ble_send_buf, len);
    nordic_spp_service_server_send(ble_con_handle, ble_send_buf, len);

    if (ble_tx_fifo.Available()) {
        att_server_request_can_send_now_event(ble_con_handle); // more pending
    } else {
        ble_send_requested = false;
    }
}


void ble_setup(String name)
{
    ble_tx_fifo.Init();
    ble_rx_fifo.Init();

    strncpy(ble_gap_name, name.c_str(), sizeof(ble_gap_name) - 1);
    ble_gap_name[sizeof(ble_gap_name) - 1] = '\0';

    BluetoothLock lock;

    l2cap_init();
    sm_init();

    // GATT DB: GAP service with device name, NUS service
    att_db_util_init();
    att_db_util_add_service_uuid16(GAP_SERVICE_UUID);
    att_db_util_add_characteristic_uuid16(GAP_DEVICE_NAME_UUID,
        ATT_PROPERTY_READ, ATT_SECURITY_NONE, ATT_SECURITY_NONE, (uint8_t*)ble_gap_name, strlen(ble_gap_name));
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

    uint8_t name_len = strlen(ble_gap_name);
    if (name_len > 29) name_len = 29;
    pos = 0;
    ble_scan_resp_data[pos++] = name_len + 1;
    ble_scan_resp_data[pos++] = BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME;
    memcpy(&ble_scan_resp_data[pos], ble_gap_name, name_len); pos += name_len;
    ble_scan_resp_data_len = pos;

    bd_addr_t null_addr = {};
    gap_advertisements_set_params(0x0030, 0x0060, 0, 0, null_addr, 0x07, 0x00);
    gap_advertisements_set_data(ble_adv_data_len, ble_adv_data);
    gap_scan_response_set_data(ble_scan_resp_data_len, ble_scan_resp_data);
    gap_advertisements_enable(1); // stays enabled, btstack re-advertises after a disconnect

    hci_power_control(HCI_POWER_ON);

    DBG_PRINTLN("BLE advertising started");
}

#endif // USE_WIRELESS_PROTOCOL_BLE


//-------------------------------------------------------
// Clients list (for UDP)
//-------------------------------------------------------
#ifdef USE_WIRELESS_PROTOCOL_UDP

#define UDP_CLIENTS_COUNT_MAX  3

class tClientList {
  public:
    struct tUdpClient {
        IPAddress ip;
        int port;
    };

    void Init(void) {
        clients_cnt = 0;
        for (int i = 0; i < UDP_CLIENTS_COUNT_MAX; i++) {
            clients[i].port = -1; // indicates that it is empty
        }
        gcs_seen = false;
    }

    void Add(IPAddress ip, int port, bool is_gcs) {
        for (int i = 0; i < clients_cnt; i++) {
            if (clients[i].ip == ip && clients[i].port == port) return; // found, is already in list
        }
        for (int i = 0; i < UDP_CLIENTS_COUNT_MAX; i++) {
            if (clients[i].port < 0) { // empty spot found
                clients[i].ip = ip;
                clients[i].port = port;
                clients_cnt++;
                if (is_gcs) gcs_seen = true;
                return; // added
            }
        }
    }

    bool HasGcs(void) {
        return gcs_seen;
    }

    int clients_cnt;
    tUdpClient clients[UDP_CLIENTS_COUNT_MAX];
    bool gcs_seen;
};

#endif // USE_WIRELESS_PROTOCOL_UDP


//-------------------------------------------------------
// WiFi setup helpers
//-------------------------------------------------------

void setup_ap_mode(IPAddress __ip)
{
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(__ip, ip_gateway, netmask);
    WiFi.softAP(device_name.c_str(), (device_password.length()) ? device_password.c_str() : nullptr, g_wifichannel);
    WiFi.noLowPowerMode(); // CYW43 power save adds tens of ms of latency, we don't want that
    setup_wifipower(CYW43_ITF_AP);
    DBG_PRINT("ap ip address: ");
    DBG_PRINTLN(WiFi.softAPIP()); // comes out as what was set with softAPConfig()
    DBG_PRINT("channel: ");
    DBG_PRINTLN(WiFi.channel());
}


// the network's broadcast address, arduino-pico's WiFi class has no broadcastIP()
IPAddress broadcast_ip(void)
{
    IPAddress lip = WiFi.localIP();
    IPAddress mask = WiFi.subnetMask();
    IPAddress bip;
    for (int i = 0; i < 4; i++) bip[i] = (lip[i] & mask[i]) | (~mask[i] & 0xFF);
    return bip;
}


// true: has connected, false: not yet connected, retry
bool setup_sta_mode_nonblocking(bool first, bool config_ip, IPAddress ip)
{
static unsigned long tlast_ms;

    if (first) {
        WiFi.mode(WIFI_STA);
        WiFi.disconnect();
        if (config_ip) {
            WiFi.config(ip, ip_dns, ip_gateway, netmask);
        }
        WiFi.begin(device_name.c_str(), device_password.c_str());
        tlast_ms = millis();
    }
    if (WiFi.status() == WL_CONNECTED) {
        WiFi.noLowPowerMode();
        setup_wifipower(CYW43_ITF_STA);
        DBG_PRINTLN("connected");
        DBG_PRINT("network ip address: ");
        DBG_PRINTLN(WiFi.localIP());
        return true;
    }
    if (millis() > tlast_ms + 1000) {
        tlast_ms = millis();
        DBG_PRINTLN("connecting to WiFi network...");
    }
    return false;
}


//-------------------------------------------------------
// Wifi Classes
//-------------------------------------------------------

//-------------------------------------------------------
//-- Wifi Base class
// note: g_ sould be already set up

class tWifiHandler {
  public:
    IPAddress _ip;
    int _port;
    unsigned long serial_data_received_tfirst_ms;
    int _setup_state; // 0: first call, 1: trying to connect, 2: done, some need a state machine

    void Init() {
        serial_data_received_tfirst_ms = 0;
        _setup_state = 0;

        // the RP2040/RP2350 flash id is unique per board and available without the radio being up
        pico_unique_board_id_t board_id;
        pico_get_unique_board_id(&board_id);
        uint8_t* id_buf = board_id.id; // 8 bytes
        for (uint8_t i = 0; i < 5; i++) device_id += id_buf[i] + ((uint16_t)id_buf[i + 1] << 8) / 39;
        device_id += id_buf[5];
        device_name = "mLRS-" + String(device_id);
#ifdef DEVICE_NAME_HEAD
        device_name = String(DEVICE_NAME_HEAD) + "-mLRS-" + String(device_id);
#endif
        // set STAUDP devicename here, to stay in sync with the ESP version
        if (g_network_ssid != "") { // definition in memory overwrites default
            device_name_STAUDP = g_network_ssid;
        } else { // we don't have any so set a default
            device_name_STAUDP = device_name + " STA UDP";
        }
    }

    void set_device_password(String forced_password, String std_password) {
        if (forced_password != "") {
            device_password = forced_password;
        } else if (g_password != "") {
            device_password = g_password;
        } else {
            device_password = std_password;
        }
    }

    void set_connected() {
        is_connected = true;
        is_connected_tlast_ms = millis();
    }

    void serial_read_wifi_write(uint8_t* buf, int sizeofbuf) {
        unsigned long tnow_ms = millis();
        int avail = SERIAL.available();
        if (avail <= 0) {
            serial_data_received_tfirst_ms = tnow_ms;
        } else
        if ((tnow_ms - serial_data_received_tfirst_ms) > 10 || avail > 128) { // 10 ms at 57600 bps corresponds to 57 bytes, no chance for 128 bytes
            serial_data_received_tfirst_ms = tnow_ms;
            int len = serialReadBuf(buf, sizeofbuf);
            wifi_write(buf, len);
        }
    }

    virtual void wifi_setup() {}
    virtual void wifi_write(uint8_t* buf, int len) {}

    void set_wifi_setup_trying() { _setup_state = 1; }
    void set_wifi_setup_done() { _setup_state = 2; }

    bool Setup() { // true: setup has completed and is not called anymore
        if (_setup_state >= 2) return true;
        wifi_setup();
        if (_setup_state == 1) return false;
        _setup_state = 2;
        return true;
    }

    bool IsSetUp() {
        return (_setup_state >= 2);
    }

    virtual void Loop(uint8_t* buf, int sizeofbuf) {}
};

tWifiHandler* wifi_handler;


//-------------------------------------------------------
//-- TCP class
#ifdef USE_WIRELESS_PROTOCOL_TCP

class tTCPHandler : public tWifiHandler {
  public:
    void Init(IPAddress __ip) {
        tWifiHandler::Init();
        device_name = (ssid != "") ? ssid : device_name + " AP TCP";
        set_device_password(password, "");
        _ip = __ip;
    }

    void wifi_setup() override {
        setup_ap_mode(_ip); // AP mode
        server.begin();
        server.setNoDelay(true);
        set_wifi_setup_done();
    }

    void Loop(uint8_t* buf, int sizeofbuf) override {
        if (server.hasClient()) {
            if (!client.connected()) {
                client.stop(); // doesn't appear to make a difference
                client = server.accept();
                DBG_PRINTLN("connection");
            } else { // is already connected, so reject, doesn't seem to ever happen
                server.accept().stop();
                DBG_PRINTLN("connection rejected");
            }
        }

        if (!client.connected()) { // nothing to do
            client.stop();
            serialFlushRx();
            is_connected = false;
            return;
        }

        while (client.available()) {
            int len = client.read(buf, sizeofbuf);
            if (len <= 0) break;
            SERIAL.write(buf, len);
            set_connected();
        }

        serial_read_wifi_write(buf, sizeofbuf);
    }

    void wifi_write(uint8_t* buf, int len) override {
        client.write(buf, len);
    }
};
tTCPHandler tcp_handler;
#endif


//-------------------------------------------------------
//-- UDP class
#ifdef USE_WIRELESS_PROTOCOL_UDP

class tUDPHandler : public tWifiHandler, tClientList {
  public:
    IPAddress _ip_ap;

    void Init(IPAddress __ip, int __port) {
        tWifiHandler::Init();
        tClientList::Init();
        device_name = (ssid != "") ? ssid : device_name + " AP UDP";
        set_device_password(password, "");
        _ip = _ip_ap = __ip;
        _ip[3] = 255; // start with broadcast, the subnet mask is 255.255.255.0 so just last octet needs to change
        _port = __port;
    }

    void wifi_setup() override {
        setup_ap_mode(_ip_ap); // AP mode
        udp.begin(_port);
        set_wifi_setup_done();
    }

    void Loop(uint8_t* buf, int sizeofbuf) override {
        int packetSize = udp.parsePacket();
        if (packetSize > 0) {
            int len = udp.read(buf, sizeofbuf);
            if (len > 0) { // let's assume that this is the GCS, so forward
                SERIAL.write(buf, len);
            }
            Add(udp.remoteIP(), udp.remotePort(), (len > 0)); // true if it's from a GCS
            set_connected(); // should we indicate connected only if we have seen a GCS?
        }
        serial_read_wifi_write(buf, sizeofbuf);
    }

    void wifi_write(uint8_t* buf, int len) override {
        if (!HasGcs()) {
            udp.beginPacket(_ip, _port);
            udp.write(buf, len);
            udp.endPacket();
        } else
        for (int i = 0; i < clients_cnt; i++) {
            udp.beginPacket(clients[i].ip, clients[i].port);
            udp.write(buf, len);
            udp.endPacket();
        }
    }
};
tUDPHandler udp_handler;
#endif


//-------------------------------------------------------
//-- UDPSTA class
// network_ssid, network_password
// g_bindphrase
#ifdef USE_WIRELESS_PROTOCOL_UDPSTA

class tUDPSTAHandler : public tWifiHandler {
  public:
    int _initial_port;

    void Init(int __port) {
        tWifiHandler::Init();
        device_name = (network_ssid != "") ? network_ssid : device_name_STAUDP;
        set_device_password(network_password, String("mLRS-") + g_bindphrase);
        _ip = IPAddress(255, 255, 255, 255); // start with broadcast, is refined once the GCS is known
        _port = _initial_port = __port;
    }

    void wifi_setup() override {
        bool res = setup_sta_mode_nonblocking((_setup_state == 0), false, IPAddress()); // STA mode, without config ip, so dummy ip
        set_wifi_setup_trying(); // switch to trying
        if (res) { // done
            _ip = broadcast_ip(); // the network's broadcast address, arduino-pico has no broadcastIP()
            udp.begin(_port);
            set_wifi_setup_done(); // we are actually connected, so signal done
        }
    }

    void Loop(uint8_t* buf, int sizeofbuf)  override {
        if (!is_connected && WiFi.status() != WL_CONNECTED) {
            udp.stop();
            _port = _initial_port;
            _setup_state = 0; // attempt to reconnect if WiFi got disconnected
            return;
        }

        int packetSize = udp.parsePacket();
        if (packetSize > 0) {
            int len = udp.read(buf, sizeofbuf);
            SERIAL.write(buf, len);
            if (!is_connected) { // first received UDP packet
                _ip = udp.remoteIP(); // stop broadcast, switch to unicast to avoid Aurdino performance issue
                _port = udp.remotePort();
            }
            set_connected();
        }

        serial_read_wifi_write(buf, sizeofbuf);
    }

    void wifi_write(uint8_t* buf, int len) override {
        udp.beginPacket(_ip, _port);
        udp.write(buf, len);
        udp.endPacket();
    }
};
tUDPSTAHandler udpsta_handler;
#endif


//-------------------------------------------------------
//-- UDPCl class
#ifdef USE_WIRELESS_PROTOCOL_UDPCL

class tUDPClHandler : public tWifiHandler {
  public:
    void Init(IPAddress __ip, int __port) {
        tWifiHandler::Init();
        device_name = network_ssid; // we only allow that specified in code
        device_password = network_password; // we only allow that specified in code
        _ip = __ip;
        _port = __port;
    }

    void wifi_setup() override {
        bool res = setup_sta_mode_nonblocking((_setup_state == 0), true, _ip); // STA mode, with config ip
        set_wifi_setup_trying(); // switch to trying
        if (res) { // done
            udp.begin(_port);
            set_wifi_setup_done(); // we are actually connected, so signal done
        }
    }

    void Loop(uint8_t* buf, int sizeofbuf)  override {
        int packetSize = udp.parsePacket();
        if (packetSize > 0) {
            int len = udp.read(buf, sizeofbuf);
            SERIAL.write(buf, len);
            set_connected();
        }

        if (!is_connected) {
            // we wait for a first message from the remote
            // remote's ip and port not known, so jump out
            serialFlushRx();
            return;
        }

        serial_read_wifi_write(buf, sizeofbuf);
    }

    void wifi_write(uint8_t* buf, int len) override {
        udp.beginPacket(udp.remoteIP(), udp.remotePort());
        udp.write(buf, len);
        udp.endPacket();
    }
};
tUDPClHandler udpcl_handler;
#endif


//-------------------------------------------------------
//-- BLUETOOTH class
#ifdef USE_WIRELESS_PROTOCOL_BLUETOOTH

class tBTClassicHandler : public tWifiHandler {
  public:
    void Init() {
        tWifiHandler::Init();
        device_name = (bluetooth_device_name != "") ? bluetooth_device_name : device_name + " BT";
    }

    void wifi_setup() override {
        SerialBT.setFIFOSize(4096);
        SerialBT.setName(device_name.c_str());
        SerialBT.begin();
        set_wifi_setup_done();
    }

    void Loop(uint8_t* buf, int sizeofbuf) override {
        int len = SerialBT.available();
        if (len > 0) {
            if (len > sizeofbuf) len = sizeofbuf;
            for (int i = 0; i < len; i++) buf[i] = SerialBT.read();
            SERIAL.write(buf, len);
            set_connected();
        }

        if (SerialBT.availableForWrite() <= 0) { // no client connected
            serialFlushRx();
            return;
        }

        serial_read_wifi_write(buf, sizeofbuf);
    }

    void wifi_write(uint8_t* buf, int len) override {
        SerialBT.write(buf, len);
    }
};
tBTClassicHandler bt_handler;
#endif


//-------------------------------------------------------
//-- BLE class
#ifdef USE_WIRELESS_PROTOCOL_BLE

class tBLEHandler : public tWifiHandler {
  public:
    void Init() {
        tWifiHandler::Init();
        device_name = (ble_device_name != "") ? ble_device_name : device_name + " BLE";
    }

    void wifi_setup() override {
        ble_setup(device_name);
        set_wifi_setup_done();
    }

    void Loop(uint8_t* buf, int sizeofbuf) override {
        if (!ble_device_connected) {
            ble_send_requested = false; // in case the link dropped between request and event
            ble_tx_fifo.Flush(); // no client, discard
            serialFlushRx();
            is_connected = false;
            return;
        }

        int len = ble_rx_fifo.Get(buf, sizeofbuf);
        if (len > 0) {
            SERIAL.write(buf, len);
            set_connected();
        }

        serial_read_wifi_write(buf, sizeofbuf);

        if (ble_tx_fifo.Available() && !ble_send_requested) {
            ble_send_requested = true;
            BluetoothLock lock;
            att_server_request_can_send_now_event(ble_con_handle);
        }
    }

    void wifi_write(uint8_t* buf, int len) override {
        ble_tx_fifo.Put(buf, len);
    }
};
tBLEHandler ble_handler;

#endif // USE_WIRELESS_PROTOCOL_BLE


//-------------------------------------------------------
// setup() and loop()
//-------------------------------------------------------

void setup()
{
    led_init();
    dbg_init();

    // Preferences
#ifdef USE_AT_MODE
    preferences.begin("setup", false);

    g_protocol = preferences.getInt(G_PROTOCOL_STR, 255); // 255 indicates not available
    if (g_protocol != WIRELESS_PROTOCOL_TCP && g_protocol != WIRELESS_PROTOCOL_UDP && g_protocol != WIRELESS_PROTOCOL_UDPSTA &&
        g_protocol != WIRELESS_PROTOCOL_UDPCl && g_protocol != WIRELESS_PROTOCOL_BT && g_protocol != WIRELESS_PROTOCOL_BLE) { // not a valid value
        g_protocol = PROTOCOL_DEFAULT;
        preferences.putInt(G_PROTOCOL_STR, g_protocol);
    }

    g_baudrate = preferences.getInt(G_BAUDRATE_STR, 0); // 0 indicates not available
    if (g_baudrate != 9600 && g_baudrate != 19200 && g_baudrate != 38400 &&
        g_baudrate != 57600 && g_baudrate != 115200 && g_baudrate != 230400) { // not a valid value
        g_baudrate = BAUDRATE_DEFAULT;
        preferences.putInt(G_BAUDRATE_STR, g_baudrate);
    }

    g_wifichannel = preferences.getInt(G_WIFICHANNEL_STR, 0); // 0 indicates not available
    if (g_wifichannel != 1 && g_wifichannel != 6 && g_wifichannel != 11) { // not a valid value, 13 is not supported
        g_wifichannel = WIFICHANNEL_DEFAULT;
        preferences.putInt(G_WIFICHANNEL_STR, g_wifichannel);
    }

    g_wifipower = preferences.getInt(G_WIFIPOWER_STR, 255); // 255 indicates not available
    if (g_wifipower < WIFIPOWER_LOW || g_wifipower > WIFIPOWER_MAX) { // not a valid value
        g_wifipower = WIFIPOWER_DEFAULT;
        preferences.putInt(G_WIFIPOWER_STR, g_wifipower);
    }

    g_bindphrase = preferences.getString(G_BINDPHRASE_STR, "mlrs.0"); // "mlrs.0" is the mLRS default bind phrase
    // TODO: we should check for sanity

    g_password = preferences.getString(G_PASSWORD_STR, ""); // "" is the default password
    g_network_ssid = preferences.getString(G_NETWORK_SSID_STR, ""); // "" is the default network ssid
#endif

    // Wifi handler
    switch (g_protocol) {
#ifdef USE_WIRELESS_PROTOCOL_TCP
        case WIRELESS_PROTOCOL_TCP: tcp_handler.Init(ip); wifi_handler = &tcp_handler; break;
#endif
#ifdef USE_WIRELESS_PROTOCOL_UDP
        case WIRELESS_PROTOCOL_UDP: udp_handler.Init(ip, port_udp); wifi_handler = &udp_handler; break;
#endif
#ifdef USE_WIRELESS_PROTOCOL_UDPSTA
        case WIRELESS_PROTOCOL_UDPSTA: udpsta_handler.Init(port_udp); wifi_handler = &udpsta_handler; break;
#endif
#ifdef USE_WIRELESS_PROTOCOL_UDPCL
        case WIRELESS_PROTOCOL_UDPCl: udpcl_handler.Init(ip_udpcl, port_udpcl); wifi_handler = &udpcl_handler; break;
#endif
#ifdef USE_WIRELESS_PROTOCOL_BLUETOOTH
        case WIRELESS_PROTOCOL_BT: bt_handler.Init(); wifi_handler = &bt_handler; break;
#endif
#ifdef USE_WIRELESS_PROTOCOL_BLE
        case WIRELESS_PROTOCOL_BLE: ble_handler.Init(); wifi_handler = &ble_handler; break;
#endif
    }

    // Serial
    SERIAL.setFIFOSize(2*1024); // must come before uart started
#ifdef SERIAL_RXD // if SERIAL_TXD is not defined the compiler will complain, so all good
    SERIAL.setRX(SERIAL_RXD);
    SERIAL.setTX(SERIAL_TXD);
#endif
#ifdef USE_SERIAL_INVERTED
    SERIAL.setInvertRX(true);
    SERIAL.setInvertTX(true);
#endif
    SERIAL.begin(g_baudrate);

    DBG_PRINTLN(g_protocol);
    DBG_PRINTLN(device_name);
    //DBG_PRINTLN(device_password);
    if (!wifi_handler) { DBG_PRINTLN("No protocol selected"); while(1){} }

    // Gpio0 handling
#ifdef USE_AT_MODE
    at_mode.Init(GPIO0_IO);
#endif

    led_tlast_ms = 0;
    led_state = false;

    is_connected = false;
    is_connected_tlast_ms = 0;

    serialFlushRx();
}


void loop()
{
#ifdef USE_AT_MODE
    if (at_mode.Do()) return;
#endif
    unsigned long tnow_ms = millis();

    if (is_connected && (tnow_ms - is_connected_tlast_ms > 2000)) { // nothing from GCS for 2 secs
        is_connected = false;
    }

    if (tnow_ms - led_tlast_ms > (is_connected ? 500 : (wifi_handler->IsSetUp()) ? 200 : 75)) {
        led_tlast_ms = tnow_ms;
        led_state = !led_state;
        if (led_state) led_on(is_connected); else led_off();
    }

    //-- here comes the core code, handle WiFi or Bluetooth connection and do the bridge

    uint8_t buf[256]; // working buffer

    if (!wifi_handler->Setup()) {
        return;
    }

    wifi_handler->Loop(buf, sizeof(buf));

    delay(2); // give it always a bit of time
}
