//*******************************************************
// mLRS Wireless Bridge for BW16 (RTL8720DN)
// Copyright (c) www.olliw.eu, OlliW, OlliW42
// License: GPL v3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// Basic but effective & reliable transparent WiFi or BLE <-> serial bridge.
// Minimizes wireless traffic while respecting latency by better packeting algorithm.
//*******************************************************
// 13. Mar. 2026
//*********************************************************/
// Adapted from the ESP32 wireless bridge for the BW16 (RTL8720DN / AmebaD).
//
// Dependencies:
// - AmebaD board package for Arduino IDE
//   Add to File->Preferences->Additional Board managers URLs:
//   https://github.com/ambiot/ambd_arduino/raw/master/Arduino_package/package_realtek.com_amebad_index.json
// - Install via Boards Manager: Realtek AmebaD Boards
// - Select board: BW16 (RTL8720DN)
// - Library Manager: FlashStorage_RTL8720 by Khoi Hoang

/*
Supported wireless protocols:
  0 = WiFi TCP
  1 = WiFi UDP
  2 = WiFi UDPSTA
  4 = WiFi UDPCl
  5 = BLE

Note: Bluetooth Classic (3) and ESP-NOW (6) are not available on RTL8720DN.
*/


//-------------------------------------------------------
// Includes (early, needed for IPAddress type in user config)
//-------------------------------------------------------

#include <WiFi.h>
#include <WiFiUdp.h>


//-------------------------------------------------------
// User configuration
//-------------------------------------------------------

// Wireless protocol
//     0 = WiFi TCP
//     1 = WiFi UDP
//     2 = Wifi UDPSTA
//     4 = Wifi UDPCl
//     5 = BLE
// Note: If GPIO0_IO is defined, then this only sets the default protocol
#define WIRELESS_PROTOCOL  1

// GPIO0 usage
// uncomment if your module supports the RESET and GPIO0 lines (aka AT mode)
//#define GPIO0_IO  PB3

//**********************//
//*** WiFi settings ***//

// For TCP, UDP (only for these two)
// ssid = "" results in a default name, like "mLRS-13427 AP UDP"
// password = "" makes it an open AP
String ssid = ""; // "mLRS AP"; // WiFi name
String password = ""; // "thisisgreat"; // WiFi password (min 8 chars)

IPAddress ip(192, 168, 4, 55); // connect to this IP

int port_tcp = 5760;  // connect to this port per TCP  // MissionPlanner default is 5760
int port_udp = 14550; // connect to this port per UDP // MissionPlanner default is 14550

// For UDPSTA, UDPCl (only for these two)
String network_ssid = "";     // name of your WiFi network
String network_password = ""; // password to access your WiFi network (min 8 chars)

IPAddress ip_udpcl(192, 168, 0, 164); // your network's IP (only for UDPCl)

int port_udpcl = 14550; // listens to this port per UDPCl (only for UDPCl)

// WiFi channel (for TCP, UDP AP mode)
// 2.4 GHz: choose 1, 6, 11, 13
// 5 GHz:   choose 36
// Channel 13 (2461-2483 MHz) has the least overlap with mLRS 2.4 GHz frequencies.
// Note: Channel 13 is generally not available in the US, where 11 is the maximum.
#define WIFI_CHANNEL  36

// WiFi power
// this sets the power level for the WiFi protocols
// Note: If GPIO0_IO is defined, this sets the power for the medium power option.
// Note: BW16 wifi power control is limited compared to ESP32
#define WIFI_POWER  1 // 0 = low, 1 = medium, 2 = max


//**************************//
//*** BLE settings ***//

// ble_device_name = "" results in a default name, like "mLRS-13427 BLE"
String ble_device_name = ""; // name of your BLE device


//**************************//
//*** General settings ***//

// Baudrate
#define BAUD_RATE  115200


//-------------------------------------------------------
// Version
//-------------------------------------------------------

#define VERSION_STR  "v1.3.09"


//-------------------------------------------------------
// Module details
//-------------------------------------------------------

#include "mlrs-wireless-bridge-boards.h"


//-------------------------------------------------------
// Helpers
//-------------------------------------------------------

// AmebaD UARTClassTwo only has single-byte read(), so we need a helper
int serial_read_bytes(uint8_t* buf, int maxlen)
{
    int cnt = 0;
    while (cnt < maxlen && SERIAL.available() > 0) {
        buf[cnt++] = SERIAL.read();
    }
    return cnt;
}

#if defined GPIO0_IO
  #define USE_AT_MODE
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
#if defined USE_AT_MODE || (WIRELESS_PROTOCOL == 5)
    #define USE_WIRELESS_PROTOCOL_BLE
#endif

// BLE support
#ifdef USE_WIRELESS_PROTOCOL_BLE
#include <BLEDevice.h>

#define BLE_SERVICE_UUID            "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // Nordic UART service
#define BLE_CHARACTERISTIC_UUID_RX  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_CHARACTERISTIC_UUID_TX  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEService ble_uart_service(BLE_SERVICE_UUID);
BLECharacteristic ble_rx_characteristic(BLE_CHARACTERISTIC_UUID_RX);
BLECharacteristic ble_tx_characteristic(BLE_CHARACTERISTIC_UUID_TX);

bool ble_device_connected = false;
bool ble_serial_started = false;
unsigned long ble_adv_tlast_ms = 0;

// forward declarations, needed for BLE callbacks
extern bool is_connected;
extern unsigned long is_connected_tlast_ms;

// ring buffer for BLE receive callback
#define BLE_RXBUF_SIZE  512
uint8_t ble_rxbuf[BLE_RXBUF_SIZE];
volatile uint16_t ble_rxbuf_head = 0;
volatile uint16_t ble_rxbuf_tail = 0;

void ble_rxbuf_push(const uint8_t* data, int len)
{
    for (int i = 0; i < len; i++) {
        uint16_t next = (ble_rxbuf_head + 1) & (BLE_RXBUF_SIZE - 1);
        if (next == ble_rxbuf_tail) break; // full, drop
        ble_rxbuf[ble_rxbuf_head] = data[i];
        ble_rxbuf_head = next;
    }
}

int ble_rxbuf_pop(uint8_t* buf, int maxlen)
{
    int cnt = 0;
    while (ble_rxbuf_tail != ble_rxbuf_head && cnt < maxlen) {
        buf[cnt++] = ble_rxbuf[ble_rxbuf_tail];
        ble_rxbuf_tail = (ble_rxbuf_tail + 1) & (BLE_RXBUF_SIZE - 1);
    }
    return cnt;
}

void ble_rx_callback(BLECharacteristic* chr, uint8_t conn_id)
{
    if (!ble_serial_started) {
        ble_serial_started = true;
    }
    uint16_t len = chr->getDataLen();
    if (len > 0) {
        const uint8_t* data = chr->readData8();
        if (data) {
            ble_rxbuf_push(data, len);
            is_connected = true;
            is_connected_tlast_ms = millis();
        }
    }
}

void ble_connect_callback(BLEDevice* dev)
{
    ble_device_connected = true;
    DBG_PRINTLN("BLE connected");
}

void ble_disconnect_callback(BLEDevice* dev)
{
    ble_device_connected = false;
    ble_serial_started = false;
    DBG_PRINTLN("BLE disconnected");
}

void ble_setup(String device_name_str)
{
    ble_device_connected = false;
    ble_serial_started = false;
    ble_adv_tlast_ms = 0;
    ble_rxbuf_head = 0;
    ble_rxbuf_tail = 0;

    BLEDevice::init(device_name_str);

    BLEDevice::configServer(1);
    BLEDevice::setDeviceName(device_name_str);

    ble_rx_characteristic.setWriteProperty(true);
    ble_rx_characteristic.setWriteNRProperty(true);
    ble_rx_characteristic.setWriteCallback(ble_rx_callback);
    ble_rx_characteristic.setBufferLen(256);

    ble_tx_characteristic.setReadProperty(true);
    ble_tx_characteristic.setNotifyProperty(true);
    ble_tx_characteristic.setBufferLen(256);

    ble_uart_service.addCharacteristic(ble_rx_characteristic);
    ble_uart_service.addCharacteristic(ble_tx_characteristic);

    BLEDevice::addService(ble_uart_service);

    BLEDevice::configAdvData()->addCompleteServices(BLEUUID(BLE_SERVICE_UUID));
    BLEDevice::configAdvData()->addFlags(GAP_ADTYPE_FLAGS_LIMITED | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED);

    BLEDevice::setConnectCallback(ble_connect_callback);
    BLEDevice::setDisconnectCallback(ble_disconnect_callback);

    BLEDevice::beginPeripheral();

    DBG_PRINTLN("BLE advertising started");
}
#endif // USE_WIRELESS_PROTOCOL_BLE


//-------------------------------------------------------
// Internals
//-------------------------------------------------------

// TCP, UDP, UDPCl
IPAddress ip_gateway(0, 0, 0, 0);
IPAddress netmask(255, 255, 255, 0);

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
    WIRELESS_PROTOCOL_UDPCl = 4,
    WIRELESS_PROTOCOL_BLE = 5,
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

#define G_PROTOCOL_STR    "protocol"
int g_protocol = PROTOCOL_DEFAULT;
#define G_BAUDRATE_STR    "baudrate"
int g_baudrate = BAUDRATE_DEFAULT;
#define G_WIFICHANNEL_STR "wifichannel"
int g_wifichannel = WIFICHANNEL_DEFAULT;
#define G_WIFIPOWER_STR   "wifipower"
int g_wifipower = WIFIPOWER_DEFAULT;
#define G_BINDPHRASE_STR  "bindphrase"
String g_bindphrase = "mlrs.0";
#define G_PASSWORD_STR    "password"
String g_password = "";
#define G_NETWORK_SSID_STR "network_ssid"
String g_network_ssid = "";

uint16_t device_id = 0;
String device_name = "";
String device_name_STAUDP;
String device_password = "";

bool settings_need_save = false;

#ifdef USE_AT_MODE
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
            clients[i].port = -1;
        }
        gcs_seen = false;
    }

    void Add(IPAddress ip, int port, bool is_gcs) {
        for (int i = 0; i < clients_cnt; i++) {
            if (clients[i].ip == ip && clients[i].port == port) return;
        }
        for (int i = 0; i < UDP_CLIENTS_COUNT_MAX; i++) {
            if (clients[i].port < 0) {
                clients[i].ip = ip;
                clients[i].port = port;
                clients_cnt++;
                if (is_gcs) gcs_seen = true;
                return;
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
// setup() and loop() helpers
//-------------------------------------------------------

extern "C" int wifi_set_country(unsigned long country_code);

void setup_wifipower()
{
    // TX power control is not available on the AmebaD Arduino platform.
    // wifi_set_txpower() is disabled (#if 0) in the SDK.
    // the RTL8720DN runs at default max TX power:
    //   ~17 dBm on 2.4 GHz, ~14 dBm on 5 GHz
}


void setup_ap_mode(IPAddress)
{
    // for AmebaD, apbegin sets up the AP and assigns the IP internally
    // note: __ip parameter is unused, AmebaD assigns the AP IP automatically
    // set EU region (ETSI1 = 0x21) to enable channels 1-13 and 5 GHz
    wifi_set_country(0x21);
    char ssid_buf[64];
    device_name.toCharArray(ssid_buf, sizeof(ssid_buf));
    char chan_buf[8];
    String(g_wifichannel).toCharArray(chan_buf, sizeof(chan_buf));
    if (device_password.length() >= 8) {
        // secured AP: 3-arg form (AmebaD requires password >= 8 chars)
        char pass_buf[64];
        device_password.toCharArray(pass_buf, sizeof(pass_buf));
        WiFi.apbegin(ssid_buf, pass_buf, chan_buf);
    } else {
        // open AP: 2-arg form (no password)
        WiFi.apbegin(ssid_buf, chan_buf);
    }
    setup_wifipower();
    DBG_PRINT("ap ip address: ");
    DBG_PRINTLN(WiFi.localIP());
    DBG_PRINT("channel: ");
    DBG_PRINTLN(g_wifichannel);
}


// true: has connected, false: not yet connected, retry
bool setup_sta_mode_nonblocking(bool first, bool /* config_ip */, IPAddress /* sta_ip */)
{
static unsigned long tlast_ms;

    if (first) {
        WiFi.disconnect();
        WiFi.begin((char*)device_name.c_str(), (char*)device_password.c_str());
        tlast_ms = millis();
    }
    if (WiFi.status() == WL_CONNECTED) {
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
// WiFi Classes
//-------------------------------------------------------

//-------------------------------------------------------
//-- WiFi Base class

// AmebaD low-level MAC access (works before WiFi.begin/apbegin)
extern "C" int wifi_get_mac_address(char* mac);

class tWifiHandler {
  public:
    IPAddress _ip;
    int _port;
    unsigned long serial_data_received_tfirst_ms;
    int _setup_state; // 0: first call, 1: trying to connect, 2: done

    void Init() {
        serial_data_received_tfirst_ms = 0;
        _setup_state = 0;

        // derive device_id from MAC address
        uint8_t MAC_buf[6] = {0};
        wifi_get_mac_address((char*)MAC_buf);
        device_id = 0;
        for (uint8_t i = 0; i < 5; i++) device_id += MAC_buf[i] + ((uint16_t)MAC_buf[i + 1] << 8) / 39;
        device_id += MAC_buf[5];
        device_name = "mLRS-" + String(device_id);

        // set STAUDP devicename here, as it may be needed in AT
        if (g_network_ssid != "") {
            device_name_STAUDP = g_network_ssid;
        } else {
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
        if ((tnow_ms - serial_data_received_tfirst_ms) > 10 || avail > 128) {
            serial_data_received_tfirst_ms = tnow_ms;
            int len = serial_read_bytes(buf, sizeofbuf);
            wifi_write(buf, len);
        }
    }

    virtual void wifi_setup() {}
    virtual void wifi_write(uint8_t* /* buf */, int /* len */) {}

    void set_wifi_setup_trying() { _setup_state = 1; }
    void set_wifi_setup_done() { _setup_state = 2; }

    bool Setup() { // true: setup has completed
        if (_setup_state >= 2) return true;
        wifi_setup();
        if (_setup_state == 1) return false;
        _setup_state = 2;
        return true;
    }

    bool IsSetUp() {
        return (_setup_state >= 2);
    }

    virtual void Loop(uint8_t* /* buf */, int /* sizeofbuf */) {}
};

tWifiHandler* wifi_handler = nullptr;


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
        set_wifi_setup_done();
    }

    void Loop(uint8_t* buf, int sizeofbuf) override {
        if (server.hasClient()) {
            if (!client.connected()) {
                client.stop();
                client = server.available();
                DBG_PRINTLN("connection");
            } else {
                server.available().stop();
                DBG_PRINTLN("connection rejected");
            }
        }

        if (!client.connected()) {
            client.stop();
            serialFlushRx();
            is_connected = false;
            return;
        }

        while (client.available()) {
            int len = client.read(buf, sizeofbuf);
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
        _ip[3] = 255; // start with broadcast
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
            if (len > 0) {
                SERIAL.write(buf, len);
            }
            Add(udp.remoteIP(), udp.remotePort(), (len > 0));
            set_connected();
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
#ifdef USE_WIRELESS_PROTOCOL_UDPSTA

class tUDPSTAHandler : public tWifiHandler {
  public:
    int _initial_port;

    void Init(int __port) {
        tWifiHandler::Init();
        device_name = (network_ssid != "") ? network_ssid : device_name_STAUDP;
        set_device_password(network_password, String("mLRS-") + g_bindphrase);
        _ip = IPAddress(255, 255, 255, 255); // start with broadcast
        _port = _initial_port = __port;
    }

    void wifi_setup() override {
        bool res = setup_sta_mode_nonblocking((_setup_state == 0), false, IPAddress());
        set_wifi_setup_trying();
        if (res) {
            setup_wifipower();
            udp.begin(_port);
            set_wifi_setup_done();
        }
    }

    void Loop(uint8_t* buf, int sizeofbuf) override {
        if (!is_connected && WiFi.status() != WL_CONNECTED) {
            udp.stop();
            _port = _initial_port;
            _setup_state = 0; // attempt to reconnect
            return;
        }

        int packetSize = udp.parsePacket();
        if (packetSize > 0) {
            int len = udp.read(buf, sizeofbuf);
            SERIAL.write(buf, len);
            if (!is_connected) {
                _ip = udp.remoteIP();
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
        device_name = network_ssid;
        device_password = network_password;
        _ip = __ip;
        _port = __port;
    }

    void wifi_setup() override {
        bool res = setup_sta_mode_nonblocking((_setup_state == 0), true, _ip);
        set_wifi_setup_trying();
        if (res) {
            setup_wifipower();
            udp.begin(_port);
            set_wifi_setup_done();
        }
    }

    void Loop(uint8_t* buf, int sizeofbuf) override {
        int packetSize = udp.parsePacket();
        if (packetSize > 0) {
            int len = udp.read(buf, sizeofbuf);
            SERIAL.write(buf, len);
            set_connected();
        }

        if (!is_connected) {
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
        unsigned long tnow_ms = millis();
        if (ble_device_connected) {
            // read from BLE rx ring buffer (filled by callback)
            int len = ble_rxbuf_pop(buf, sizeofbuf);
            if (len > 0) {
                SERIAL.write(buf, len);
                set_connected();
            }
            // read from serial and send via BLE
            int avail = SERIAL.available();
            if (avail <= 0) {
                serial_data_received_tfirst_ms = tnow_ms;
            } else
            if ((tnow_ms - serial_data_received_tfirst_ms) > 10 || avail > 128) {
                serial_data_received_tfirst_ms = tnow_ms;
                int send_len = (sizeofbuf < 244) ? sizeofbuf : 244; // BLE MTU limit
                int rlen = serial_read_bytes(buf, send_len);
                wifi_write(buf, rlen);
            }
        } else {
            serialFlushRx();
            is_connected = false;
            if (tnow_ms - ble_adv_tlast_ms > 5000) {
                ble_adv_tlast_ms = tnow_ms;
                // restart advertising if not connected
                BLEDevice::beginPeripheral();
            }
        }
    }

    void wifi_write(uint8_t* buf, int len) override {
        ble_tx_characteristic.setData(buf, len);
        ble_tx_characteristic.notify(0);
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

    // load persistent settings
#ifdef USE_AT_MODE
    tSettings stored;
    settings_load(stored);

    if (stored.magic == SETTINGS_MAGIC) {
        g_protocol = stored.protocol;
        g_baudrate = stored.baudrate;
        g_wifichannel = stored.wifichannel;
        g_wifipower = stored.wifipower;
        g_bindphrase = String(stored.bindphrase);
        g_password = String(stored.password);
        g_network_ssid = String(stored.network_ssid);
    }

    // validate protocol
    if (g_protocol != WIRELESS_PROTOCOL_TCP && g_protocol != WIRELESS_PROTOCOL_UDP &&
        g_protocol != WIRELESS_PROTOCOL_UDPSTA && g_protocol != WIRELESS_PROTOCOL_UDPCl &&
        g_protocol != WIRELESS_PROTOCOL_BLE) {
        g_protocol = PROTOCOL_DEFAULT;
    }

    // validate baudrate
    if (g_baudrate != 9600 && g_baudrate != 19200 && g_baudrate != 38400 &&
        g_baudrate != 57600 && g_baudrate != 115200 && g_baudrate != 230400) {
        g_baudrate = BAUDRATE_DEFAULT;
    }

    // validate wifi channel (2.4 GHz: 1,6,11,13; 5 GHz: 36)
    if (g_wifichannel != 1 && g_wifichannel != 6 && g_wifichannel != 11 && g_wifichannel != 13 &&
        g_wifichannel != 36) {
        g_wifichannel = WIFICHANNEL_DEFAULT;
    }

    // validate wifi power
    if (g_wifipower < WIFIPOWER_LOW || g_wifipower > WIFIPOWER_MAX) {
        g_wifipower = WIFIPOWER_DEFAULT;
    }
#endif

    // wifi handler
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
#ifdef USE_WIRELESS_PROTOCOL_BLE
        case WIRELESS_PROTOCOL_BLE: ble_handler.Init(); wifi_handler = &ble_handler; break;
#endif
    }

    // serial
    SERIAL.begin(g_baudrate);

    DBG_PRINTLN(g_protocol);
    DBG_PRINTLN(device_name);
    if (!wifi_handler) { DBG_PRINTLN("No protocol selected"); while(1){} }

    // GPIO0 handling
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

    //-- here comes the core code, handle WiFi or BLE connection and do the bridge

    uint8_t buf[256]; // working buffer

    if (!wifi_handler->Setup()) {
        return;
    }

    wifi_handler->Loop(buf, sizeof(buf));

    delay(2); // give it always a bit of time
}
