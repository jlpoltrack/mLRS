//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// Wireless bridge for CYW43439 (Pico 2 W)
// low-level: direct CYW43 + lwIP, no Arduino WiFi dependency
// protocols: TCP (AP), UDP (AP), UDP STA; BT & BLE in rp-bt.h
//*******************************************************
#ifndef RP_WIFI_H
#define RP_WIFI_H
#pragma once

#ifndef PICO_CYW43_SUPPORTED
  #error DEVICE_HAS_CYW_WIFI requires a Pico W board variant (PICO_CYW43_SUPPORTED)!
#endif

#include <pico/cyw43_arch.h>
#include <pico/time.h>
#include "lwip/udp.h"
#include "lwip/tcp.h"
#include "lwip/dhcp.h"
#include "lwip/pbuf.h"
#include "lwip/netif.h"
#include "lwip/etharp.h"
#include "netif/ethernet.h"
#include "../modules/rp-lib/dhcpserver.h"
#include "../modules/rp-lib/crosscore_fifo.h"
#include "../modules/rp-lib/rp-mcu.h"
#include "../modules/rp-lib/clm_blob_1yn.h"


// wireless bridge configuration
// the protocol is selected via the Tx WifiProtocol parameter (allowed set in setup.h)
// ports match the esp wireless bridge so GCS workflows transfer unchanged
#define WIFI_TCP_PORT             5760    // MissionPlanner standard TCP port
#define WIFI_UDP_PORT             14550   // MAVLink standard UDP port
#define WIFI_UDP_PAYLOAD_LEN_MAX  1400    // keep a UDP datagram within one ethernet frame

// UDP STA: WiFi network to join, leave "" for the defaults, which are (like the esp bridge)
// - network name "mLRS STA UDP XXXX" (XXXX from serial number, published via info.wireless)
// - network password "mLRS-<bindphrase>", e.g. "mLRS-mlrs.0"
// so a phone hotspot can be set up to match without touching the code
#define WIFI_STA_NETWORK_SSID     ""
#define WIFI_STA_NETWORK_PASSWORD ""


// maps TX_WIFI_CHANNEL_ENUM to the actual WiFi channel number
static const uint8_t wifi_channel_map[] = { 1, 6, 11, 13 }; // must match "1,6,11,13" option list in setup_list.h
static_assert(sizeof(wifi_channel_map) == WIFI_CHANNEL_NUM,
    "wifi_channel_map[] out of sync with TX_WIFI_CHANNEL_ENUM in setup_types.h");


#if CYW43_ENABLE_BLUETOOTH
// implemented in rp-bt.h, included below the class
void bt_init(uint8_t protocol, const char* name);
void bt_do(uint8_t protocol);
#endif


// itf whose netif is registered with lwIP, set on Core 0 before the netif is added
static volatile int wifi_active_itf = CYW43_ITF_AP;


// netif link-layer output — sends frames through CYW43 driver
static err_t wifi_netif_output(struct netif* netif, struct pbuf* p)
{
    int itf = netif->name[1] - '0';
    int ret = cyw43_send_ethernet((cyw43_t*)netif->state, itf, p->tot_len, (void*)p, true);
    return ret ? ERR_IF : ERR_OK;
}

// netif init callback for netif_add
static err_t wifi_netif_init(struct netif* netif)
{
    netif->linkoutput = wifi_netif_output;
    netif->output = etharp_output;
    netif->mtu = 1500;
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP |
                   NETIF_FLAG_ETHERNET | NETIF_FLAG_IGMP;
    cyw43_wifi_get_mac((cyw43_t*)netif->state, netif->name[1] - '0', netif->hwaddr);
    netif->hwaddr_len = sizeof(netif->hwaddr);
    return ERR_OK;
}

// Strong override of the weak symbol in cyw43_wrappers.cpp
// Must be C++ linkage (no extern "C") to match the weak declaration
struct netif* __getCYW43Netif()
{
    return &cyw43_state.netif[wifi_active_itf];
}


// Reload CLM blob at runtime — replaces stock 984-byte Pico W CLM with
// the 4752-byte Murata 1YN CLM that has proper per-country channel 12-13 support.
// Must be called after cyw43_arch_init() and before cyw43_wifi_set_up().
static bool clm_reload(void)
{
    const uint8_t* clm = clm_blob_1yn;
    const uint32_t clm_len = CLM_BLOB_1YN_LEN;
    const uint32_t chunk_size = 1024;  // must match CLM_CHUNK_LEN for SPI mode

    uint8_t buf[20 + 1024 + 4];  // header + max chunk, padded so the 8-byte-aligned ioctl length stays in bounds
    uint32_t off = 0;

    while (off < clm_len) {
        uint32_t this_len = (clm_len - off < chunk_size) ? (clm_len - off) : chunk_size;
        uint16_t flags = 0x1000;  // DLOAD_HANDLER_VER
        if (off == 0) flags |= 0x0002;  // DL_BEGIN
        if (off + this_len >= clm_len) flags |= 0x0004;  // DL_END

        memcpy(buf, "clmload\x00", 8);
        buf[8]  = flags & 0xff;
        buf[9]  = (flags >> 8) & 0xff;
        buf[10] = 0x02; buf[11] = 0x00;  // type
        buf[12] = this_len & 0xff;
        buf[13] = (this_len >> 8) & 0xff;
        buf[14] = (this_len >> 16) & 0xff;
        buf[15] = (this_len >> 24) & 0xff;
        buf[16] = 0; buf[17] = 0; buf[18] = 0; buf[19] = 0;  // reserved
        memcpy(buf + 20, clm + off, this_len);

        uint32_t total = 20 + this_len;
        total = (total + 7) & ~7u;  // align to 8 bytes
        int ret = cyw43_ioctl(&cyw43_state, CYW43_IOCTL_SET_VAR, total, buf, CYW43_ITF_STA);
        if (ret != 0) return false;

        off += this_len;
    }

    // verify status
    memcpy(buf, "clmload_status\x00\x00\x00\x00\x00", 19);
    if (cyw43_ioctl(&cyw43_state, CYW43_IOCTL_GET_VAR, 20, buf, CYW43_ITF_STA) != 0) return false;
    uint32_t status = buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24);
    return (status == 0);
}


// Set TX power via "qtxpower" iovar (value in quarter-dBm).
// dBm mapping mirrors setup_wifipower() in esp/mlrs-wireless-bridge.
// Must be called after cyw43_wifi_set_up().
static void wifi_set_txpower(uint8_t power, int itf)
{
    static const int8_t power_dbm[] = { 0, 5, 20 };  // WIFI_POWER_LOW, _MED, _MAX
    int8_t dbm = (power < sizeof(power_dbm)) ? power_dbm[power] : 5;

    uint8_t buf[9 + 4];
    memcpy(buf, "qtxpower\x00", 9);
    uint32_t q = (uint32_t)(dbm * 4);
    buf[9]  = q & 0xff;
    buf[10] = (q >> 8) & 0xff;
    buf[11] = (q >> 16) & 0xff;
    buf[12] = (q >> 24) & 0xff;
    cyw43_ioctl(&cyw43_state, CYW43_IOCTL_SET_VAR, sizeof(buf), buf, itf);
}


class tTxWifiNative : public tSerialBase
{
  public:
    // initialization (called from loop() on Core 0 via deferred init)
    // note: the FIFOs are deliberately NOT reset here — the global object is zero-initialized,
    // and Core 1 may already be writing tx_fifo, whose head index Core 0 must never touch
    void Init(uint8_t _protocol, const char* name, const char* password, uint8_t channel, uint8_t power)
    {
        protocol = _protocol;
        remote_port = WIFI_UDP_PORT;

        switch (protocol) {
        case WIFI_PROTOCOL_TCP:
            ap_setup(name, channel, power);
            tcp_setup();
            break;
        case WIFI_PROTOCOL_UDPSTA:
            sta_setup(name, password, power);
            udp_setup();
            break;
#if CYW43_ENABLE_BLUETOOTH
        case WIFI_PROTOCOL_BT:
        case WIFI_PROTOCOL_BLE:
            bt_init(protocol, name);
            break;
#endif
        default: // WIFI_PROTOCOL_UDP
            ap_setup(name, channel, power);
            udp_setup();
            break;
        }
    }

    // periodic processing (called from loop() on Core 0)
    void Do(void)
    {
        if (tx_flush_request) {
            tx_flush_request = false;
            tx_fifo.Flush();
        }

        switch (protocol) {
        case WIFI_PROTOCOL_TCP:
            do_tx_tcp();
            break;
        case WIFI_PROTOCOL_UDPSTA:
            sta_poll();
            do_tx_udp();
            break;
#if CYW43_ENABLE_BLUETOOTH
        case WIFI_PROTOCOL_BT:
        case WIFI_PROTOCOL_BLE:
            bt_do(protocol);
            break;
#endif
        default: // WIFI_PROTOCOL_UDP
            do_tx_udp();
            break;
        }
        // RX handled by lwIP/btstack callbacks — nothing to poll
    }

    // tSerialBase interface (called from Core 1 via sx_serial)
    bool available(void) override
    {
        return rx_fifo.Available();
    }

    char getc(void) override
    {
        return rx_fifo.Get();
    }

    void putbuf(uint8_t* const buf, uint16_t len) override
    {
        tx_fifo.PutBuf((char*)buf, len);
    }

    void flush(void) override
    {
        rx_fifo.Flush(); // Core 1 owns rx_fifo's read side
        tx_flush_request = true; // tx_fifo's read side is owned by Core 0, flushed in Do()
    }

    // cross-core FIFOs, public so the rp-bt.h pump can access them (Core 0 only)
    tCrossCoreFifo<char, 4096> tx_fifo;  // Core 1 -> Core 0
    tCrossCoreFifo<char, 4096> rx_fifo;  // Core 0 -> Core 1

  private:
    //-- AP modes (TCP, UDP): AP netif 192.168.4.1/24 + DHCP server

    void ap_setup(const char* ssid, uint8_t channel, uint8_t power)
    {
        uint8_t ch = (channel < WIFI_CHANNEL_NUM) ? wifi_channel_map[channel] : 1;

        // reload CLM with 1YN blob that supports channels 12-13
        if (!clm_reload() && ch > 11) ch = 11; // stock CLM allows only channels 1-11

        wifi_active_itf = CYW43_ITF_AP;

        // configure AP on CYW43 driver
        cyw43_wifi_ap_set_channel(&cyw43_state, ch);
        cyw43_wifi_ap_set_ssid(&cyw43_state, strlen(ssid), (const uint8_t*)ssid);
        cyw43_wifi_ap_set_auth(&cyw43_state, CYW43_AUTH_OPEN);

        // bring up AP with CYW43_COUNTRY_GERMANY — the 1YN CLM unlocks channels 1-13
        cyw43_wifi_set_up(&cyw43_state, CYW43_ITF_AP, true, CYW43_COUNTRY_GERMANY);

        wifi_set_txpower(power, CYW43_ITF_AP);

        // create lwIP netif in cyw43_state.netif[AP] so link callbacks work automatically
        // AP subnet is defined here once, gw/broadcast/DHCP range all derive from it
        ip4_addr_t ip, mask;
        IP4_ADDR(&ip, 192, 168, 4, 1);
        IP4_ADDR(&mask, 255, 255, 255, 0);

        cyw43_arch_lwip_begin();
        struct netif* n = &cyw43_state.netif[CYW43_ITF_AP];
        n->name[0] = 'w';
        n->name[1] = '0' + CYW43_ITF_AP;
        netif_add(n, &ip, &mask, &ip, &cyw43_state, wifi_netif_init, ethernet_input);
        netif_set_default(n);
        netif_set_up(n);
        netif_set_link_up(n);

        // start DHCP server
        ip_addr_t dhcp_ip, dhcp_mask;
        ip_addr_copy(dhcp_ip, n->ip_addr);
        ip_addr_copy(dhcp_mask, n->netmask);
        dhcp_server_init(&dhcp_srv, &dhcp_ip, &dhcp_mask);
        cyw43_arch_lwip_end();

        // subnet broadcast address for initial TX (until first client packet arrives)
        ip4_addr_set_u32(&remote_ip,
            (ip4_addr_get_u32(&ip) & ip4_addr_get_u32(&mask)) | ~ip4_addr_get_u32(&mask));

        link_ready = true;
    }

    //-- STA mode (UDP STA): join a WiFi network, DHCP client, auto-reconnect

    void sta_setup(const char* ssid, const char* password, uint8_t power)
    {
        clm_reload(); // 1YN CLM also allows joining networks on channels 12-13

        wifi_active_itf = CYW43_ITF_STA;

        cyw43_wifi_set_up(&cyw43_state, CYW43_ITF_STA, true, CYW43_COUNTRY_GERMANY);

        wifi_set_txpower(power, CYW43_ITF_STA);

        cyw43_arch_lwip_begin();
        struct netif* n = &cyw43_state.netif[CYW43_ITF_STA];
        n->name[0] = 'w';
        n->name[1] = '0' + CYW43_ITF_STA;
        netif_add(n, IP4_ADDR_ANY4, IP4_ADDR_ANY4, IP4_ADDR_ANY4, &cyw43_state, wifi_netif_init, ethernet_input);
        netif_set_default(n);
        netif_set_up(n);
        netif_set_link_up(n); // frames flow only after join, DHCP retries until then
#if LWIP_NETIF_HOSTNAME
        netif_set_hostname(n, "mLRS");
#endif
        dhcp_start(n);
        cyw43_arch_lwip_end();

        strncpy(sta_ssid, ssid, sizeof(sta_ssid) - 1); // buffers are zero-initialized, stays terminated
        strncpy(sta_password, password, sizeof(sta_password) - 1);

        sta_join();
        sta_state = STA_JOINING;
        sta_tlast_ms = to_ms_since_boot(get_absolute_time());
    }

    void sta_join(void)
    {
        cyw43_wifi_join(&cyw43_state, strlen(sta_ssid), (const uint8_t*)sta_ssid,
            strlen(sta_password), (const uint8_t*)sta_password,
            CYW43_AUTH_WPA2_AES_PSK, nullptr, CYW43_CHANNEL_NONE);
    }

    // non-blocking connect/reconnect state machine, called from Do()
    void sta_poll(void)
    {
        uint32_t tnow_ms = to_ms_since_boot(get_absolute_time());
        int status = cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA);

        switch (sta_state) {
        case STA_JOINING:
            if (status == CYW43_LINK_JOIN) {
                struct netif* n = &cyw43_state.netif[CYW43_ITF_STA];
                if (!ip4_addr_isany_val(*netif_ip4_addr(n))) { // DHCP bound
                    // TX to subnet broadcast until first client packet arrives
                    ip4_addr_set_u32(&remote_ip,
                        (ip4_addr_get_u32(netif_ip4_addr(n)) & ip4_addr_get_u32(netif_ip4_netmask(n))) |
                        ~ip4_addr_get_u32(netif_ip4_netmask(n)));
                    remote_port = WIFI_UDP_PORT;
                    link_ready = true;
                    sta_state = STA_CONNECTED;
                }
            } else if (tnow_ms - sta_tlast_ms > 5000) { // join failed or still down, retry
                sta_tlast_ms = tnow_ms;
                sta_join();
            }
            break;
        case STA_CONNECTED:
            if (status != CYW43_LINK_JOIN) { // connection lost
                link_ready = false;
                sta_state = STA_JOINING;
                sta_tlast_ms = tnow_ms;
                sta_join();
            }
            break;
        }
    }

    //-- UDP data transfer (UDP, UDP STA)

    void udp_setup(void)
    {
        cyw43_arch_lwip_begin();
        udp_pcb = udp_new();
        udp_bind(udp_pcb, IP_ADDR_ANY, WIFI_UDP_PORT);
        udp_recv(udp_pcb, udp_rx_callback, this);
        cyw43_arch_lwip_end();
    }

    void do_tx_udp(void)
    {
        uint16_t len = tx_fifo.AvailableCount();
        if (len == 0) return;

        if (!link_ready) { // STA not (yet) connected, discard
            tx_fifo.Flush();
            return;
        }

        if (len > WIFI_UDP_PAYLOAD_LEN_MAX) len = WIFI_UDP_PAYLOAD_LEN_MAX;

        cyw43_arch_lwip_begin();
        struct pbuf* p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
        if (p) {
            tx_fifo.GetBuf((char*)p->payload, len); // PBUF_RAM payload is contiguous
            udp_sendto(udp_pcb, p, &remote_ip, remote_port);
            pbuf_free(p);
        }
        cyw43_arch_lwip_end();
    }

    // lwIP receive callback — called from lwIP context
    static void udp_rx_callback(void* arg, struct udp_pcb* pcb,
                                struct pbuf* p, const ip_addr_t* addr, u16_t port)
    {
        auto* self = (tTxWifiNative*)arg;

        // remember remote endpoint — switch from broadcast to unicast
        self->remote_ip = *addr;
        self->remote_port = port;

        // copy pbuf chain into RX FIFO
        struct pbuf* q = p;
        while (q != nullptr) {
            self->rx_fifo.PutBuf((char*)q->payload, q->len);
            q = q->next;
        }
        pbuf_free(p);
    }

    //-- TCP data transfer (TCP): server, single client

    void tcp_setup(void)
    {
        cyw43_arch_lwip_begin();
        struct tcp_pcb* l = tcp_new_ip_type(IPADDR_TYPE_V4);
        tcp_bind(l, IP_ADDR_ANY, WIFI_TCP_PORT);
        tcp_listen_pcb = tcp_listen_with_backlog(l, 1);
        tcp_arg(tcp_listen_pcb, this);
        tcp_accept(tcp_listen_pcb, tcp_accept_callback);
        cyw43_arch_lwip_end();
    }

    void do_tx_tcp(void)
    {
        uint16_t len = tx_fifo.AvailableCount();
        if (len == 0) return;

        cyw43_arch_lwip_begin();
        if (tcp_client_pcb == nullptr) { // no client, discard
            cyw43_arch_lwip_end();
            tx_fifo.Flush();
            return;
        }
        uint16_t space = tcp_sndbuf(tcp_client_pcb);
        if (len > space) len = space;
        if (len > sizeof(tcp_tx_buf)) len = sizeof(tcp_tx_buf);
        if (len > 0) {
            tx_fifo.GetBuf((char*)tcp_tx_buf, len);
            tcp_write(tcp_client_pcb, tcp_tx_buf, len, TCP_WRITE_FLAG_COPY);
            tcp_output(tcp_client_pcb);
        }
        cyw43_arch_lwip_end();
    }

    // called from lwIP context
    static err_t tcp_accept_callback(void* arg, struct tcp_pcb* newpcb, err_t err)
    {
        auto* self = (tTxWifiNative*)arg;
        if ((err != ERR_OK) || (newpcb == nullptr) || (self == nullptr)) return ERR_VAL;

        // single client: a new connection replaces the current one
        if (self->tcp_client_pcb != nullptr) {
            struct tcp_pcb* old = self->tcp_client_pcb;
            self->tcp_client_pcb = nullptr;
            tcp_arg(old, nullptr);
            tcp_recv(old, nullptr);
            tcp_err(old, nullptr);
            tcp_abort(old); // frees the pcb, sends RST
        }

        tcp_arg(newpcb, self);
        tcp_recv(newpcb, tcp_recv_callback);
        tcp_err(newpcb, tcp_err_callback);
        tcp_nagle_disable(newpcb);
        self->tcp_client_pcb = newpcb;
        return ERR_OK;
    }

    // called from lwIP context
    static err_t tcp_recv_callback(void* arg, struct tcp_pcb* tpcb, struct pbuf* p, err_t err)
    {
        auto* self = (tTxWifiNative*)arg;

        if (p == nullptr) { // remote closed the connection
            if (self != nullptr && self->tcp_client_pcb == tpcb) self->tcp_client_pcb = nullptr;
            tcp_arg(tpcb, nullptr);
            tcp_recv(tpcb, nullptr);
            tcp_err(tpcb, nullptr);
            if (tcp_close(tpcb) != ERR_OK) {
                tcp_abort(tpcb);
                return ERR_ABRT;
            }
            return ERR_OK;
        }

        if (self != nullptr) {
            struct pbuf* q = p;
            while (q != nullptr) {
                self->rx_fifo.PutBuf((char*)q->payload, q->len);
                q = q->next;
            }
        }
        tcp_recved(tpcb, p->tot_len);
        pbuf_free(p);
        return ERR_OK;
    }

    // called from lwIP context, the pcb is already freed
    static void tcp_err_callback(void* arg, err_t err)
    {
        auto* self = (tTxWifiNative*)arg;
        if (self != nullptr) self->tcp_client_pcb = nullptr;
    }

    //-- state

    uint8_t protocol;

    volatile bool tx_flush_request;  // set by Core 1 flush(), serviced by Core 0 Do()

    bool link_ready;  // AP up resp. STA joined with IP, gates UDP TX

    // lwIP sockets
    struct udp_pcb* udp_pcb;
    struct tcp_pcb* tcp_listen_pcb;
    struct tcp_pcb* tcp_client_pcb;  // written from lwIP context, read on Core 0 under lwip lock
    uint8_t tcp_tx_buf[TCP_MSS];

    // DHCP server (AP modes)
    dhcp_server_t dhcp_srv;

    // STA connect state machine
    enum { STA_JOINING = 0, STA_CONNECTED };
    uint8_t sta_state;
    uint32_t sta_tlast_ms;
    char sta_ssid[32+1];
    char sta_password[64];

    // remote endpoint, broadcast until first packet received
    ip_addr_t remote_ip;
    uint16_t remote_port;
};


extern tTxWifiNative wifi; // wbridge serial port, defined in mlrs-tx.cpp, routed in tSerialPorts::Init()

#if CYW43_ENABLE_BLUETOOTH
#include "rp-bt.h" // implements bt_init()/bt_do() using wifi.tx_fifo/wifi.rx_fifo
#endif

extern tSetup Setup;
extern tGlobalConfig Config;

static volatile bool wifi_requested = false;  // set by Core 1 after config loaded
static bool wifi_initialized = false;  // Core 0 local state

// config values captured by Core 1, read by Core 0
static uint8_t wifi_cfg_protocol;
static uint8_t wifi_cfg_channel;
static uint8_t wifi_cfg_power;
static char wifi_cfg_name[32+1] = "";  // AP ssid / STA network name / BT,BLE device name
static char wifi_cfg_password[64] = "";  // STA network password


static const char* wifi_ssid(void) // "" when the wireless bridge is not enabled
{
    return wifi_cfg_name;
}


// called from main_loop() on Core 1 during init
void wifi_init(void)
{
    if (Setup.Tx[Config.ConfigId].SerialPort != TX_SERIAL_PORT_WIRELESS_BRIDGE) return;

    wifi_cfg_protocol = Setup.Tx[Config.ConfigId].WifiProtocol;
    wifi_cfg_channel = Setup.Tx[Config.ConfigId].WifiChannel;
    wifi_cfg_power = Setup.Tx[Config.ConfigId].WifiPower;

    uint8_t sn[8];
    mcu_serial_number(sn);
    char sn_str[5];
    snprintf(sn_str, sizeof(sn_str), "%02X%02X", sn[6], sn[7]);

    // device resp. network name, also published via info.wireless
    switch (wifi_cfg_protocol) {
    case WIFI_PROTOCOL_TCP:
        snprintf(wifi_cfg_name, sizeof(wifi_cfg_name), "mLRS AP TCP %s", sn_str);
        break;
    case WIFI_PROTOCOL_UDPSTA:
        if (WIFI_STA_NETWORK_SSID[0] != '\0') {
            snprintf(wifi_cfg_name, sizeof(wifi_cfg_name), "%s", WIFI_STA_NETWORK_SSID);
        } else {
            snprintf(wifi_cfg_name, sizeof(wifi_cfg_name), "mLRS STA UDP %s", sn_str);
        }
        if (WIFI_STA_NETWORK_PASSWORD[0] != '\0') {
            snprintf(wifi_cfg_password, sizeof(wifi_cfg_password), "%s", WIFI_STA_NETWORK_PASSWORD);
        } else {
            snprintf(wifi_cfg_password, sizeof(wifi_cfg_password), "mLRS-%s", Setup.Common[Config.ConfigId].BindPhrase);
        }
        break;
    case WIFI_PROTOCOL_BT:
        snprintf(wifi_cfg_name, sizeof(wifi_cfg_name), "mLRS BT %s", sn_str);
        break;
    case WIFI_PROTOCOL_BLE:
        snprintf(wifi_cfg_name, sizeof(wifi_cfg_name), "mLRS BLE %s", sn_str);
        break;
    default: // WIFI_PROTOCOL_UDP
        snprintf(wifi_cfg_name, sizeof(wifi_cfg_name), "mLRS AP UDP %s", sn_str);
        break;
    }

    __dmb();  // ensure config values visible before flag
    wifi_requested = true;
}


// called from loop() on Core 0 — handles both deferred init and polling
void wifi_loop(void)
{
    if (!wifi_initialized && wifi_requested) {
        __dmb();  // ensure we see config values written before flag
        wifi.Init(wifi_cfg_protocol, wifi_cfg_name, wifi_cfg_password, wifi_cfg_channel, wifi_cfg_power);
        wifi_initialized = true;
    }

    if (wifi_initialized) {
        wifi.Do();
    }
}


#endif // RP_WIFI_H
