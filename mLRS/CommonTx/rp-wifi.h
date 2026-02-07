//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// WiFi serial destination for CYW43439 (Pico 2 W)
// low-level: direct CYW43 + lwIP, no Arduino WiFi dependency
// 7 Feb 2026
//*******************************************************
#ifndef RP_WIFI_H
#define RP_WIFI_H
#pragma once

#include <pico/cyw43_arch.h>
#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "lwip/netif.h"
#include "lwip/etharp.h"
#include "netif/ethernet.h"
#include "../Common/rp-lib/dhcpserver.h"
#include "../Common/rp-lib/crosscore_fifo.h"
#include "../Common/rp-lib/rp-mcu.h"
#include "../Common/rp-lib/clm_blob_1yn.h"


// wifi AP configuration
#define WIFI_AP_PORT              14550   // MAVLink standard UDP port
#define WIFI_AP_SSID_PREFIX       "mLRS AP"


// wifi protocol method strings for SSID generation
static const char* const wifi_method_str[] = {
    "TCP",     // WIFI_PROTOCOL_TCP
    "UDP",     // WIFI_PROTOCOL_UDP
    "BT",      // WIFI_PROTOCOL_BT
    "UDPSTA",  // WIFI_PROTOCOL_UDPSTA
    "BLE",     // WIFI_PROTOCOL_BLE
};


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
    return &cyw43_state.netif[CYW43_ITF_AP];
}


// Reload CLM blob at runtime — replaces stock 984-byte Pico W CLM with
// the 4752-byte Murata 1YN CLM that has proper per-country channel 12-13 support.
// Must be called after cyw43_arch_init() and before cyw43_wifi_set_up().
static bool clm_reload(void)
{
    const uint8_t* clm = clm_blob_1yn;
    const uint32_t clm_len = CLM_BLOB_1YN_LEN;
    const uint32_t chunk_size = 1024;  // must match CLM_CHUNK_LEN for SPI mode

    uint8_t buf[20 + 1024];  // header + max chunk
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
    cyw43_ioctl(&cyw43_state, CYW43_IOCTL_GET_VAR, 20, buf, CYW43_ITF_STA);
    uint32_t status = buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24);
    return (status == 0);
}


class tTxWifiNative : public tSerialBase
{
  public:
    // initialization (called from loop() on Core 0 via deferred init)
    void Init(uint8_t protocol, uint8_t channel, uint8_t power)
    {
        wifi_protocol = protocol;
        wifi_channel = channel;
        wifi_power = power;

        tx_fifo.Init();
        rx_fifo.Init();

        client_connected = false;
        remote_port = WIFI_AP_PORT;

        // generate SSID: "mLRS AP UDP XXXX"
        uint8_t sn[8];
        mcu_serial_number(sn);
        const char* method = wifi_method_str[wifi_protocol];
        snprintf(ssid, sizeof(ssid), "%s %s %02X%02X", WIFI_AP_SSID_PREFIX, method, sn[6], sn[7]);

        // map channel enum to actual WiFi channel number
        static const uint8_t channel_map[] = { 1, 6, 11, 13 };
        uint8_t ch = (wifi_channel < sizeof(channel_map)) ? channel_map[wifi_channel] : 1;

        // reload CLM with 1YN blob that supports channels 12-13
        clm_reload();

        // configure AP on CYW43 driver
        cyw43_wifi_ap_set_channel(&cyw43_state, ch);
        cyw43_wifi_ap_set_ssid(&cyw43_state, strlen(ssid), (const uint8_t*)ssid);
        cyw43_wifi_ap_set_auth(&cyw43_state, CYW43_AUTH_OPEN);

        // bring up AP with CYW43_COUNTRY_GERMANY — the 1YN CLM unlocks channels 1-13
        cyw43_wifi_set_up(&cyw43_state, CYW43_ITF_AP, true, CYW43_COUNTRY_GERMANY);

        // create lwIP netif in cyw43_state.netif[AP] so link callbacks work automatically
        ip4_addr_t ip, mask, gw;
        IP4_ADDR(&ip, 192, 168, 4, 1);
        IP4_ADDR(&mask, 255, 255, 255, 0);
        IP4_ADDR(&gw, 192, 168, 4, 1);

        cyw43_arch_lwip_begin();
        struct netif* n = &cyw43_state.netif[CYW43_ITF_AP];
        n->name[0] = 'w';
        n->name[1] = '0' + CYW43_ITF_AP;
        netif_add(n, &ip, &mask, &gw, &cyw43_state, wifi_netif_init, ethernet_input);
        netif_set_default(n);
        netif_set_up(n);
        netif_set_link_up(n);

        // start DHCP server
        ip_addr_t dhcp_ip, dhcp_mask;
        ip_addr_copy(dhcp_ip, n->ip_addr);
        ip_addr_copy(dhcp_mask, n->netmask);
        dhcp_server_init(&dhcp_srv, &dhcp_ip, &dhcp_mask);

        // broadcast address for initial TX (before client responds)
        IP4_ADDR(&remote_ip, 192, 168, 4, 255);

        // raw lwIP UDP socket for data transfer
        udp_pcb = udp_new();
        udp_bind(udp_pcb, IP_ADDR_ANY, WIFI_AP_PORT);
        udp_recv(udp_pcb, rx_callback, this);
        cyw43_arch_lwip_end();
    }

    // periodic processing (called from loop() on Core 0)
    void Do(void)
    {
        // drain TX FIFO -> send via UDP
        do_tx();

        // RX handled by lwIP callback — nothing to poll
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
        tx_fifo.Flush();
        rx_fifo.Flush();
    }

    // status
    bool IsConnected(void) { return client_connected; }

  private:
    void do_tx(void)
    {
        uint16_t avail = tx_fifo.AvailableCount();
        if (avail == 0) return;

        // drain FIFO into a local buffer, then send as one UDP packet
        char buf[256];
        uint16_t len = tx_fifo.GetBuf(buf, (avail < sizeof(buf)) ? avail : sizeof(buf));
        if (len > 0) {
            cyw43_arch_lwip_begin();
            struct pbuf* p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
            if (p) {
                memcpy(p->payload, buf, len);
                udp_sendto(udp_pcb, p, &remote_ip, remote_port);
                pbuf_free(p);
            }
            cyw43_arch_lwip_end();
        }
    }

    // lwIP receive callback — called from lwIP context
    static void rx_callback(void* arg, struct udp_pcb* pcb,
                            struct pbuf* p, const ip_addr_t* addr, u16_t port)
    {
        auto* self = (tTxWifiNative*)arg;

        // remember remote endpoint — switch from broadcast to unicast
        self->remote_ip = *addr;
        self->remote_port = port;
        self->client_connected = true;

        // copy pbuf chain into RX FIFO
        struct pbuf* q = p;
        while (q != nullptr) {
            self->rx_fifo.PutBuf((char*)q->payload, q->len);
            q = q->next;
        }
        pbuf_free(p);
    }

    // cross-core FIFOs
    tCrossCoreFifo<char, 4096> tx_fifo;  // Core 1 -> Core 0
    tCrossCoreFifo<char, 4096> rx_fifo;  // Core 0 -> Core 1

    // lwIP UDP socket
    struct udp_pcb* udp_pcb;

    // DHCP server
    dhcp_server_t dhcp_srv;

    // connection tracking
    bool client_connected;
    ip_addr_t remote_ip;
    uint16_t remote_port;

    // configuration
    uint8_t wifi_protocol;
    uint8_t wifi_channel;
    uint8_t wifi_power;
    char ssid[33];
};


tTxWifiNative wifi;  // global instance, accessed via extern from mlrs-tx.cpp

volatile bool wifi_requested = false;  // set by Core 1 after config loaded
static bool wifi_initialized = false;  // Core 0 local state

// config values captured by Core 1, read by Core 0
static uint8_t wifi_cfg_protocol;
static uint8_t wifi_cfg_channel;
static uint8_t wifi_cfg_power;


// called from main_loop() on Core 1 during init
void wifi_init(void)
{
    if (Setup.Tx[Config.ConfigId].SerialDestination == SERIAL_DESTINATION_WIFI) {
        wifi_cfg_protocol = Setup.Tx[Config.ConfigId].WifiProtocol;
        wifi_cfg_channel = Setup.Tx[Config.ConfigId].WifiChannel;
        wifi_cfg_power = Setup.Tx[Config.ConfigId].WifiPower;
        __dmb();  // ensure config values visible before flag
        wifi_requested = true;
    }
}


// called from loop() on Core 0 — handles both deferred init and polling
void wifi_loop(void)
{
    if (!wifi_initialized && wifi_requested) {
        __dmb();  // ensure we see config values written before flag
        wifi.Init(wifi_cfg_protocol, wifi_cfg_channel, wifi_cfg_power);
        wifi_initialized = true;
    }

    if (wifi_initialized) {
        wifi.Do();
    }
}


#endif // RP_WIFI_H
