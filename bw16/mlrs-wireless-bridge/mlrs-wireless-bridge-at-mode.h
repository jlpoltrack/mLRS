//*******************************************************
// mLRS Wireless Bridge for BW16 (RTL8720DN)
// Copyright (c) www.olliw.eu, OlliW, OlliW42
// License: GPL v3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// AT Mode
//*******************************************************
// 13. Mar. 2026


// forward declarations
void serialFlushRx(void);


//-------------------------------------------------------
// Persistent storage via FlashStorage
//-------------------------------------------------------
// settings struct stored atomically in flash.
// a magic byte is used to detect first-run / uninitialized flash.

#include <FlashStorage_RTL8720.h>

#define SETTINGS_MAGIC  0xA5

struct tSettings {
    uint8_t magic;
    int protocol;
    int baudrate;
    int wifichannel;
    int wifipower;
    char bindphrase[8];   // 6 chars + null + pad
    char password[26];    // 24 chars + null + pad
    char network_ssid[26]; // 24 chars + null + pad
};

FlashStorage(flash_settings, tSettings);

void settings_load(tSettings& s)
{
    flash_settings.read(s);
}

void settings_save(const tSettings& s)
{
    flash_settings.write(s);
}


//-------------------------------------------------------
// AT command definitions
//-------------------------------------------------------

typedef enum {
    AT_NAME_QUERY = 0,
    AT_RESTART,
    AT_BAUD_QUERY,
    AT_BAUD_9600,
    AT_BAUD_19200,
    AT_BAUD_38400,
    AT_BAUD_57600,
    AT_BAUD_115200,
    AT_BAUD_230400,
    AT_WIFICHANNEL_QUERY,
    AT_WIFICHANNEL_1,
    AT_WIFICHANNEL_6,
    AT_WIFICHANNEL_11,
    AT_WIFICHANNEL_13,
    AT_WIFICHANNEL_36,
    AT_WIFIPOWER_QUERY,
    AT_WIFIPOWER_0_LOW,
    AT_WIFIPOWER_1_MED,
    AT_WIFIPOWER_2_MAX,
    AT_PROTOCOL_QUERY,
    AT_PROTOCOL_0_TCP,
    AT_PROTOCOL_1_UDP,
    AT_PROTOCOL_2_UDPSTA,
    AT_PROTOCOL_4_UDPCl,
    AT_PROTOCOL_5_BLE,
    AT_WIFIDEVICEID_QUERY,
    AT_WIFIDEVICENAME_QUERY,
    AT_BINDPHRASE_QUERY,
    AT_BINDPHRASE_XXXXXX,
    AT_PSWD_QUERY,
    AT_PSWD_XXXXXX,
    AT_NETSSID_QUERY,
    AT_NETSSID_XXXXXX,
    AT_CMDS_NUM,
} AT_NAME_ENUM;

const char* at_cmds[AT_CMDS_NUM] = {
     "AT+NAME=?",
     "AT+RESTART",
     "AT+BAUD=?",
     "AT+BAUD=9600",
     "AT+BAUD=19200",
     "AT+BAUD=38400",
     "AT+BAUD=57600",
     "AT+BAUD=115200",
     "AT+BAUD=230400",
     "AT+WIFICHANNEL=?",
     "AT+WIFICHANNEL=01",
     "AT+WIFICHANNEL=06",
     "AT+WIFICHANNEL=11",
     "AT+WIFICHANNEL=13",
     "AT+WIFICHANNEL=36",   // 5 GHz
     "AT+WIFIPOWER=?",
     "AT+WIFIPOWER=0",
     "AT+WIFIPOWER=1",
     "AT+WIFIPOWER=2",
     "AT+PROTOCOL=?",
     "AT+PROTOCOL=0",
     "AT+PROTOCOL=1",
     "AT+PROTOCOL=2",
     "AT+PROTOCOL=4",
     "AT+PROTOCOL=5",
     "AT+WIFIDEVICEID=?",
     "AT+WIFIDEVICENAME=?",
     "AT+BINDPHRASE=?",
     "AT+BINDPHRASE=xxxxxx",
     "AT+PSWD=?",
     "AT+PSWD=xxxxxxxxxxxxxxxxxxxxxxxx",
     "AT+NETSSID=?",
     "AT+NETSSID=xxxxxxxxxxxxxxxxxxxxxxxx",
};


//-------------------------------------------------------
// AT Mode class
//-------------------------------------------------------

class AtMode {
  public:
    void Init(uint8_t _gpio_pin);
    bool Do(void);

  private:
    uint8_t gpio0_pin;
    bool gpio_is_low;
    unsigned long startup_tmo_ms;
    uint8_t at_pos;
    char at_buf[128];

    bool restart_needed;

    void restart(void);
};


void AtMode::Init(uint8_t _gpio_pin)
{
    gpio0_pin = _gpio_pin;
    pinMode(gpio0_pin, INPUT);

    at_pos = 0;
    gpio_is_low = false;
    restart_needed = false;

    startup_tmo_ms = 750 + millis(); // stay in AT loop for at least 750 ms
}


bool AtMode::Do(void)
{
    // handle toggles in GPIO0
    if (!gpio_is_low) {
        if (digitalRead(gpio0_pin) == LOW) { // toggled LOW
            gpio_is_low = true;
            at_pos = 0;
            restart_needed = false;
            serialFlushRx();
        }
    } else {
        if (digitalRead(gpio0_pin) == HIGH) { // toggled HIGH
            gpio_is_low = false;
            startup_tmo_ms = 0; // also declare end of startup timeout phase
            serialFlushRx();
        }
    }

    // handle startup timeout
    if (startup_tmo_ms) {
        if (millis() > startup_tmo_ms) startup_tmo_ms = 0;
    }

    // when not in startup phase and not GPIO0 low, go to WiFi loop
    if (!startup_tmo_ms && !gpio_is_low) return false;

    int avail = SERIAL.available();
    if (avail > 0) {
        char c = SERIAL.read();

        if (at_pos > 96) at_pos = 0; // argh ...

        at_buf[at_pos++] = c;
        at_buf[at_pos] = '\0'; // to make it a str
        bool possible_match = false;
        for (int i = 0; i < AT_CMDS_NUM; i++) {
            char at_cmd[64];
            strcpy(at_cmd, at_cmds[i]);

            // adjust cmd for bindphrase, password, netssid
            if (i == AT_BINDPHRASE_XXXXXX) {
                for (int pp = 14; pp < 14+6; pp++) if (pp < at_pos) at_cmd[pp] = at_buf[pp];
            }
            if (i == AT_PSWD_XXXXXX) {
                for (int pp = 8; pp < 8+24; pp++) if (pp < at_pos) at_cmd[pp] = at_buf[pp];
            }
            if (i == AT_NETSSID_XXXXXX) {
                for (int pp = 11; pp < 11+24; pp++) if (pp < at_pos) at_cmd[pp] = at_buf[pp];
            }

            // check if it could become a match by reading more chars
            if (strncmp(at_buf, at_cmd, at_pos) != 0) continue;
            possible_match = true;

            // check if it is a full match already, and if so take action
            if (strcmp(at_buf, at_cmd) == 0) {
                if (i == AT_NAME_QUERY || i == AT_BAUD_QUERY || i == AT_WIFICHANNEL_QUERY || i == AT_WIFIPOWER_QUERY ||
                    i == AT_PROTOCOL_QUERY || i == AT_WIFIDEVICEID_QUERY || i == AT_WIFIDEVICENAME_QUERY ||
                    i == AT_BINDPHRASE_QUERY || i == AT_PSWD_QUERY || i == AT_NETSSID_QUERY) {
                    at_buf[0] = 'O';
                    at_buf[1] = 'K';
                    SERIAL.write(at_buf, at_pos - 1); // don't send the '?'
                    switch (i) {
                        case AT_NAME_QUERY:
                            SERIAL.write("mLRS-Wireless-Bridge-BW16-");
                            SERIAL.write(VERSION_STR);
                            break;
                        case AT_BAUD_QUERY: SERIAL.print(g_baudrate); break;
                        case AT_WIFICHANNEL_QUERY: SERIAL.print(g_wifichannel); break;
                        case AT_WIFIPOWER_QUERY: SERIAL.print(g_wifipower); break;
                        case AT_PROTOCOL_QUERY: SERIAL.print(g_protocol); break;
                        case AT_WIFIDEVICEID_QUERY: SERIAL.print(device_id); break;
                        case AT_WIFIDEVICENAME_QUERY: SERIAL.print(device_name); break;
                        case AT_BINDPHRASE_QUERY: SERIAL.print(g_bindphrase); break;
                        case AT_PSWD_QUERY: SERIAL.print(g_password); break;
                        case AT_NETSSID_QUERY: SERIAL.print(g_network_ssid); break;
                    }
                    SERIAL.write("\r\n");
                } else
                if (i == AT_RESTART) {
                    if (restart_needed) {
                        at_buf[0] = 'O';
                        at_buf[1] = 'K';
                        SERIAL.write(at_buf, at_pos);
                        SERIAL.write("\r\n");
                        delay(100);
                        restart();
                    } else {
                        SERIAL.write("KO\r\n"); // cmd recognized, but a restart was not needed
                    }
                } else
                if (i >= AT_BAUD_9600 && i <= AT_BAUD_230400) {
                    at_buf[0] = 'O';
                    at_buf[1] = 'K';
                    int new_baudrate = atoi(at_buf + 8);
                    if (new_baudrate != g_baudrate) {
                        g_baudrate = new_baudrate;
                        settings_need_save = true;
                        restart_needed = true;
                        at_buf[2] = '*';
                    }
                    SERIAL.write(at_buf, at_pos);
                    SERIAL.write("\r\n");
                } else
                if (i >= AT_WIFICHANNEL_1 && i <= AT_WIFICHANNEL_36) {
                    at_buf[0] = 'O';
                    at_buf[1] = 'K';
                    int new_wifichannel = atoi(at_buf + 15);
                    if (new_wifichannel != g_wifichannel) {
                        g_wifichannel = new_wifichannel;
                        settings_need_save = true;
                        restart_needed = true;
                        at_buf[2] = '*';
                    }
                    SERIAL.write(at_buf, at_pos);
                    SERIAL.write("\r\n");
                } else
                if (i >= AT_WIFIPOWER_0_LOW && i <= AT_WIFIPOWER_2_MAX) {
                    at_buf[0] = 'O';
                    at_buf[1] = 'K';
                    int new_wifipower = atoi(at_buf + 13);
                    if (new_wifipower != g_wifipower) {
                        g_wifipower = new_wifipower;
                        settings_need_save = true;
                        restart_needed = true;
                        at_buf[2] = '*';
                    }
                    SERIAL.write(at_buf, at_pos);
                    SERIAL.write("\r\n");
                } else
                if (i >= AT_PROTOCOL_0_TCP && i <= AT_PROTOCOL_5_BLE) {
                    at_buf[0] = 'O';
                    at_buf[1] = 'K';
                    int new_protocol = atoi(at_buf + 12);
                    if (new_protocol != g_protocol) {
                        g_protocol = new_protocol;
                        settings_need_save = true;
                        restart_needed = true;
                        at_buf[2] = '*';
                    }
                    SERIAL.write(at_buf, at_pos);
                    SERIAL.write("\r\n");
                } else
                if (i == AT_BINDPHRASE_XXXXXX) {
                    at_buf[0] = 'O';
                    at_buf[1] = 'K';
                    String new_bindphrase = "";
                    for (int pp = 14; pp < 14+6; pp++) new_bindphrase += at_buf[pp];
                    if (new_bindphrase != g_bindphrase) {
                        g_bindphrase = new_bindphrase;
                        settings_need_save = true;
                        restart_needed = true;
                        at_buf[2] = '*';
                    }
                    SERIAL.write(at_buf, at_pos);
                    SERIAL.write("\r\n");
                } else
                if (i == AT_PSWD_XXXXXX) {
                    at_buf[0] = 'O';
                    at_buf[1] = 'K';
                    String new_password = "";
                    for (int pp = 8; pp < 8+24; pp++) if (at_buf[pp] < 255) new_password += at_buf[pp];
                    if (new_password != g_password) {
                        g_password = new_password;
                        settings_need_save = true;
                        restart_needed = true;
                        at_buf[2] = '*';
                    }
                    SERIAL.write(at_buf, at_pos);
                    SERIAL.write("\r\n");
                } else
                if (i == AT_NETSSID_XXXXXX) {
                    at_buf[0] = 'O';
                    at_buf[1] = 'K';
                    String new_network_ssid = "";
                    for (int pp = 11; pp < 11+24; pp++) if (at_buf[pp] < 255) new_network_ssid += at_buf[pp];
                    if (new_network_ssid != g_network_ssid) {
                        g_network_ssid = new_network_ssid;
                        settings_need_save = true;
                        restart_needed = true;
                        at_buf[2] = '*';
                    }
                    SERIAL.write(at_buf, at_pos);
                    SERIAL.write("\r\n");
                }
                at_pos = 0;
                break;
            }
        }
        if (!possible_match) { // no possible match found, so reset
            at_pos = 0;
        }

    } // avail

    return true;
}


void AtMode::restart(void)
{
    // save settings to flash before restart
    if (settings_need_save) {
        tSettings s;
        s.magic = SETTINGS_MAGIC;
        s.protocol = g_protocol;
        s.baudrate = g_baudrate;
        s.wifichannel = g_wifichannel;
        s.wifipower = g_wifipower;
        memset(s.bindphrase, 0, sizeof(s.bindphrase));
        g_bindphrase.toCharArray(s.bindphrase, sizeof(s.bindphrase));
        memset(s.password, 0, sizeof(s.password));
        g_password.toCharArray(s.password, sizeof(s.password));
        memset(s.network_ssid, 0, sizeof(s.network_ssid));
        g_network_ssid.toCharArray(s.network_ssid, sizeof(s.network_ssid));
        settings_save(s);
        settings_need_save = false;
    }

    // use the AmebaD system reset
    sys_reset();
}
