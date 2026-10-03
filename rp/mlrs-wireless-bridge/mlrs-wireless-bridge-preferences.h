//*******************************************************
// mLRS Wireless Bridge for RP2040/RP2350 (Pico W family)
// Copyright (c) www.olliw.eu, OlliW, OlliW42
// License: GPL v3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// Preferences
//*******************************************************
// The ESP version stores the AT mode settings with the ESP32 Preferences library.
// arduino-pico has no such library, so this is a minimal drop-in replacement with the
// same API for the handful of keys the bridge uses, backed by the EEPROM emulation
// (one flash sector, written back on each put).
//*******************************************************
#pragma once

#include <EEPROM.h>


#define PREFERENCES_MAGIC  0x424C524D // 'MRLB'

#define PREFERENCES_BINDPHRASE_LEN  16
#define PREFERENCES_STR_LEN  32


class tPreferences {
  public:
    void begin(const char* name, bool readonly = false) {
        (void)name;
        (void)readonly;
        EEPROM.begin(sizeof(tData) + 16);
        EEPROM.get(0, data);
        if (data.magic != PREFERENCES_MAGIC) set_defaults();
    }

    void end(void) {
        EEPROM.end();
    }

    int getInt(const char* key, int dflt) {
        int* p = int_ptr(key);
        return (p) ? *p : dflt;
    }

    void putInt(const char* key, int value) {
        int* p = int_ptr(key);
        if (!p || *p == value) return;
        *p = value;
        store();
    }

    String getString(const char* key, String dflt) {
        char* p = str_ptr(key);
        return (p) ? String(p) : dflt;
    }

    void putString(const char* key, String value) {
        char* p = str_ptr(key);
        if (!p) return;
        uint16_t len = str_len(key);
        char buf[PREFERENCES_STR_LEN];
        memset(buf, 0, sizeof(buf));
        strncpy(buf, value.c_str(), len - 1);
        if (!memcmp(p, buf, len)) return;
        memcpy(p, buf, len);
        store();
    }

  private:
    struct tData {
        uint32_t magic;
        int32_t protocol;
        int32_t baudrate;
        int32_t wifichannel;
        int32_t wifipower;
        char bindphrase[PREFERENCES_BINDPHRASE_LEN];
        char password[PREFERENCES_STR_LEN];
        char network_ssid[PREFERENCES_STR_LEN];
    };

    tData data;

    void set_defaults(void) {
        memset(&data, 0, sizeof(data));
        data.magic = PREFERENCES_MAGIC;
        // the "not available" values which the sketch checks for, so it fills in its defaults
        data.protocol = 255;
        data.baudrate = 0;
        data.wifichannel = 0;
        data.wifipower = 255;
        strcpy(data.bindphrase, "mlrs.0");
        store();
    }

    void store(void) {
        EEPROM.put(0, data);
        EEPROM.commit();
    }

    int* int_ptr(const char* key) {
        if (!strcmp(key, G_PROTOCOL_STR)) return (int*)&data.protocol;
        if (!strcmp(key, G_BAUDRATE_STR)) return (int*)&data.baudrate;
        if (!strcmp(key, G_WIFICHANNEL_STR)) return (int*)&data.wifichannel;
        if (!strcmp(key, G_WIFIPOWER_STR)) return (int*)&data.wifipower;
        return nullptr;
    }

    char* str_ptr(const char* key) {
        if (!strcmp(key, G_BINDPHRASE_STR)) return data.bindphrase;
        if (!strcmp(key, G_PASSWORD_STR)) return data.password;
        if (!strcmp(key, G_NETWORK_SSID_STR)) return data.network_ssid;
        return nullptr;
    }

    uint16_t str_len(const char* key) {
        return (!strcmp(key, G_BINDPHRASE_STR)) ? PREFERENCES_BINDPHRASE_LEN : PREFERENCES_STR_LEN;
    }
};
