//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP Powerup Counter
//********************************************************
#ifndef ESP_POWERUP_CNT_H
#define ESP_POWERUP_CNT_H
#pragma once

// Needed for entering bind mode with rapid power cycles


#include <inttypes.h>


typedef enum {
    POWERUPCNT_TASK_NONE = 0,
    POWERUPCNT_TASK_BIND,
} POWERUPCNT_TASK_ENUM;


extern volatile uint32_t millis32(void);


#define POWERUPCNT_BIND_CNT       4 // number of rapid power ups to enter bind mode

#define POWERUPCNT_TMO_MS         2000


// The count is kept in a raw flash sector, the first sector of the otherwise unused file system
// area. As on STM32, a word goes FF (free) -> AA (power up counted) -> 00 (cleared), so that
// counting and clearing are just word programs. A file system or nvs write can stall the receive
// loop for several frames. The sector is only erased in Init().
// ESP8266: file system area of the linker script. ESP32: spiffs partition.
// Measured times of the clear in Do(), which runs 2 s after power up while the link is active:
//   ESP8285 (generic 2400 d pa):  164 us,  was 22 - 92 ms with Preferences/LittleFS
//   ESP32 (RadioMaster XR4):      247 us,  was 2.9 ms with nvs
// For comparison, a frame is 9 ms in FLRC 111 Hz mode. With the 22 - 92 ms the receiver fell out
// of step with the sx, and was reset by the watchdog or ended up in a fail state.
// The write and the sector erase in Init() are done before the link is up, and were not measured.

#ifdef ESP8266
#include <flash_hal.h>
#else
#include <esp_partition.h>
#endif

#define POWERUPCNT_SECTOR_SIZE    4096

#define POWERUPCNT_FF             0xFFFFFFFF
#define POWERUPCNT_AA             0xAAAAAAAA


static bool powerup_counter_initialized = false;


class tPowerupCounter
{
  public:
    void Init(void);
    void Do(void);
    uint8_t Task(void);

  private:
    // flash access, ofs is relative to the start of the sector
    bool flash_init(void);
    bool flash_read(uint32_t ofs, uint32_t* const buf, uint16_t len);
    bool flash_write(uint32_t ofs, uint32_t* const buf, uint16_t len);
    bool flash_erase(void);

    void clear(void);

    bool powerup_do;
    uint8_t task;

    uint32_t run_ofs; // offset of the first counted word
    uint8_t run_len; // number of counted words

#ifndef ESP8266
    const esp_partition_t* part;
#endif
};


#ifdef ESP8266

bool tPowerupCounter::flash_init(void)
{
    return (FS_PHYS_SIZE >= POWERUPCNT_SECTOR_SIZE); // without file system area we would hit the eeprom
}

bool tPowerupCounter::flash_read(uint32_t ofs, uint32_t* const buf, uint16_t len)
{
    return ESP.flashRead(FS_PHYS_ADDR + ofs, buf, len);
}

bool tPowerupCounter::flash_write(uint32_t ofs, uint32_t* const buf, uint16_t len)
{
    noInterrupts(); // holds, ESP.flashWrite() does not yield
    bool res = ESP.flashWrite(FS_PHYS_ADDR + ofs, buf, len);
    interrupts();
    return res;
}

bool tPowerupCounter::flash_erase(void)
{
    return ESP.flashEraseSector(FS_PHYS_ADDR / POWERUPCNT_SECTOR_SIZE);
}

#else

bool tPowerupCounter::flash_init(void)
{
    part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, NULL);
    return (part != NULL && part->size >= POWERUPCNT_SECTOR_SIZE);
}

bool tPowerupCounter::flash_read(uint32_t ofs, uint32_t* const buf, uint16_t len)
{
    return (esp_partition_read(part, ofs, buf, len) == ESP_OK);
}

bool tPowerupCounter::flash_write(uint32_t ofs, uint32_t* const buf, uint16_t len)
{
    return (esp_partition_write(part, ofs, buf, len) == ESP_OK);
}

bool tPowerupCounter::flash_erase(void)
{
    return (esp_partition_erase_range(part, 0, POWERUPCNT_SECTOR_SIZE) == ESP_OK);
}

#endif


void tPowerupCounter::clear(void)
{
    uint32_t zeros[POWERUPCNT_BIND_CNT] = {};

    flash_write(run_ofs, zeros, run_len * sizeof(uint32_t));
}


void tPowerupCounter::Init(void)
{
    // a soft restart is not a power up, and must not disturb a count which is in progress
    if (powerup_counter_initialized) return;
    powerup_counter_initialized = true;

    powerup_do = false;
    task = POWERUPCNT_TASK_NONE;

    if (!flash_init()) return;

    // search for the first free word, and count the power ups counted directly before it
    uint32_t buf[16];
    uint32_t free_ofs = POWERUPCNT_SECTOR_SIZE; // none found yet
    uint8_t cnt = 0;
    bool valid = true;
    for (uint32_t ofs = 0; ofs < POWERUPCNT_SECTOR_SIZE; ofs += sizeof(buf)) {
        if (!flash_read(ofs, buf, sizeof(buf))) return;
        for (uint8_t n = 0; n < sizeof(buf)/sizeof(buf[0]); n++) {
            if (free_ofs < POWERUPCNT_SECTOR_SIZE) {
                if (buf[n] != POWERUPCNT_FF) valid = false;
            } else
            if (buf[n] == POWERUPCNT_FF) {
                free_ofs = ofs + n * sizeof(uint32_t);
            } else
            if (buf[n] == POWERUPCNT_AA) {
                if (cnt < UINT8_MAX) cnt++;
            } else
            if (buf[n] == 0) {
                cnt = 0;
            } else {
                valid = false;
            }
        }
    }

    // sector is full or holds something else, so erase. A count in progress is lost, which is ok.
    if (!valid || free_ofs >= POWERUPCNT_SECTOR_SIZE || cnt >= POWERUPCNT_BIND_CNT) {
        if (!flash_erase()) return;
        free_ofs = 0;
        cnt = 0;
    }

    run_ofs = free_ofs - cnt * sizeof(uint32_t);
    run_len = cnt;

    // the count is advanced on every boot, a reset counts like a power up.
    // the timeout in Do() is what keeps a reset from leaving a stale count behind.
    cnt++;

    if (cnt >= POWERUPCNT_BIND_CNT) {
        task = POWERUPCNT_TASK_BIND;
        clear();
        return;
    }

    uint32_t val = POWERUPCNT_AA;
    flash_write(free_ofs, &val, sizeof(val));
    run_len = cnt;
    powerup_do = true; // count is cleared again if we stay powered for long enough
}


void tPowerupCounter::Do(void)
{
    if (!powerup_do) return;

    if (millis32() < POWERUPCNT_TMO_MS) return;

    powerup_do = false;

    clear();
}


uint8_t tPowerupCounter::Task(void)
{
    switch (task) {
    case POWERUPCNT_TASK_BIND:
        task = POWERUPCNT_TASK_NONE;
        return POWERUPCNT_TASK_BIND;
    }

    return POWERUPCNT_TASK_NONE;
}


#endif // ESP_POWERUP_CNT
