//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP WiFi Update
//*******************************************************
// Rx opens WiFi AP "mLRS <device name>", upload firmware at http://10.0.0.1/
#ifndef ESP_WIFI_UPDATE_H
#define ESP_WIFI_UPDATE_H
#pragma once

#ifdef ESP8266
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Updater.h>
typedef ESP8266WebServer WebServer;
#define WIFI_UPDATE_MAX_SIZE  ((ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000)
#else
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#define WIFI_UPDATE_MAX_SIZE  UPDATE_SIZE_UNKNOWN
#endif


#define WIFI_UPDATE_AFTER_POWERUP_MS  60000


// styled like the mLRS Web Flasher, self-contained since the AP has no internet
static const char wifi_update_page[] PROGMEM = R"HTML(<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width,initial-scale=1"><title>mLRS WiFi Update</title><style>
body{background:#0f172a;color:#cbd5e1;font-family:sans-serif;margin:0;padding:24px 16px}
.c{background:#1e293b;border:1px solid #0284c7;border-radius:8px;padding:24px;max-width:28rem;margin:auto}
.h{display:flex;align-items:center;gap:20px}.h svg{flex:none}
h1{color:#e2e8f0;font-size:1.6rem;margin:0 0 4px}.h p{margin:0}
input,button{display:block;width:100%;box-sizing:border-box;margin-top:16px}
input{background:#334155;border:1px solid #0284c7;border-radius:8px;padding:8px;color:#cbd5e1}
button{background:#0284c7;color:#fff;border:0;border-radius:8px;padding:10px;font-size:1rem;font-weight:bold}
button:disabled{opacity:.5}progress{width:100%;margin-top:16px;accent-color:#0284c7}
</style></head><body><div class="c">
<div class="h"><svg viewBox="10 9 110 47" width="160"><g fill="none" stroke-width="1.5"><g stroke="#a5e8fb">
<circle cx="14.3" cy="21" r="2.8"/><circle cx="25.7" cy="14" r="2.8"/><circle cx="60" cy="14" r="2.8"/>
<circle cx="73" cy="20.3" r="2.8"/><circle cx="54" cy="29.5" r="2.8"/>
<path d="M16.3 23L21 28V35M25.7 16.8V35M57.2 14H39L30 23V35M70.2 20.3L43 21L34.5 29V35M52 27L50.5 25.5H46L39 31.5V35"/></g>
<g stroke-width="4"><path stroke="#18c8fa" d="M92.5 12A24.5 24.5 0 0 1 117 36.5M92.5 19A17.5 17.5 0 0 1 110 36.5"/>
<path stroke="#ff4500" stroke-width="3.5" d="M92.5 25.3A11.2 11.2 0 0 1 103.7 36.5"/></g></g>
<text x="14" y="54" font-size="33" font-weight="900" font-family="sans-serif" fill="#fff" textLength="85"
 lengthAdjust="spacingAndGlyphs"><tspan fill="#18c8fa">m</tspan>LRS</text></svg>
<div><h1>WiFi Update</h1><p>)HTML" DEVICE_NAME R"HTML(</p></div></div>
<form id="f" method="POST" enctype="multipart/form-data"><input type="file" accept=".bin" name="firmware" required>
<button id="b">Flash</button></form><progress id="p" value="0" hidden></progress><p id="s"></p></div><script>
f.onsubmit=e=>{e.preventDefault();let x=new XMLHttpRequest();x.open('POST','/');
x.upload.onprogress=e=>p.value=e.loaded/e.total;x.onload=()=>s.textContent=x.responseText;
x.onerror=()=>s.textContent='Upload failed.';p.hidden=0;b.disabled=1;x.send(new FormData(f))}
</script></body></html>)HTML";


// call in systask while not connected, never returns once started, restarts the ESP after flashing
void wifi_update_tick(void)
{
    if (millis32() < WIFI_UPDATE_AFTER_POWERUP_MS) return;

    gpio_low(SX_RESET); // hold radios in reset
#ifdef SX2_RESET
    gpio_low(SX2_RESET);
#endif

    WebServer server(80);

    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(IPAddress(10,0,0,1), IPAddress(10,0,0,1), IPAddress(255,255,255,0));
    WiFi.softAP("mLRS " DEVICE_NAME);

    server.on("/", HTTP_GET, [&]() {
        server.send_P(200, PSTR("text/html"), wifi_update_page);
    });
    server.on("/", HTTP_POST, [&]() {
        server.send_P(200, PSTR("text/plain"), Update.hasError() ? PSTR("Update failed. Rebooting.") : PSTR("Update OK. Rebooting."));
        delay(500);
        ESP.restart();
    }, [&]() {
        HTTPUpload& upload = server.upload();
        if (upload.status == UPLOAD_FILE_START) {
            Update.begin(WIFI_UPDATE_MAX_SIZE);
        } else if (upload.status == UPLOAD_FILE_WRITE) {
            Update.write(upload.buf, upload.currentSize);
        } else if (upload.status == UPLOAD_FILE_END) {
            Update.end(true);
        }
    });
    server.begin();

    uint32_t tlast_ms = millis32();
    uint8_t blink_step = 0;
    while (1) {
        server.handleClient();
        if ((millis32() - tlast_ms) >= 100) { // double blink: on/off 100 ms twice, then off 600 ms
            tlast_ms += 100;
            if (blink_step == 0 || blink_step == 2) led_red_on(); else led_red_off();
            if (++blink_step >= 9) blink_step = 0;
        }
        delay(1);
    }
}


#endif // ESP_WIFI_UPDATE_H
