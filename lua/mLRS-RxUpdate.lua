--local toolName = "TNS|mLRS Receiver Update|TNE"
----------------------------------------------------------------------
-- Copyright (c) MLRS project
-- GPL3
-- https://www.gnu.org/licenses/gpl-3.0.de.html
----------------------------------------------------------------------
-- Lua TOOLS script
-- updates the receiver over the air with an image file on the SD card
----------------------------------------------------------------------
-- Works with mLRS tx modules run in CRSF mode.
-- The image files are in /FIRMWARE, made by tools/run_rx_ota_file.py.
-- The script is the host, as tools/run_rx_ota.py is, see Common/ota/ota_protocol.h, the tx module relays, see
-- CommonTx/ota_relay_tx.h. The frames are as with the PC, STX, len, packet, crc16, and go as a byte stream in
-- crsfbridge envelopes, 0xEE/0xEA, len, 0x81/0x82, 0x66, seq, n, data[n], crc8. An answer takes 5 envelopes.
----------------------------------------------------------------------

local FW_DIR = "/FIRMWARE"
local FW_EXT = "%.ota$"

-- image file: header, raw deflate stream or the image as it is, see tools/run_rx_ota_file.py
local FILE_MAGIC = 0x32544F4D
local FILE_HEADER_LEN = 56
local FILE_FLAG_RAW = 0x01

-- must match Common/ota/ota_protocol.h and CommonTx/ota_relay_tx.h
local OTA_PROTOCOL_VERSION = 1
local OTA_CMD_INFO, OTA_CMD_DATA, OTA_CMD_RESULT = 1, 2, 3
local OTA_CMD_RESPONSE = 0x80
local OTA_STATUS_OK, OTA_STATUS_ERR_TARGET, OTA_STATUS_ERR_LENGTH, OTA_STATUS_ERR_VERSION = 0, 1, 2, 3
local OTA_STATUS_ERR_UNSUPPORTED = 6
local OTA_FLAG_DEFLATE, OTA_FLAG_GZIP = 0x01, 0x02
local OTA_STATUS_STR = { [0] = "ok", "image is for\nanother receiver", "image too large",
    "receiver has another\nprotocol version", "flash error", "image damaged", "receiver can't\ntake this image",
    "data error" }

local OTA_DATA_LEN_MAX = 248
local OTA_PACKET_HEADER_LEN = 3

local RELAY_STX = 0xA5
local ENVELOPE_CMD = 0x66
local ENVELOPE_DATA_LEN_MAX = 54 -- 61-byte frames, as the old script
local MBRIDGE_CMD_RX_OTA = 0xA0 + 19 -- MBRIDGE_COMMANDPACKET_STX + MBRIDGE_CMD_RX_OTA
local READ_AHEAD_LEN = 1024 -- is read from the file in one go
local READ_ERRORS_MAX = 5 -- the receiver tries a request 6 times, 300 ms apart, we give up before it does

-- in 10 ms ticks
local START_REPEAT_TICKS = 300 -- the start command goes again if no request came
local NO_RECEIVER_TICKS = 1200 -- the receiver waits 10 s for us after it got the command
local SILENT_TICKS = 600 -- the receiver gives up after 5 s without answers
local RESULT_LINGER_TICKS = 50 -- keep answering, the receiver repeats its result if our answer got lost
-- the script is called every 50 ms, it stays this long and answers what comes in meanwhile,
-- EdgeTx suspends it at 50 ms on B&W radios, run() then doesn't return, so no drawing and no events
local BURST_TICKS = 4

-- EdgeTx looks library functions up by name in its read-only tables on each use, so keep them in locals
local band, bxor, lshift, rshift = bit32.band, bit32.bxor, bit32.lshift, bit32.rshift
local sbyte = string.byte

local PAGE_PICK, PAGE_RUN, PAGE_DONE = 0, 1, 2
local page = PAGE_PICK
local files = {}
local cursor = 1

local file = nil
local image = {} -- name, device_name, version, data_length, image_length, ...
local head, tail = "", "" -- what goes around the deflate stream, as the receiver wants it
local length = 0 -- what the receiver gets
local cache, cache_offset = "", 0 -- what was read ahead of what the receiver gets
local keep = 0 -- the receiver has what is before keep
local out, out_pos = {}, 1 -- bytes for the tx module, out_pos is the next to go
local tx_seq = 0
local rx_state, rx_len, rx_crc, rx = 0, 0, 0, {} -- parser of what comes from the tx module
local start_t, start_last_t, transfer_t, last_t, result_t, end_t = 0, 0, 0, 0, 0, 0
local requests = 0
local read_errors = 0
local result = nil
local ending, end_text = false, ""
local done_text, done_stats = "", ""


local function u32(s, pos)
    local b1, b2, b3, b4 = sbyte(s, pos, pos + 3)
    return b1 + b2 * 256 + b3 * 65536 + b4 * 16777216
end

local function u32t(t, i)
    return t[i] + t[i + 1] * 256 + t[i + 2] * 65536 + t[i + 3] * 16777216
end

-- EdgeTx Lua has 32 bit integers and float, a crc32 doesn't survive a division
local function u32le(v)
    return string.char(band(v, 0xFF), band(rshift(v, 8), 0xFF), band(rshift(v, 16), 0xFF), rshift(v, 24))
end

local function versionStr(v)
    return string.format("v%d.%d.%02d", math.floor(v / 10000), math.floor(v / 100) % 100, v % 100)
end

-- the device name, up to the first zero
local function nameOf(s)
    local n = string.find(s, "\0", 1, true)
    return n and string.sub(s, 1, n - 1) or s
end


local function scanFiles()
    files = {}
    for name in dir(FW_DIR) do
        -- macOS puts a "._name" file next to each file it copies to the card
        if string.sub(name, 1, 1) ~= "." and string.match(string.lower(name), FW_EXT) then files[#files + 1] = name end
    end
    -- insertion sort, B&W radios have no table library
    for i = 2, #files do
        local v, j = files[i], i - 1
        while j >= 1 and files[j] > v do files[j + 1] = files[j]; j = j - 1 end
        files[j + 1] = v
    end
    if cursor > #files then cursor = 1 end
end


-- returns nil, or what is wrong
local function openImage(name)
    file = io.open(FW_DIR.."/"..name, "r")
    if file == nil then return "can't open the file" end
    local s = io.read(file, FILE_HEADER_LEN)
    if s == nil or #s ~= FILE_HEADER_LEN or u32(s, 1) ~= FILE_MAGIC then
        io.close(file); file = nil
        return "not an image file"
    end
    image = { name = name, version = u32(s, 5), data_length = u32(s, 9), image_length = u32(s, 13),
              adler32 = u32(s, 17), crc32 = u32(s, 21), zlib_crc = u32(s, 25), gzip_crc = u32(s, 29),
              device_name = nameOf(string.sub(s, 33, 52)), raw = band(u32(s, 53), FILE_FLAG_RAW) ~= 0 }
    -- a copy to the card can end short, e.g. when the radio's USB drive is pulled without eject
    local st = fstat(FW_DIR.."/"..name)
    if st == nil or st.size ~= FILE_HEADER_LEN + image.data_length then
        io.close(file); file = nil
        return "image file is\nincomplete"
    end
    return nil
end


--------------------------------------------------------------
-- relay frames: STX, len, packet[len], crc16, as a byte stream
--------------------------------------------------------------

-- fmav_crc_accumulate(), by table
local CRC16_TABLE = {}
for i = 0, 255 do
    local t = band(bxor(i, lshift(i, 4)), 0xFF)
    CRC16_TABLE[i + 1] = bxor(lshift(t, 8), lshift(t, 3), rshift(t, 4))
end

local function crc16(crc, b)
    return bxor(rshift(crc, 8), CRC16_TABLE[band(bxor(crc, b), 0xFF) + 1])
end


-- EdgeTx before #7773 (2.12.4 and older) leaves each payload byte of crossfireTelemetryPush() on the Lua stack,
-- past the 20 slots a C function gets, the unused locals reserve the room so it doesn't overwrite the heap
local function telemetryPush(cmd, data)
    local ok = crossfireTelemetryPush(cmd, data)
    local r01, r02, r03, r04, r05, r06, r07, r08, r09, r10, r11, r12, r13, r14, r15, r16, r17, r18, r19, r20, r21, r22, r23, r24, r25, r26, r27, r28, r29, r30, r31, r32, r33, r34, r35, r36, r37, r38, r39, r40, r41, r42, r43, r44, r45, r46, r47, r48, r49, r50, r51, r52, r53, r54, r55, r56, r57, r58, r59, r60, r61, r62, r63, r64
    return ok
end


-- the packet is the bytes of s
local function relaySend(s)
    local n, len = #out, #s
    local crc = crc16(0xFFFF, len)
    out[n + 1], out[n + 2] = RELAY_STX, len
    n = n + 2
    for i = 1, len do
        local c = sbyte(s, i)
        n = n + 1; out[n] = c
        crc = crc16(crc, c)
    end
    out[n + 1], out[n + 2] = band(crc, 0xFF), rshift(crc, 8)
end


-- sends the next envelope, if EdgeTx takes it
local function pushEnvelope()
    local n = math.min(#out - out_pos + 1, ENVELOPE_DATA_LEN_MAX)
    if n <= 0 or not crossfireTelemetryPush() then return end
    local t = { ENVELOPE_CMD, tx_seq, n }
    for i = 1, n do t[3 + i] = out[out_pos + i - 1] end
    if not telemetryPush(0x81, t) then return end
    tx_seq = (tx_seq + 1) % 16
    out_pos = out_pos + n
    if out_pos > #out then out, out_pos = {}, 1 end
end


-- returns true when a frame with valid crc is in rx
local function relayReceive(c)
    if rx_state == 0 then
        if c == RELAY_STX then rx_state = 1 end
    elseif rx_state == 1 then
        rx_len, rx, rx_crc = c, {}, crc16(0xFFFF, c)
        rx_state = (c > 0) and 2 or 3
    elseif rx_state == 2 then
        rx[#rx + 1] = c
        rx_crc = crc16(rx_crc, c)
        if #rx == rx_len then rx_state = 3 end
    elseif rx_state == 3 then
        rx_state = (c == band(rx_crc, 0xFF)) and 4 or 0
    else
        rx_state = 0
        return c == rshift(rx_crc, 8)
    end
    return false
end


--------------------------------------------------------------
-- the host
--------------------------------------------------------------

local function finish(text)
    if file ~= nil then io.close(file); file = nil end
    done_text = text
    done_stats = ""
    if transfer_t > 0 then
        local got = (result == OTA_STATUS_OK) and length or keep
        local dt = getTime() - transfer_t
        done_stats = string.format("%d B in %d s, %d B/s", got, math.floor(dt / 100), (dt > 0) and math.floor(got * 100 / dt) or 0)
    end
    page = PAGE_DONE
end


-- session id is filled in by the tx module
local function answer(cmd, payload)
    relaySend(string.char(cmd + OTA_CMD_RESPONSE, 0, 0)..payload)
end


-- ends the relay, the receiver gives up by itself if it isn't done
local function endRelay(text)
    relaySend("")
    ending, end_text, end_t = true, text, getTime()
end


-- reads ahead from offset on, what the receiver gets is head, the file data, tail
local function readAhead(offset)
    local hlen, dlen = #head, image.data_length
    local s = ""
    if offset < hlen then s = string.sub(head, offset + 1) end
    local pos = math.max(offset - hlen, 0) -- in the file data
    local cnt = math.min(READ_AHEAD_LEN, dlen - pos)
    if cnt > 0 then
        if file == nil then file = io.open(FW_DIR.."/"..image.name, "r") end
        if file == nil then return false end
        io.seek(file, FILE_HEADER_LEN + pos)
        local r = io.read(file, cnt)
        if r == nil or #r ~= cnt then
            -- FatFs keeps a read error in the file handle, every read after fails, the next try takes a new one
            io.close(file); file = nil
            return false
        end
        s = s..r
        pos = pos + cnt
    end
    if pos >= dlen then s = s..string.sub(tail, math.max(offset - hlen - dlen, 0) + 1) end
    cache, cache_offset = s, offset
    return true
end


-- the n bytes the receiver gets from offset on
local function streamRead(offset, n)
    if offset < cache_offset or offset + n > cache_offset + #cache then
        if not readAhead(offset) then return nil end
    end
    return string.sub(cache, offset - cache_offset + 1, offset - cache_offset + n)
end


-- sets up what the receiver gets according to what it can take, returns the status and the crc32
local function info(rx_space, rx_flags, rx_name)
    if rx_name ~= image.device_name then return OTA_STATUS_ERR_TARGET end
    if image.image_length > rx_space then return OTA_STATUS_ERR_LENGTH end
    if image.raw then -- STM32, goes as it is, both stream crcs are the crc of the image
        head, tail = "", ""
        return OTA_STATUS_OK, 0, image.zlib_crc
    end
    if band(rx_flags, OTA_FLAG_DEFLATE) ~= 0 then -- zlib stream: cmf, flg, deflate stream, adler32 big endian
        head = string.char(0x78, 0xDA)
        tail = string.reverse(u32le(image.adler32))
        return OTA_STATUS_OK, OTA_FLAG_DEFLATE, image.zlib_crc
    end
    if band(rx_flags, OTA_FLAG_GZIP) ~= 0 then -- gzip stream: header without options, deflate stream, crc32, length
        head = string.char(0x1F, 0x8B, 0x08, 0, 0, 0, 0, 0, 0x02, 0xFF)
        tail = u32le(image.crc32)..u32le(image.image_length)
        return OTA_STATUS_OK, OTA_FLAG_GZIP, image.gzip_crc
    end
    return OTA_STATUS_ERR_UNSUPPORTED
end


-- d is a packet from the receiver: cmd, session_id[2], payload
local function handle(d)
    if #d < OTA_PACKET_HEADER_LEN or ending then return end
    local t = getTime()
    local cmd = d[1]
    local p = OTA_PACKET_HEADER_LEN -- payload byte i is d[p + i]
    last_t = t

    if cmd == OTA_CMD_DATA and #d == p + 5 and length > 0 then
        -- offset[4], max_len
        local offset = u32t(d, p + 1)
        local n = math.min(d[p + 5], OTA_DATA_LEN_MAX, length - offset)
        if n <= 0 then return end
        local s = streamRead(offset, n)
        if s == nil or #s ~= n then
            -- a slow card times out in EdgeTx, no answer, so the receiver repeats and we read again
            read_errors = read_errors + 1
            if read_errors >= READ_ERRORS_MAX then endRelay("SD card read error") end
            return
        end
        read_errors = 0
        keep = offset
        requests = requests + 1
        answer(OTA_CMD_DATA, u32le(offset)..s)

    elseif cmd == OTA_CMD_INFO and #d >= p + 1 then
        -- version, space[4], flags, device_name_20[20]
        local status, flags, crc = OTA_STATUS_ERR_VERSION, 0, 0
        if d[p + 1] == OTA_PROTOCOL_VERSION and #d == p + 26 then
            local s = ""
            for i = p + 2, p + 26 do s = s..string.char(d[i]) end
            status, flags, crc = info(u32(s, 1), d[p + 6], nameOf(string.sub(s, 6, 25)))
        end
        if status ~= OTA_STATUS_OK then
            answer(OTA_CMD_INFO, string.char(status)..string.rep("\0", 13))
            result, result_t = status, t
            return
        end
        length = #head + image.data_length + #tail
        answer(OTA_CMD_INFO, string.char(status)..u32le(length)..string.char(flags)..u32le(image.image_length)..u32le(crc))
        if transfer_t == 0 then transfer_t = t end

    elseif cmd == OTA_CMD_RESULT and #d == p + 1 then
        answer(OTA_CMD_RESULT, "")
        if result == nil then result, result_t = d[p + 1], t end
    end
end


local function popAll()
    while true do
        local cmd, data = crossfireTelemetryPop()
        if cmd == nil then return end
        if cmd == 0x82 and data ~= nil and data[1] == ENVELOPE_CMD and #data >= 3 then
            for i = 4, math.min(3 + data[3], #data) do
                if relayReceive(data[i]) then handle(rx) end
            end
        end
    end
end


local function step()
    local t = getTime()
    popAll()
    pushEnvelope()

    if ending then
        if #out == 0 or t - end_t > RESULT_LINGER_TICKS then finish(end_text) end -- the tx module may be gone
        return
    end
    if result ~= nil and t - result_t > RESULT_LINGER_TICKS then
        endRelay((result == OTA_STATUS_OK) and "ok, receiver reboots" or (OTA_STATUS_STR[result] or ("error "..tostring(result))))
        return
    end
    if last_t == 0 then
        if t - start_t > NO_RECEIVER_TICKS then finish("no receiver"); return end
        if t - start_last_t >= START_REPEAT_TICKS and crossfireTelemetryPush() then
            -- 'O', 'W', len/cmd, see mLRS.lua
            if telemetryPush(0x81, { 79, 87, MBRIDGE_CMD_RX_OTA }) then start_last_t = t end
        end
    elseif t - last_t > SILENT_TICKS then
        finish("receiver went silent")
    end
end


--------------------------------------------------------------
-- pages
--------------------------------------------------------------

local function drawLines(lines, cursor_line)
    local color = (LCD_W >= 320)
    local dy = color and 22 or 9
    local y = color and 8 or 1
    for i = 1, #lines do
        lcd.drawText(color and 10 or 1, y, lines[i], (i == cursor_line) and INVERS or 0)
        y = y + dy
        if i == 1 then y = y + dy end -- blank line below the title
    end
end


local function doPagePick(event)
    local lines = { "mLRS Receiver Update" }
    if #files == 0 then lines[2] = "no image files in "..FW_DIR end
    for i = 1, #files do lines[1 + i] = files[i] end

    if #files == 0 then -- nothing to pick
    elseif event == EVT_VIRTUAL_NEXT then
        cursor = cursor % #files + 1
    elseif event == EVT_VIRTUAL_PREV then
        cursor = (cursor - 2) % #files + 1
    elseif event == EVT_VIRTUAL_ENTER then
        -- reset before openImage(), finish() would show the stats of the previous run
        out, out_pos, tx_seq, rx_state = {}, 1, 0, 0
        head, tail, length, result, ending = "", "", 0, nil, false
        cache, cache_offset, keep, requests, read_errors = "", 0, 0, 0, 0
        transfer_t, last_t = 0, 0
        local err = openImage(files[cursor])
        if err == nil then
            start_t = getTime()
            start_last_t = start_t - START_REPEAT_TICKS
            page = PAGE_RUN
        else
            finish(err)
        end
    end

    drawLines(lines, (#files == 0) and 0 or (1 + cursor))
end


local function doPageRun(event)
    if event == EVT_VIRTUAL_EXIT and not ending then endRelay("stopped") end

    local t0 = getTime()
    repeat step() until page ~= PAGE_RUN or getTime() - t0 >= BURST_TICKS
    if page ~= PAGE_RUN then return end

    local lines = { "mLRS Receiver Update", image.name,
        string.format("%s  %s", image.device_name, versionStr(image.version)) }
    if length == 0 then
        lines[4] = "looking for receiver..."
        if getRSSI() == 0 then lines[5] = "(receiver not connected)" end
    else
        local dt = getTime() - transfer_t
        lines[4] = string.format("%d %%   %d of %d", math.floor(100 * keep / length), keep, length)
        lines[5] = string.format("%d B/s   %d requests", (dt > 0) and math.floor(keep * 100 / dt) or 0, requests)
    end
    drawLines(lines, 0)
end


local function doPageDone(event)
    if event == EVT_VIRTUAL_EXIT or event == EVT_VIRTUAL_ENTER then
        scanFiles()
        page = PAGE_PICK
        return
    end
    local lines = { "mLRS Receiver Update" }
    for line in string.gmatch(done_text, "[^\n]+") do
        lines[#lines + 1] = (#lines == 1) and ("Result: "..line) or line
    end
    if done_stats ~= "" then
        lines[#lines + 1] = done_stats
        lines[#lines + 1] = string.format("%d requests", requests)
    end
    drawLines(lines, 0)
end


local function scriptRun(event)
    if event == nil then error("Cannot be run as a model script!") end
    if crossfireTelemetryPush() == nil then error("mLRS not accessible: CRSF not enabled!") end

    lcd.clear()
    if page == PAGE_PICK then
        if event == EVT_VIRTUAL_EXIT then return 2 end
        doPagePick(event)
    elseif page == PAGE_RUN then
        doPageRun(event)
    else
        doPageDone(event)
    end

    return 0
end

return { init=scanFiles, run=scriptRun }
