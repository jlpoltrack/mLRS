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
-- The image files are in /FIRMWARE, see ota_loader.h for their format. The tx module does the update,
-- this script only feeds it with the data, see ota_relay_tx.h.
----------------------------------------------------------------------

local FW_DIR = "/FIRMWARE"
local FW_EXT = "%.ota$"

local HEADER_LEN = 40
local ENVELOPE_OTA = 0x67 -- 0xEE, len, 0x81, 0x67, offset[3], n, data[n], crc8
local OFFSET_START = 0xFFFFFF -- data is the header
local CHUNK_LEN = 52 -- a frame can hold 55
local STATUS_CMD = 0xA0 + 20 -- MBRIDGE_CMD_RX_OTA_STATUS
local STATE_STARTING = 1
local STATE_TRANSFER = 2
local STATE_DONE = 3

-- the script is called every 50 ms, and one frame can be on its way to the tx module
-- so the script stays for some 10 ms ticks, and sends more frames, must stay well below the 50 ms it is given
-- 20 ms was the best on the bench (Bandit and TX15 internal, 1.87 MBaud, 500 Hz), longer was not faster
local BURST_TICKS = 2

-- go back if the tx module has not taken anything for this many 10 ms ticks though data was sent
-- it tells when a frame got lost, but can't when the one which was sent again gets lost too
local STALL_TICKS = 10

local RESULT_STR = { "ok", "no receiver", "image is for\nanother receiver", "image damaged", "transfer failed",
    "rejected by receiver", "radio too slow" }

local PAGE_PICK, PAGE_RUN, PAGE_DONE = 0, 1, 2
local page = PAGE_PICK
local files = {}
local cursor = 1

local file = nil
local image = {} -- name, target_id, version, data_length
local header = {}
local sent = 0 -- what was read from the file and sent
local status = nil -- state, result, nack_seq, rx_offset, room, t
local nack_seq = 0
local stall_offset, stall_t = 0, 0
local start_t, start_last_t, transfer_t = 0, 0, 0
local frames = 0
local gobacks = 0 -- times the script went back in the file
local done_stats = ""
local done_text = ""
local done_rate = 0


local function u32(s, pos)
    local b1, b2, b3, b4 = string.byte(s, pos, pos + 3)
    return b1 + b2 * 256 + b3 * 65536 + b4 * 16777216
end

local function versionStr(v)
    return string.format("v%d.%d.%02d", math.floor(v / 10000), math.floor(v / 100) % 100, v % 100)
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


local function statsStr()
    if status == nil then return "" end
    return string.format("back %d nack %d crc %d rej %d", gobacks, status.nack_seq, status.crc_errors, status.rejected)
end


local function finish(text)
    if file ~= nil then io.close(file); file = nil end
    done_text = text
    done_rate = 0
    done_stats = ""
    if status ~= nil then
        done_stats = string.format("%d of %d", status.rx_offset, image.data_length or 0)
    end
    if transfer_t > 0 and status ~= nil then
        local dt = getTime() - transfer_t
        if dt > 0 then done_rate = math.floor(status.rx_offset * 100 / dt) end
    end
    transfer_t = 0
    page = PAGE_DONE
end


local function openImage(name)
    file = io.open(FW_DIR.."/"..name, "r")
    if file == nil then return false end
    local s = io.read(file, HEADER_LEN)
    if s == nil or #s ~= HEADER_LEN or u32(s, 1) ~= 0x53544F4D then
        io.close(file); file = nil
        return false
    end
    header = { string.byte(s, 1, HEADER_LEN) }
    image = { name = name, target_id = u32(s, 5), version = u32(s, 9), data_length = u32(s, 17) }
    return true
end


local function pushStart()
    local t = { ENVELOPE_OTA, 0xFF, 0xFF, 0xFF, HEADER_LEN }
    for i = 1, HEADER_LEN do t[5 + i] = header[i] end
    return crossfireTelemetryPush(0x81, t)
end


local function goBack(offset)
    if offset ~= sent then gobacks = gobacks + 1 end
    sent = offset
    io.seek(file, HEADER_LEN + sent)
end


local function pushData()
    local n = image.data_length - sent
    if n > CHUNK_LEN then n = CHUNK_LEN end
    if n > status.rx_offset + status.room - sent then n = status.rx_offset + status.room - sent end
    if n <= 0 then return end
    local s = io.read(file, n)
    if s == nil or #s ~= n then goBack(sent); return end
    local t = { ENVELOPE_OTA, sent % 256, math.floor(sent / 256) % 256, math.floor(sent / 65536), n, string.byte(s, 1, n) }
    if crossfireTelemetryPush(0x81, t) then
        sent = sent + n
        frames = frames + 1
    else
        goBack(sent)
    end
end


local function popStatus()
    while true do
        local cmd, data = crossfireTelemetryPop()
        if cmd == nil then return end
        -- 0xEA, len, 0x82, 0xA0 + cmd, payload, crc8
        if cmd == 0x82 and data ~= nil and data[1] == STATUS_CMD and data[11] ~= nil then
            status = {
                state = data[2], result = data[3], nack_seq = data[4],
                rx_offset = data[6] + data[7] * 256 + data[8] * 65536 + data[9] * 16777216,
                room = data[10] + data[11] * 256,
                crc_errors = data[5], rejected = (data[12] or 0) + (data[13] or 0) * 256,
                t = getTime(),
            }
        end
    end
end


local function step()
    local t = getTime()
    popStatus()

    if status == nil then
        -- the tx module answers only when it has started, that takes more than a second
        if t - start_t > 1000 then finish("no answer from\ntx module"); return end
        if t - start_last_t >= 50 and crossfireTelemetryPush() then
            start_last_t = t
            pushStart()
        end
        return
    end

    if status.state == STATE_DONE then
        finish(RESULT_STR[status.result] or ("failed ("..tostring(status.result)..")"))
        return
    end
    if t - status.t > 300 then finish("tx module\nis silent"); return end
    if status.state ~= STATE_TRANSFER then return end

    if transfer_t == 0 then
        transfer_t = t
        stall_offset, stall_t = status.rx_offset, t
        nack_seq = status.nack_seq
    end

    -- the tx module lost a frame, or nothing goes on anymore
    if status.nack_seq ~= nack_seq then
        nack_seq = status.nack_seq
        goBack(status.rx_offset)
        stall_t = t
    elseif status.rx_offset ~= stall_offset then
        stall_offset, stall_t = status.rx_offset, t
    elseif sent > status.rx_offset and t - stall_t >= STALL_TICKS then
        goBack(status.rx_offset)
        stall_t = t
    end

    if sent < image.data_length and crossfireTelemetryPush() then pushData() end
end


local function drawLines(lines, cursor_line)
    local color = (LCD_W >= 320)
    local dy = color and 22 or 9
    local y = color and 8 or 1
    for i = 1, #lines do
        local flags = 0
        if i == cursor_line then flags = INVERS end
        lcd.drawText(color and 10 or 1, y, lines[i], flags)
        y = y + dy
        if i == 1 then y = y + dy end -- blank line below the title
    end
end


local function doPagePick(event)
    local lines = { "mLRS Receiver Update" }
    if #files == 0 then
        lines[2] = "no image files in "..FW_DIR
    end
    for i = 1, #files do lines[1 + i] = files[i] end

    if #files == 0 then
        -- nothing to pick
    elseif event == EVT_VIRTUAL_NEXT then
        cursor = cursor + 1
        if cursor > #files then cursor = 1 end
    elseif event == EVT_VIRTUAL_PREV then
        cursor = cursor - 1
        if cursor < 1 then cursor = #files end
    elseif event == EVT_VIRTUAL_ENTER then
        if openImage(files[cursor]) then
            sent, frames, gobacks, status, transfer_t = 0, 0, 0, nil, 0
            start_t = getTime()
            start_last_t = start_t - 50
            page = PAGE_RUN
        else
            finish("not an image file")
        end
    end

    drawLines(lines, (#files == 0) and 0 or (1 + cursor))
end


local function doPageRun(event)
    if event == EVT_VIRTUAL_EXIT then finish("stopped"); return end

    local t0 = getTime()
    repeat
        step()
    until page ~= PAGE_RUN or getTime() - t0 >= BURST_TICKS
    if page ~= PAGE_RUN then return end

    local lines = { "mLRS Receiver Update", image.name,
        string.format("target %08X  %s", image.target_id, versionStr(image.version)) }
    if status == nil or status.state == STATE_STARTING then
        lines[4] = "looking for receiver..."
    else
        local dt = getTime() - transfer_t
        local rate = 0
        if dt > 0 then rate = math.floor(status.rx_offset * 100 / dt) end
        lines[4] = string.format("%d %%   %d of %d", math.floor(100 * status.rx_offset / image.data_length),
            status.rx_offset, image.data_length)
        lines[5] = string.format("%d B/s   %d frames", rate, frames)
        lines[6] = statsStr()
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
    if done_stats ~= "" then lines[#lines + 1] = done_stats end
    if done_rate > 0 then lines[#lines + 1] = string.format("%d B/s   %d frames", done_rate, frames) end
    if done_stats ~= "" then lines[#lines + 1] = statsStr() end
    drawLines(lines, 0)
end


local function scriptInit()
    scanFiles()
end


local function scriptRun(event)
    if event == nil then
        error("Cannot be run as a model script!")
        return 2
    end
    if crossfireTelemetryPush() == nil then
        error("mLRS not accessible: CRSF not enabled!")
        return 2
    end

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

return { init=scriptInit, run=scriptRun }
