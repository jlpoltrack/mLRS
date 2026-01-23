--local toolName = "TNS|mLRS Configurator|TNE"
----------------------------------------------------------------------
-- Copyright (c) MLRS project
-- GPL3
-- https://www.gnu.org/licenses/gpl-3.0.de.html
-- OlliW @ www.olliw.eu
----------------------------------------------------------------------
-- Lua TOOLS script
----------------------------------------------------------------------
-- copy script to SCRIPTS\TOOLS folder on OpenTx SD card
-- works with mLRS v1.3.03 and later, mOTX v33

local VERSION = {
    SCRIPT = '2026-01-23.01',
    REQUIRED_TX = 10303,  -- 'v1.3.03'
    REQUIRED_RX = 10303,  -- 'v1.3.03'
}

-- timing defaults
local DEFAULTS = {
    SAVE_DELAY = 300,           -- 3s delay after save before param requests (in 10ms units)
    POPUP_TIMEOUT_LONG = 300,   -- 3s popup for critical errors (in 10ms units)
}


----------------------------------------------------------------------
-- Screen
----------------------------------------------------------------------
-- TX16, T16, etc.:    480 x 272
-- T15, TX15:          480 x 320
-- PA01:               320 x 240
-- TX16S MK3:          800 x 480

local THEME = {
    screenSize = nil,
    textColor = nil,
    titleBgColor = nil,
    menuTitleColor = nil,
    textDisableColor = nil,
}

-- consolidated layout constants (saves 21 locals)
local LAYOUT = {
    PAGE_N1 = 9,             -- number of options in left column
    PAGE_N = 18,             -- total options on page (2 * PAGE_N1)
    W_HALF = LCD_W / 2,
    -- text sizing
    TEXT_SIZE = 0,           -- font size flag (0=default, MIDSIZE for 800x480)
    TEXT_DY = 21,            -- row height for parameter lists
    TEXT_VAL_X = 140,        -- x offset for parameter values
    -- position constants
    HEADER_Y = 35,           -- y position for Tx/Rx device info
    EDIT_START_Y = 60,       -- y start for edit/tools pages
    MAIN_START_Y = 95,       -- y start for main page params
    LINE_HEIGHT = 21,        -- vertical spacing between rows
    HEADER_LINE_SMALL = 16,  -- spacing for SMLSIZE text in header
    MENU_GAP = 4,            -- extra spacing between menu items
    EXCEPT_STR_OFFSET = 70,  -- x offset for exception string
    CHAR_SCALE = 1.0,        -- multiplier for charSize on larger fonts
    -- popup box
    POPUP_X = 80,
    POPUP_Y = 76,
    POPUP_W = 320,
    POPUP_H = 80,
    -- warning box
    WARN_X = 30,
    WARN_W = 420,
    WARN_H = 50,
    -- main page buttons
    BUTTONS_Y = 171,
    EDIT_TX_X = 10,
    EDIT_RX_X = 90,
    SAVE_X = 170,
    RELOAD_X = 235,
    BIND_X = 315,
    TOOLS_X = 375,
    -- info section
    INFO_Y = 210,
    INFO_DY = 20,
    INFO_LEFT_X = 10,
    INFO_LEFT_VAL_X = 140,
    INFO_RIGHT_X = 10 + LCD_W / 2,
    INFO_RIGHT_VAL_X = 140 + LCD_W / 2,
}

local function setupScreen()
    THEME.screenSize = LCD_W * 1000 + LCD_H
    if THEME.screenSize == 320240 then -- 320x240, PA01
        LAYOUT.PAGE_N1 = 7
        LAYOUT.PAGE_N = 7 -- single column
        LAYOUT.POPUP_X = 10
        LAYOUT.POPUP_W = 300
        LAYOUT.WARN_X = 10
        LAYOUT.WARN_W = 300
        LAYOUT.BUTTONS_Y = 158
        LAYOUT.EDIT_TX_X = 2
        LAYOUT.EDIT_RX_X = 66
        LAYOUT.SAVE_X = 130
        LAYOUT.RELOAD_X = 175
        LAYOUT.BIND_X = 234
        LAYOUT.TOOLS_X = 277
        LAYOUT.INFO_Y = 180
        LAYOUT.INFO_DY = 15
        LAYOUT.INFO_LEFT_X = 10
        LAYOUT.INFO_LEFT_VAL_X = 90
        LAYOUT.INFO_RIGHT_X = 165
        LAYOUT.INFO_RIGHT_VAL_X = 245
    elseif THEME.screenSize == 800480 then -- 800x480
        LAYOUT.PAGE_N1 = 15
        LAYOUT.PAGE_N = 2 * LAYOUT.PAGE_N1
        -- text sizing for larger screen
        LAYOUT.TEXT_SIZE = MIDSIZE
        LAYOUT.TEXT_DY = 28
        LAYOUT.TEXT_VAL_X = 170
        -- position overrides for larger screen
        LAYOUT.HEADER_Y = 50
        LAYOUT.EDIT_START_Y = 80
        LAYOUT.MAIN_START_Y = 130
        LAYOUT.LINE_HEIGHT = 28
        LAYOUT.HEADER_LINE_SMALL = 22
        LAYOUT.MENU_GAP = 6
        LAYOUT.EXCEPT_STR_OFFSET = 100
        LAYOUT.CHAR_SCALE = 1.4
        -- popup/warning boxes
        LAYOUT.POPUP_X = 200
        LAYOUT.POPUP_W = 400
        LAYOUT.POPUP_Y = 140
        LAYOUT.POPUP_H = 100
        LAYOUT.WARN_X = 100
        LAYOUT.WARN_W = 600
        LAYOUT.WARN_H = 60
        LAYOUT.BUTTONS_Y = 300
        LAYOUT.EDIT_TX_X = 20
        LAYOUT.EDIT_RX_X = 20 + 110
        LAYOUT.SAVE_X = 20 + 220
        LAYOUT.RELOAD_X = 20 + 320
        LAYOUT.BIND_X = 20 + 440
        LAYOUT.TOOLS_X = 20 + 540
        LAYOUT.INFO_Y = 360
        LAYOUT.INFO_DY = 25
        LAYOUT.INFO_LEFT_X = 20
        LAYOUT.INFO_LEFT_VAL_X = 180
        LAYOUT.INFO_RIGHT_X = 20 + LAYOUT.W_HALF
        LAYOUT.INFO_RIGHT_VAL_X = 180 + LAYOUT.W_HALF
    elseif THEME.screenSize == 480320 then -- 480x320, T15
        LAYOUT.PAGE_N1 = 11
        LAYOUT.PAGE_N = 2 * LAYOUT.PAGE_N1
    else
        LAYOUT.PAGE_N1 = 9
        LAYOUT.PAGE_N = 2 * LAYOUT.PAGE_N1
    end
end

local function setupColors()
    local ver, radio, maj, minor, rev, osname = getVersion()

    if maj == nil then maj = 0 end
    if minor == nil then minor = 0 end

    if (osname == 'EdgeTX') and (maj > 2 or (maj == 2 and minor >= 4)) then
        THEME.textColor = COLOR_THEME_SECONDARY1
        THEME.titleBgColor = COLOR_THEME_SECONDARY1
        THEME.menuTitleColor = COLOR_THEME_PRIMARY2
        THEME.textDisableColor = COLOR_THEME_DISABLED
    else
        THEME.textColor = TEXT_COLOR
        THEME.titleBgColor = TITLE_BGCOLOR
        THEME.menuTitleColor = MENU_TITLE_COLOR
        THEME.textDisableColor = TEXT_DISABLE_COLOR
    end
end


----------------------------------------------------------------------
-- Drawing helpers
----------------------------------------------------------------------

local function drawFilledTriangle(x0, y0, x1, y1, x2, y2, flags)
    if lcd.drawFilledTriangle == nil then return end
    lcd.drawFilledTriangle(x0, y0, x1, y1, x2, y2, flags)
end

local charSize = {}
charSize["a"] = 10
charSize["b"] = 10
charSize["c"] = 9
charSize["d"] = 10
charSize["e"] = 9
charSize["f"] = 7
charSize["g"] = 10
charSize["h"] = 10
charSize["i"] = 5
charSize["j"] = 6
charSize["k"] = 10
charSize["l"] = 5
charSize["m"] = 15
charSize["n"] = 10
charSize["o"] = 10
charSize["p"] = 10
charSize["q"] = 10
charSize["r"] = 7
charSize["s"] = 9
charSize["t"] = 6
charSize["u"] = 10
charSize["v"] = 9
charSize["w"] = 13
charSize["x"] = 9
charSize["y"] = 9
charSize["z"] = 9
charSize["0"] = 10
charSize["1"] = 10
charSize["2"] = 10
charSize["3"] = 10
charSize["4"] = 10
charSize["5"] = 10
charSize["6"] = 10
charSize["7"] = 10
charSize["8"] = 10
charSize["9"] = 10
charSize["_"] = 9
charSize["#"] = 11
charSize["-"] = 6
charSize["."] = 5

local function getCharWidth(c)
    local base = charSize[c] or 10
    return math.floor(base * LAYOUT.CHAR_SCALE)
end


----------------------------------------------------------------------
-- MBridge CRSF emulation
----------------------------------------------------------------------

local isConnected = nil
local cmdPush = nil
local cmdPop = nil

-- consolidated MBRIDGE constants (saves 28 locals)
local MB = {
    STX  = 0xA0,
    MASK = 0xE0,
    -- command lengths
    LEN_TX_LINK_STATS = 22,
    LEN_DEVICE_ITEM   = 24,
    LEN_PARAM_ITEM    = 24,
    LEN_REQUEST_CMD   = 18,
    LEN_INFO          = 24,
    LEN_PARAM_SET     = 7,
    LEN_MODELID_SET   = 3,
    -- param types
    TYPE_UINT8  = 0,
    TYPE_INT8   = 1,
    TYPE_UINT16 = 2,
    TYPE_INT16  = 3,
    TYPE_LIST   = 4,
    TYPE_STR6   = 5,
    -- commands
    CMD_TX_LINK_STATS      = 2,
    CMD_REQUEST_INFO       = 3,
    CMD_DEVICE_ITEM_TX     = 4,
    CMD_DEVICE_ITEM_RX     = 5,
    CMD_PARAM_REQUEST_LIST = 6,
    CMD_PARAM_ITEM         = 7,
    CMD_PARAM_ITEM2        = 8,
    CMD_PARAM_ITEM3        = 9,
    CMD_REQUEST_CMD        = 10,
    CMD_INFO               = 11,
    CMD_PARAM_SET          = 12,
    CMD_PARAM_STORE        = 13,
    CMD_BIND_START         = 14,
    CMD_BIND_STOP          = 15,
    CMD_MODELID_SET        = 16,
    CMD_SYSTEM_BOOTLOADER  = 17,
    CMD_FLASH_ESP          = 18,
}

local CMD_LEN = {
    [MB.CMD_TX_LINK_STATS] = MB.LEN_TX_LINK_STATS,
    [MB.CMD_DEVICE_ITEM_TX] = MB.LEN_DEVICE_ITEM,
    [MB.CMD_DEVICE_ITEM_RX] = MB.LEN_DEVICE_ITEM,
    [MB.CMD_PARAM_ITEM]     = MB.LEN_PARAM_ITEM,
    [MB.CMD_PARAM_ITEM2]    = MB.LEN_PARAM_ITEM,
    [MB.CMD_PARAM_ITEM3]    = MB.LEN_PARAM_ITEM,
    [MB.CMD_REQUEST_CMD]    = MB.LEN_REQUEST_CMD,
    [MB.CMD_INFO]           = MB.LEN_INFO,
    [MB.CMD_PARAM_SET]      = MB.LEN_PARAM_SET,
    [MB.CMD_MODELID_SET]    = MB.LEN_MODELID_SET,
}

local function mbridgeCmdLen(cmd)
    return CMD_LEN[cmd] or 0
end

local function crsfIsConnected()
    if getRSSI() ~= 0 then return true end
    return false
end

local function crsfCmdPush(cmd, payload)
    -- 'O', 'W', len/cmd, payload bytes
    local data = { 79, 87, cmd + MB.STX }
    for i=1, mbridgeCmdLen(cmd) do data[#data + 1] = 0 end -- fill with zeros of correct length
    for i=1, #payload do data[3 + i] = payload[i] end -- fill in data
    -- crossfireTelemetryPush() extends it to
    -- 0xEE, len, 129, 'O', 'W', len/cmd, payload bytes, crc8
    return crossfireTelemetryPush(129, data)
end

local function crsfCmdPop()
    -- crossfireTelemetryPop() is invoked if
    -- address = RADIO_ADDRESS (0xEA) or UART_SYNC (0xC8)
    -- frame id != normal crsf telemetry sensor id
    -- 0xEE, len, 130, len/cmd, payload bytes, crc8
    local cmd, data = crossfireTelemetryPop()
    -- cmd = 130
    -- data = len/cmd, payload bytes
    if cmd == nil then return nil end
    if data == nil or data[1] == nil then return nil end -- Huston, we have a problem
    local command = data[1] - MB.STX
    local res = {
        cmd = command,
        len = mbridgeCmdLen(command),
        payload = {}
    }
    for i=2, #data do res.payload[i-2] = data[i] end
    return res
end

local function mbridgeIsConnected()
    local LStats = mbridge.getLinkStats()
    if LStats.LQ > 0 then return true end
    return false
end

local function setupBridge()
    if mbridge == nil or not mbridge.enabled() then
        isConnected = crsfIsConnected
        cmdPush = crsfCmdPush
        cmdPop = crsfCmdPop -- can return nil
    else
        isConnected = mbridgeIsConnected
        cmdPush = mbridge.cmdPush
        cmdPop = mbridge.cmdPop -- can return nil
    end
end


----------------------------------------------------------------------
-- Info/Warning box
----------------------------------------------------------------------

local POPUP = {
    active = false,
    text = "",
    t_end_10ms = 0,
}

local function setPopupWTmo(txt, tmo_10ms)
    if POPUP.t_end_10ms < 0 then return end -- blocked POPUP.active on display
    POPUP.active = true
    POPUP.text = txt
    POPUP.t_end_10ms = getTime() + tmo_10ms
end

local function setPopupBlocked(txt)
    POPUP.active = true
    POPUP.text = txt
    POPUP.t_end_10ms = -1
end

local function clearPopup()
    POPUP.active = false
    POPUP.t_end_10ms = 0
end

local function clearPopupIfBlocked()
    if POPUP.active and POPUP.t_end_10ms < 0 then clearPopup(); end
end

local function drawPopup()
    lcd.drawFilledRectangle(LAYOUT.POPUP_X-2, LAYOUT.POPUP_Y-2, LAYOUT.POPUP_W+4, LAYOUT.POPUP_H+4, THEME.textColor) --TITLE_BGCOLOR)
    lcd.drawFilledRectangle(LAYOUT.POPUP_X, LAYOUT.POPUP_Y, LAYOUT.POPUP_W, LAYOUT.POPUP_H, THEME.titleBgColor) --TEXT_BGCOLOR) --TITLE_BGCOLOR)

    local i = string.find(POPUP.text, "\n")
    local attr = THEME.menuTitleColor+MIDSIZE+CENTER
    if i == nil then
        lcd.drawText(LAYOUT.W_HALF, 99, POPUP.text, attr)
    else
        local t1 = string.sub(POPUP.text, 1,i-1)
        local t2 = string.sub(POPUP.text, i+1)
        lcd.drawText(LAYOUT.W_HALF, 85, t1, attr)
        lcd.drawText(LAYOUT.W_HALF, 85+30, t2, attr)
    end
end

local function doPopup()
    if POPUP.active then
        drawPopup()
        if POPUP.t_end_10ms > 0 then
            local t_10ms = getTime()
            if t_10ms > POPUP.t_end_10ms then clearPopup() end
        end
    end
end


----------------------------------------------------------------------
-- helper to handle connect
----------------------------------------------------------------------

local CONN = {
    active = false,
    changed = false,
    just_connected = false,
    just_disconnected = false,
}

local function doConnected()
    local is_connected = isConnected()

    CONN.changed = false
    if is_connected ~= CONN.active then CONN.changed = true end

    if CONN.changed and is_connected then CONN.just_connected = true end

    CONN.just_disconnected = false
    if CONN.changed and not is_connected then CONN.just_disconnected = true end

    CONN.active = is_connected
end

----------------------------------------------------------------------
-- variables for mBridge traffic
----------------------------------------------------------------------

local DEVICE = {
    ITEM_TX = nil,
    ITEM_RX = nil,
    INFO = nil,
    PARAM_LIST = nil,
    request_count = 0,  -- track connection attempts for timeout
}

-- consolidated param loading state
local LOAD = {
    expected_index = 0,
    current_index = -1,
    errors = 0,
    complete = false,
    is_running = true,  -- start in loading mode
    save_t_last = 0,
    request_t_last = 0,
    loop_t_last = 0,
    -- smart RX loading state
    skip_rx = false,       -- true when loading TX-only (no RX connected)
    rx_first_idx = -1,     -- index where RX params start (first "Rx " param)
    tx_only_done = false,  -- true when TX-only load finished
}

-- add retry constants to DEFAULTS
DEFAULTS.PARAM_RETRY_TIMEOUT = 20  -- 200ms timeout (~2x max response time for safety)
DEFAULTS.PARAM_RETRY_LIMIT = 10    -- individual parameter retries before "soft" restart
DEFAULTS.PARAM_RESTART_LIMIT = 1   -- full restarts before giving up (1 = try twice total)

-- debug statistics - togglable via tools menu
local DEBUG = {
    enabled = false,      -- toggle via tools page
    retryCount = 0,       -- total retries for display
    retryAttempt = 0,     -- retries this attempt (resets on full restart)
    restartCount = 0,     -- number of full restarts due to retry limit
    lastRetryIdx = -1,    -- last parameter index that triggered a retry
    consecutive = 0,      -- consecutive retries for same parameter
    duplicates = 0,       -- count of duplicate responses detected
    responseTimeMin = 999,-- minimum response time observed (10ms units)
    responseTimeMax = 0,  -- maximum response time observed (10ms units)
    responseTimeSum = 0,  -- sum for calculating average
    responseCount = 0,    -- count of responses for average calculation
    loadStartTime = 0,    -- when param loading started (10ms units)
    loadDuration = 0,     -- total load time in 10ms units
}

local function resetDebugStats()
    DEBUG.retryCount = 0
    DEBUG.retryAttempt = 0
    DEBUG.restartCount = 0
    DEBUG.lastRetryIdx = -1
    DEBUG.consecutive = 0
    DEBUG.duplicates = 0
    DEBUG.responseTimeMin = 999
    DEBUG.responseTimeMax = 0
    DEBUG.responseTimeSum = 0
    DEBUG.responseCount = 0
    DEBUG.loadStartTime = 0
    DEBUG.loadDuration = 0
    -- reset LOAD state too
    LOAD.skip_rx = false
    LOAD.rx_first_idx = -1
    LOAD.tx_only_done = false
end


local function clearParams(rxOnly)
    if rxOnly then
        -- RX-only reload: preserve TX params and device info, just reload RX params
        -- clear RX params from list while keeping TX params
        if DEVICE.PARAM_LIST ~= nil and LOAD.rx_first_idx >= 0 then
            for i = LOAD.rx_first_idx, 127 do
                DEVICE.PARAM_LIST[i] = nil
            end
        end
        LOAD.expected_index = LOAD.rx_first_idx
        LOAD.current_index = -1
        LOAD.complete = false
        LOAD.skip_rx = false  -- now we want RX params
        LOAD.tx_only_done = false
        LOAD.is_running = true
        LOAD.request_t_last = 0
        DEBUG.retryAttempt = 0
        return
    end
    -- full reload: clear everything
    DEVICE.ITEM_TX = nil
    DEVICE.ITEM_RX = nil
    DEVICE.INFO = nil
    DEVICE.request_count = 0
    DEVICE.PARAM_LIST = nil
    LOAD.expected_index = 0
    LOAD.current_index = -1
    LOAD.errors = 0
    LOAD.complete = false
    DEBUG.duplicates = 0  -- reset duplicate counter
    DEBUG.consecutive = 0  -- reset consecutive retry counter
    LOAD.skip_rx = false
    LOAD.rx_first_idx = -1
    LOAD.tx_only_done = false
    LOAD.is_running = true
    LOAD.request_t_last = 0  -- reset retry timeout
    DEBUG.retryAttempt = 0  -- reset per-attempt counter (keep retryCount for display)
end


local function paramsError(err)
    LOAD.errors = LOAD.errors + 1
end


----------------------------------------------------------------------
-- helper to convert command payloads
----------------------------------------------------------------------

local function mb_to_string(payload,pos,len)
    local t = {}
    for i = 0,len-1 do
        if payload[pos+i] == 0 then break end
        t[#t+1] = string.char(payload[pos+i])
    end
    return table.concat(t)
end

local function mb_to_u8(payload, pos)
    return payload[pos]
end

local function mb_to_i8(payload, pos)
    local v = payload[pos+0]
    if v >= 128 then v = v - 256 end
    return v
end

local function mb_to_u16(payload, pos)
    return payload[pos+0] + payload[pos+1]*256
end

local function mb_to_i16(payload, pos)
    local v = payload[pos+0] + payload[pos+1]*256
    if v >= 32768 then v = v - 65536 end
    return v
end

local function mb_to_u24(payload, pos)
    return payload[pos+0] + payload[pos+1]*256 + payload[pos+2]*256*256
end

local function mb_to_u32(payload, pos)
    return payload[pos+0] + payload[pos+1]*256 + payload[pos+2]*256*256 + payload[pos+3]*256*256*256
end

local function mb_to_value(payload, pos, typ)
    if typ == MB.TYPE_UINT8 then -- UINT8
        return mb_to_u8(payload,pos)
    elseif typ == MB.TYPE_INT8 then -- INT8
        return mb_to_i8(payload,pos)
    elseif typ == MB.TYPE_UINT16 then -- UINT16
        return mb_to_u16(payload,pos)
    elseif typ == MB.TYPE_INT16 then -- INT16
        return mb_to_i16(payload,pos)
    elseif typ == MB.TYPE_LIST then -- LIST
        return payload[pos+0]
    end
    return 0
end

local function mb_to_value_or_str6(payload, pos, typ)
    if typ == 5 then --MB.TYPE_STR6 then
        return mb_to_string(payload,pos,6)
    else
        return mb_to_value(payload,pos,typ)
    end
end

local function mb_to_options(payload, pos, len)
    local t = {}
    for i = 0,len-1 do
        if payload[pos+i] == 0 then break end
        t[#t+1] = string.char(payload[pos+i])
    end
    local str = table.concat(t) .. ","
    local opt = {};
    for s in string.gmatch(str, "([^,]+)") do
        table.insert(opt, s)
    end
    return opt
end

local function mb_to_version_int(u16)
    local major = bit32.rshift(bit32.band(u16, 0xF000), 12)
    local minor = bit32.rshift(bit32.band(u16, 0x0FC0), 6)
    local patch = bit32.band(u16, 0x003F)
    return major * 10000 + minor * 100 + patch
end

local function mb_to_version_string(u16)
    local major = bit32.rshift(bit32.band(u16, 0xF000), 12)
    local minor = bit32.rshift(bit32.band(u16, 0x0FC0), 6)
    local patch = bit32.band(u16, 0x003F)
    return string.format("v%d.%d.%02d", major, minor, patch)
end

local function mb_to_u8_bits(payload, pos, bitpos, bitmask)
    local v = payload[pos]
    v = bit32.rshift(v, bitpos)
    v = bit32.band(v, bitmask)
    return v
end

local function mb_allowed_mask_editable(allowed_mask)
    -- if none or only one option allowed -> not editable
    if allowed_mask == 0 then return false; end
    if allowed_mask == 1 then return false; end
    if allowed_mask == 2 then return false; end
    if allowed_mask == 4 then return false; end
    if allowed_mask == 8 then return false; end
    if allowed_mask == 16 then return false; end
    if allowed_mask == 32 then return false; end
    if allowed_mask == 64 then return false; end
    if allowed_mask == 128 then return false; end
    if allowed_mask == 256 then return false; end
    return true
end

local diversity_list = {} -- used for displaying on main page bottom info section
diversity_list[0] = "enabled"
diversity_list[1] = "antenna1"
diversity_list[2] = "antenna2"
diversity_list[3] = "r:en. t:ant1"
diversity_list[4] = "r:en. t:ant2"

local freq_band_list = {} -- used for displaying on main page instead of received options
freq_band_list[0] = "2.4 GHz"
freq_band_list[1] = "915 MHz FCC"
freq_band_list[2] = "868 MHz"
freq_band_list[3] = "433 MHz"
freq_band_list[4] = "70 cm HAM"
freq_band_list[5] = "866 MHz IN"

local function getExceptNoFromChar(c)
    if (c >= 'a' and c <= 'z') then return (string.byte(c) - string.byte('a')) % 5; end
    if (c >= '0' and c <= '9') then return (string.byte(c) - string.byte('0')) % 5; end
    if (c == '_') then return 1; end
    if (c == '#') then return 2; end
    if (c == '-') then return 3; end
    if (c == '.') then return 4; end
    return 0
end

local function getExceptStrFromChar(c)
    local n = getExceptNoFromChar(c)
    if (n == 1) then return "/e1"; end
    if (n == 2) then return "/e6"; end
    if (n == 3) then return "/e11"; end
    if (n == 4) then return "/e13"; end
    return "/--"
end


----------------------------------------------------------------------
-- looper to send and read command frames
----------------------------------------------------------------------
-- protocol change (2025-12-18): switched from push-based to pull-based parameter loading
--
-- state machine for parameter loading:
--   state 1: waiting for PARAM_ITEM (current_index < 0, expected_index = next param to request)
--   state 2: received PARAM_ITEM, waiting for ITEM2 (current_index >= 0)
--   state 3: received ITEM2, waiting for ITEM3 (if needed for long option lists)
--   state 4: received ITEM3, waiting for ITEM4 (if needed for very long option lists)
--   on timeout: reset to state 1 for current_index parameter (restart from PARAM_ITEM)
--   on success: advance to state 1 for expected_index (next parameter)
----------------------------------------------------------------------

----------------------------------------------------------------------
-- helper functions for retry logic
----------------------------------------------------------------------

local function resetParamRequestTimer()
    LOAD.request_t_last = getTime()
end

local function startParamRequest(index)
    cmdPush(MB.CMD_REQUEST_CMD, {MB.CMD_PARAM_ITEM, index})
    resetParamRequestTimer()
end

local function performRetry(t_10ms)
    -- determine which index to retry:
    -- if current_index >= 0, we're mid-parameter (waiting for ITEM2/3/4), restart from current
    -- if current_index < 0, we're waiting for first PARAM_ITEM, use expected_index
    local retry_index = LOAD.expected_index
    if LOAD.current_index >= 0 then
        retry_index = LOAD.current_index
        -- reset expected_index to retry this parameter from scratch
        LOAD.expected_index = LOAD.current_index
    end
    
    -- track consecutive retries for same parameter
    if retry_index == DEBUG.lastRetryIdx then
        DEBUG.consecutive = DEBUG.consecutive + 1
    else
        DEBUG.consecutive = 1  -- first retry for this parameter
    end
    
    startParamRequest(retry_index)
    DEBUG.lastRetryIdx = retry_index  -- track which param triggered retry
end

local function handleRetryLimitExceeded()
    DEBUG.restartCount = DEBUG.restartCount + 1
    if DEBUG.restartCount > DEFAULTS.PARAM_RESTART_LIMIT then
        -- too many full restarts, something is seriously wrong
        setPopupWTmo("Loading Failed:\nCheck connection", 500)
        LOAD.is_running = false
        LOAD.request_t_last = 0
        return true  -- handled, stop processing
    else
        clearParams()  -- soft restart
        return true  -- handled, stop processing
    end
end

local function checkAndHandleRetry(t_10ms)
    -- only check for retries if param loading is in progress
    if DEVICE.PARAM_LIST == nil or LOAD.complete or LOAD.request_t_last == 0 then
        return false  -- no retry needed
    end
    
    -- fixed timeout (typically retries are small, no need for adaptive backoff)
    if t_10ms - LOAD.request_t_last <= DEFAULTS.PARAM_RETRY_TIMEOUT then
        return false  -- timeout not reached
    end
    
    -- timeout reached - handle retry
    DEBUG.retryCount = DEBUG.retryCount + 1
    DEBUG.retryAttempt = DEBUG.retryAttempt + 1
    
    -- if too many retries this attempt, restart entire loading process
    if DEBUG.retryAttempt > DEFAULTS.PARAM_RETRY_LIMIT then
        return handleRetryLimitExceeded()
    end
    
    performRetry(t_10ms)
    return true  -- retry performed
end

local function doParamLoop()
    -- trigger getting device items and param items
    local t_10ms = getTime()
    local time_since_last = t_10ms - LOAD.loop_t_last
    
    local interval = 10 -- default 100ms (idle/maintenance)
    if LOAD.is_running then interval = 0 end -- 0ms (fastest) during download
    
    if time_since_last > interval then 
      LOAD.loop_t_last = t_10ms
      if t_10ms < LOAD.save_t_last + DEFAULTS.SAVE_DELAY then
          -- skip, we don't send a cmd if the last Save was recent
      elseif DEVICE.ITEM_TX == nil then
          DEVICE.request_count = DEVICE.request_count + 1
          if DEVICE.request_count > 10 then
              -- connection timeout - likely wrong baud rate
              setPopupWTmo("No module response\nMust use 400K baud rate", DEFAULTS.POPUP_TIMEOUT_LONG)
              DEVICE.request_count = 0  -- reset to retry
          end
          cmdPush(MB.CMD_REQUEST_INFO, {}) -- triggers sending DEVICE.ITEM_TX, DEVICE.ITEM_RX, INFO
          --cmdPush(MB.CMD_REQUEST_CMD, {MB.CMD_REQUEST_INFO)
          -- these should have been set when we nil-ed DEVICE.PARAM_LIST
          LOAD.expected_index = 0
          LOAD.current_index = -1
          LOAD.errors = 0
          LOAD.complete = false
      elseif DEVICE.PARAM_LIST == nil then
          if DEVICE.INFO ~= nil then -- wait for it to be populated
              DEVICE.PARAM_LIST = {}
              DEBUG.loadStartTime = getTime()  -- start load timer
              -- check if RX is available for smart loading
              if DEVICE.INFO.rx_available == 0 and LOAD.rx_first_idx < 0 then
                  -- no receiver connected - will skip RX params when we encounter them
                  LOAD.skip_rx = true
              end
              -- request first parameter by index (pull-based protocol)
              startParamRequest(LOAD.expected_index)
          end
      elseif LOAD.is_running and not LOAD.complete and LOAD.request_t_last == 0 then
          -- RX-only reload case: DEVICE.PARAM_LIST exists but we need to start fetching
          -- this happens after clearParams(true) - request timer is 0 but download is running
          -- first request device info to get updated DEVICE.ITEM_RX
          cmdPush(MB.CMD_REQUEST_INFO, {})
          DEBUG.loadStartTime = getTime()
          startParamRequest(LOAD.expected_index)
      end
    end

    -- check for timeout and handle retries if needed
    if checkAndHandleRetry(t_10ms) then
        return  -- early exit if retry handling completed (restart triggered)
    end

    -- handle received commands (limit 24 per cycle)
    for ijk = 1,24 do -- process up to 24 commands per lua cycle
        local cmd = cmdPop()
        if cmd == nil then break end
        if cmd.cmd == MB.CMD_DEVICE_ITEM_TX then
            -- MB.CMD_DEVICE_ITEM_TX
            DEVICE.ITEM_TX = cmd
            DEVICE.request_count = 0  -- reset inactivity timer - we got a response!
            DEVICE.ITEM_TX.version_u16 = mb_to_u16(cmd.payload, 0)
            DEVICE.ITEM_TX.setuplayout_u16 = mb_to_u16(cmd.payload, 2)
            DEVICE.ITEM_TX.name = mb_to_string(cmd.payload, 4, 20)
            DEVICE.ITEM_TX.version_int = mb_to_version_int(DEVICE.ITEM_TX.version_u16)
            DEVICE.ITEM_TX.version_str = mb_to_version_string(DEVICE.ITEM_TX.version_u16)
            DEVICE.ITEM_TX.setuplayout_int = mb_to_version_int(DEVICE.ITEM_TX.setuplayout_u16)
        elseif cmd.cmd == MB.CMD_DEVICE_ITEM_RX then
            -- MB.CMD_DEVICE_ITEM_RX
            DEVICE.ITEM_RX = cmd
            DEVICE.ITEM_RX.version_u16 = mb_to_u16(cmd.payload, 0)
            DEVICE.ITEM_RX.setuplayout_u16 = mb_to_u16(cmd.payload, 2)
            DEVICE.ITEM_RX.name = mb_to_string(cmd.payload, 4, 20)
            DEVICE.ITEM_RX.version_int = mb_to_version_int(DEVICE.ITEM_RX.version_u16)
            DEVICE.ITEM_RX.version_str = mb_to_version_string(DEVICE.ITEM_RX.version_u16)
            DEVICE.ITEM_RX.setuplayout_int = mb_to_version_int(DEVICE.ITEM_RX.setuplayout_u16)
        elseif cmd.cmd == MB.CMD_INFO then
            -- MB.CMD_INFO
            DEVICE.INFO = cmd
            DEVICE.INFO.receiver_sensitivity = mb_to_i16(cmd.payload, 0)
            DEVICE.INFO.has_status = mb_to_u8_bits(cmd.payload, 2, 0, 0x01)
            DEVICE.INFO.binding = mb_to_u8_bits(cmd.payload, 2, 1, 0x01)
            DEVICE.INFO.LQ_low = 0 -- mb_to_u8_bits(cmd.payload, 2, 3, 0x03)
            DEVICE.INFO.tx_power_dbm = mb_to_i8(cmd.payload, 3)
            DEVICE.INFO.rx_power_dbm = mb_to_i8(cmd.payload, 4)
            DEVICE.INFO.rx_available = mb_to_u8_bits(cmd.payload, 5, 0, 0x1)
            --DEVICE.INFO.tx_diversity = mb_to_u8_bits(cmd.payload, 5, 1, 0x3)
            --DEVICE.INFO.rx_diversity = mb_to_u8_bits(cmd.payload, 5, 3, 0x3)
            DEVICE.INFO.tx_config_id = mb_to_u8(cmd.payload, 6)
            DEVICE.INFO.tx_diversity = mb_to_u8_bits(cmd.payload, 7, 0, 0x0F)
            DEVICE.INFO.rx_diversity = mb_to_u8_bits(cmd.payload, 7, 4, 0x0F)
        elseif cmd.cmd == MB.CMD_PARAM_ITEM then
            -- MB.CMD_PARAM_ITEM
            local index = cmd.payload[0]
            -- ignore responses for indices we've already seen (from retries)
            -- only accept: the expected index, or 255 (EOL)
            if index ~= LOAD.expected_index and index ~= 255 then
                -- likely a duplicate from retry - silently ignore to avoid state corruption
                -- don't call paramsError() as this is expected behavior with retry logic
                DEBUG.duplicates = DEBUG.duplicates + 1
            else
            LOAD.current_index = index -- inform potential Item2/3 calls
            LOAD.expected_index = index + 1 -- prepare for next
            
            -- measure response time for this parameter
            if LOAD.request_t_last > 0 then  -- only measure if timer is valid
                local response_time = getTime() - LOAD.request_t_last
                if response_time > 0 and response_time < 999 then  -- sanity check
                    DEBUG.responseCount = DEBUG.responseCount + 1
                    DEBUG.responseTimeSum = DEBUG.responseTimeSum + response_time
                    if response_time < DEBUG.responseTimeMin then
                        DEBUG.responseTimeMin = response_time
                    end
                    if response_time > DEBUG.responseTimeMax then
                        DEBUG.responseTimeMax = response_time
                    end
                end
            end
            
            resetParamRequestTimer()  -- reset timer to detect lost ITEM2/3/4
            if DEVICE.PARAM_LIST == nil then
                paramsError()
            elseif index < 128 then

                    DEVICE.PARAM_LIST[index] = cmd
                    DEVICE.PARAM_LIST[index].typ = mb_to_u8(cmd.payload, 1)
                    DEVICE.PARAM_LIST[index].name = mb_to_string(cmd.payload, 2, 16)
                    DEVICE.PARAM_LIST[index].value = mb_to_value_or_str6(cmd.payload, 18, DEVICE.PARAM_LIST[index].typ)
                    DEVICE.PARAM_LIST[index].min = 0
                    DEVICE.PARAM_LIST[index].max = 0
                    DEVICE.PARAM_LIST[index].unit = ""
                    DEVICE.PARAM_LIST[index].options = {}
                    DEVICE.PARAM_LIST[index].allowed_mask = 65536
                    DEVICE.PARAM_LIST[index].editable = true
                    
                    -- smart RX loading: only when in skip mode (TX-only, no RX connected)
                    if LOAD.skip_rx then
                        if string.sub(DEVICE.PARAM_LIST[index].name, 1, 3) == "Rx " then
                            -- found first RX param - complete load early (TX-only mode)
                            LOAD.rx_first_idx = index  -- remember where RX params start
                            DEVICE.PARAM_LIST[index] = nil  -- don't keep this RX param
                            LOAD.complete = true
                            LOAD.request_t_last = 0
                            DEBUG.restartCount = 0
                            DEBUG.loadDuration = getTime() - DEBUG.loadStartTime
                            LOAD.tx_only_done = true
                            LOAD.is_running = false
                        end
                    end

            elseif index == 255 then -- eol (end of list :)
                LOAD.complete = true
                LOAD.request_t_last = 0  -- stop retry timer
                DEBUG.restartCount = 0  -- reset for next load cycle
                DEBUG.loadDuration = getTime() - DEBUG.loadStartTime  -- record total load time
                LOAD.is_running = false
            else
                paramsError()
            end
            end  -- close if index check
        elseif cmd.cmd == MB.CMD_PARAM_ITEM2 then
            -- MB.CMD_PARAM_ITEM2
            local index = cmd.payload[0]
            -- silently ignore if index doesn't match current (likely duplicate from retry)
            if index ~= LOAD.current_index then
                -- ignore stale response
                DEBUG.duplicates = DEBUG.duplicates + 1
            elseif DEVICE.PARAM_LIST == nil or DEVICE.PARAM_LIST[index] == nil then
                paramsError()
            else
                local item3_needed = false
                if DEVICE.PARAM_LIST[index].typ < MB.TYPE_LIST then
                    DEVICE.PARAM_LIST[index].min = mb_to_value(cmd.payload, 1, DEVICE.PARAM_LIST[index].typ)
                    DEVICE.PARAM_LIST[index].max = mb_to_value(cmd.payload, 3, DEVICE.PARAM_LIST[index].typ)
                    DEVICE.PARAM_LIST[index].unit = mb_to_string(cmd.payload, 7, 6)
                elseif DEVICE.PARAM_LIST[index].typ == MB.TYPE_LIST then
                    DEVICE.PARAM_LIST[index].allowed_mask = mb_to_u16(cmd.payload, 1)
                    DEVICE.PARAM_LIST[index].options = mb_to_options(cmd.payload, 3, 21)
                    DEVICE.PARAM_LIST[index].item2payload = cmd.payload
                    DEVICE.PARAM_LIST[index].min = 0
                    DEVICE.PARAM_LIST[index].max = #DEVICE.PARAM_LIST[index].options - 1
                    DEVICE.PARAM_LIST[index].editable = mb_allowed_mask_editable(DEVICE.PARAM_LIST[index].allowed_mask)
                    -- determine if we should expect an ITEM3
                    local s = mb_to_string(cmd.payload, 3, 21)
                    if string.len(s) == 21 then item3_needed = true end
                elseif DEVICE.PARAM_LIST[index].typ == MB.TYPE_STR6 then
                    -- nothing to do, is send but hasn't any content
                else
                    paramsError()
                end
                if not item3_needed then
                    -- request next parameter by index (pull-based protocol)
                    -- this ensures we explicitly fetch each parameter after processing the current one
                    startParamRequest(LOAD.expected_index)
                else
                    resetParamRequestTimer()  -- reset timer to detect lost ITEM3
                end
            end
        elseif cmd.cmd == MB.CMD_PARAM_ITEM3 then
            -- MB.CMD_PARAM_ITEM3
            local index = cmd.payload[0]
            local is_item4 = false
            if (index >= 128) then -- this is actually ITEM4
                index = index - 128;
                is_item4 = true
            end
            -- silently ignore if index doesn't match current (likely duplicate from retry)
            if index ~= LOAD.current_index then
                -- ignore stale response
                DEBUG.duplicates = DEBUG.duplicates + 1
            elseif DEVICE.PARAM_LIST == nil or DEVICE.PARAM_LIST[index] == nil then
                paramsError()
            elseif DEVICE.PARAM_LIST[index].typ ~= MB.TYPE_LIST then
                paramsError()
            elseif DEVICE.PARAM_LIST[index].item2payload == nil then
                paramsError()
            elseif is_item4 and DEVICE.PARAM_LIST[index].item3payload == nil then
                paramsError()
            else
                local s = DEVICE.PARAM_LIST[index].item2payload
                local item4_needed = false
                if not is_item4 then
                    DEVICE.PARAM_LIST[index].item3payload = cmd.payload
                    for i=1,23 do s[23+i] = cmd.payload[i] end
                    DEVICE.PARAM_LIST[index].options = mb_to_options(s, 3, 21+23)
                    -- determine if we should expect an ITEM4
                    local opts = mb_to_string(cmd.payload, 1, 23)
                    if string.len(opts) == 23 then item4_needed = true end
                else
                    local s3 = DEVICE.PARAM_LIST[index].item3payload
                    for i=1,23 do s[23+i] = s3[i]; s[23+23+i] = cmd.payload[i]; end
                    DEVICE.PARAM_LIST[index].options = mb_to_options(s, 3, 21+23+23)
                end
                DEVICE.PARAM_LIST[index].max = #DEVICE.PARAM_LIST[index].options - 1
                s = nil
                if not item4_needed then
                    -- request next parameter by index (pull-based protocol)
                    -- all ITEM3/ITEM4 processing complete for current param, fetch next one
                    startParamRequest(LOAD.expected_index)
                else
                    resetParamRequestTimer()  -- reset timer to detect lost ITEM4
                end
            end
        end
        cmd = nil
    end --for
end


local function sendParamSet(idx)
    if DEVICE.PARAM_LIST == nil then return end -- needed here??
    local p = DEVICE.PARAM_LIST[idx]
    if p == nil then return end
    
    local payload = {idx}
    local val = p.value

    if p.typ == MB.TYPE_UINT8 or p.typ == MB.TYPE_LIST then
        payload[2] = bit32.band(val, 0xFF)
    elseif p.typ == MB.TYPE_INT8 then
        payload[2] = bit32.band(val, 0xFF)
    elseif p.typ == MB.TYPE_UINT16 or p.typ == MB.TYPE_INT16 then
        payload[2] = bit32.band(val, 0xFF)
        payload[3] = bit32.band(bit32.rshift(val, 8), 0xFF)
    elseif p.typ == MB.TYPE_STR6 then
        for i = 1,6 do
            payload[i+1] = string.byte(string.sub(p.value, i,i))
        end
    end

    cmdPush(MB.CMD_PARAM_SET, payload)
end


local function sendParamStore()
    if DEVICE.PARAM_LIST == nil then return end -- needed here??
    cmdPush(MB.CMD_PARAM_STORE, {})
    LOAD.save_t_last = getTime()
    setPopupWTmo("Save Parameters", 250)
end


local function sendBind()
    --if not LOAD.complete then return end -- needed here??
    if LOAD.is_running then return end
    cmdPush(MB.CMD_BIND_START, {})
    setPopupBlocked("Binding")
end


local function sendBoot()
    --if not LOAD.complete then return end -- needed here??
    if LOAD.is_running then return end
    cmdPush(MB.CMD_SYSTEM_BOOTLOADER, {})
    setPopupBlocked("In System Bootloader")
end


local function sendFlashEsp()
    if LOAD.is_running then return end
    cmdPush(MB.CMD_FLASH_ESP, {})
    setPopupBlocked("Flash ESP")
end


local function checkBind()
    if LOAD.is_running then return end
    if DEVICE.INFO ~= nil and DEVICE.INFO.has_status == 1 and DEVICE.INFO.binding == 1 then
        setPopupBlocked("Binding")
    end
end


local function checkLQ()
--[[
-- the mechanism doesn't work well when a receiver is connected while the Lua is running
-- since in the first second the LQ may not be high
-- so we disable it for until a better approach is found
    if LOAD.is_running or not CONN.active then
        if POPUP.active and POPUP.t_end_10ms < 0 and POPUP.text == "LQ is low" then clearPopup() end
        return
    end
    if DEVICE.INFO ~= nil and DEVICE.INFO.has_status == 1 and DEVICE.INFO.LQ_low > 1 then
        setPopupBlocked("LQ is low")
    end
--]]
end


----------------------------------------------------------------------
-- UI definitions
----------------------------------------------------------------------

-- consolidated page/UI index constants (saves 17 locals)
local UI = {
    -- pages
    PAGE_MAIN = 0,
    PAGE_EDIT_TX = 1,
    PAGE_EDIT_RX = 2,
    PAGE_TOOLS = 3,
    -- main page cursor indices
    IDX_BINDPHRASE = 0,
    IDX_MODE = 1,
    IDX_RFBAND = 2,
    IDX_RFORTHO = 3,
    IDX_EDIT_TX = 4,
    IDX_EDIT_RX = 5,
    IDX_SAVE = 6,
    IDX_RELOAD = 7,
    IDX_BIND = 8,
    IDX_TOOLS = 9,
    MAIN_CURSOR_MAX = 9,
    -- tools page cursor indices
    IDX_TOOLS_BOOT = 0,
    IDX_TOOLS_FLASH_ESP = 1,
    IDX_TOOLS_DEBUG_STATS = 2,
    TOOLS_CURSOR_MAX = 2,
    -- common params
    COMMON_PARAM_MAX = 3,
    BINDPHRASE_CHARS = "abcdefghijklmnopqrstuvwxyz0123456789_#-.",
}

local CURSOR = {
    page = UI.PAGE_MAIN,    -- current page (0: main, 1: edit Tx, 2: edit Rx, 3: tools)
    idx = UI.IDX_EDIT_TX,
    edit = false,
    option_value = 0,
    pidx = 0,               -- parameter idx which corresponds to the current cursor idx
    x_idx = 0,              -- index into string for string edits
    param_cnt = 0,          -- number of parameters available on page
    top_idx = 0,            -- index of first displayed option (scroll position)
    param_list = {},        -- parameter list for current page
}


local function cur_attr(idx) -- used in menu
    local attr = THEME.textColor
    if CURSOR.idx == idx then
        attr = attr + INVERS
        if CURSOR.edit then attr = attr + BLINK end
    end
    return attr
end


local function cur_attr_x(idx, x_idx) -- for bind phrase character editing
    local attr = THEME.textColor
    if LOAD.complete and CURSOR.idx == idx then
        if CURSOR.edit then
            if CURSOR.x_idx == x_idx then attr = attr + BLINK + INVERS end
        else
            attr = attr + INVERS
        end
    end
    return attr
end


local function cur_attr_p(idx, pidx) -- used for parameters
    local attr = cur_attr(idx)
    if LOAD.complete and not DEVICE.PARAM_LIST[pidx].editable then
        lcd.setColor(CUSTOM_COLOR, GREY)
        attr = CUSTOM_COLOR
    end
    return attr
end


local function param_value_inc(idx)
    if DEVICE.PARAM_LIST == nil then return end -- needed here??
    local p = DEVICE.PARAM_LIST[idx]
    if p == nil then return end
    if p.typ < MB.TYPE_LIST then
        p.value = p.value + 1
    elseif p.typ == MB.TYPE_LIST then
        local value = p.value
        while value <= p.max do
            value = value + 1
            local m = bit32.lshift(1,value)
            if bit32.btest(m, p.allowed_mask) then p.value = value; break end
        end
    end
    if p.value > p.max then p.value = p.max end
    DEVICE.PARAM_LIST[idx].value = p.value
end


local function param_value_dec(idx)
    if DEVICE.PARAM_LIST == nil then return end -- needed here??
    local p = DEVICE.PARAM_LIST[idx]
    if p == nil then return end
    if p.typ < MB.TYPE_LIST then
        p.value = p.value - 1
    elseif p.typ == MB.TYPE_LIST then
        local value = p.value
        while value >= p.min do
            value = value - 1
            local m = bit32.lshift(1,value)
            if bit32.btest(m, p.allowed_mask) then p.value = value; break end
        end
    end
    if p.value < p.min then p.value = p.min end
    DEVICE.PARAM_LIST[idx].value = p.value
end


local function param_str6_inc(idx)
    if DEVICE.PARAM_LIST == nil then return end -- needed here??
    local p = DEVICE.PARAM_LIST[idx]
    if p == nil then return end
    if p.typ == MB.TYPE_STR6 then
        local c = string.sub(p.value, CURSOR.x_idx+1, CURSOR.x_idx+1)
        local i = string.find(UI.BINDPHRASE_CHARS, c, 1, true) -- true for plain search
        i = i + 1
        if i > string.len(UI.BINDPHRASE_CHARS) then i = 1 end
        c = string.sub(UI.BINDPHRASE_CHARS, i,i)
        p.value = string.sub(p.value, 1, CURSOR.x_idx) .. c .. string.sub(p.value, CURSOR.x_idx+2, string.len(p.value))
    end
    DEVICE.PARAM_LIST[idx].value = p.value
end


local function param_str6_dec(idx)
    if DEVICE.PARAM_LIST == nil then return end -- needed here??
    local p = DEVICE.PARAM_LIST[idx]
    if p == nil then return end
    if p.typ == MB.TYPE_STR6 then
        local c = string.sub(p.value, CURSOR.x_idx+1, CURSOR.x_idx+1)
        local i = string.find(UI.BINDPHRASE_CHARS, c, 1, true) -- true for plain search
        i = i - 1
        if i < 1 then i = string.len(UI.BINDPHRASE_CHARS) end
        c = string.sub(UI.BINDPHRASE_CHARS, i,i)
        p.value = string.sub(p.value, 1, CURSOR.x_idx) .. c .. string.sub(p.value, CURSOR.x_idx+2, string.len(p.value))
    end
    DEVICE.PARAM_LIST[idx].value = p.value
end


local function param_str6_next(idx)
    if DEVICE.PARAM_LIST == nil then return false end -- needed here??
    local p = DEVICE.PARAM_LIST[idx]
    if p == nil then return end
    if p.typ == MB.TYPE_STR6 then
        CURSOR.x_idx = CURSOR.x_idx + 1
        if CURSOR.x_idx >= string.len(p.value) then
            return true -- last char
        end
    end
    return false
end


local function param_focusable(idx)
    if DEVICE.PARAM_LIST == nil then return false end -- needed here??
    local p = DEVICE.PARAM_LIST[idx]
    if p == nil then return false end
    if p.editable == nil then return false end
    return p.editable
end


local function hasTxEspWifiParams()
    if DEVICE.PARAM_LIST == nil then return false end
    local hasProtocol = false
    local hasChannel = false
    local hasPower = false
    for _, p in pairs(DEVICE.PARAM_LIST) do
        if p.name == "Tx Wifi Protocol" then hasProtocol = true
        elseif p.name == "Tx Wifi Channel" then hasChannel = true
        elseif p.name == "Tx Wifi Power" then hasPower = true
        end
    end
    return hasProtocol and hasChannel and hasPower
end


----------------------------------------------------------------------
-- Page Edit Tx/Rx
----------------------------------------------------------------------

local function buildPageParamList(page_str)
    CURSOR.param_list = {}
    if DEVICE.PARAM_LIST == nil then return end
    for pidx = 2, #DEVICE.PARAM_LIST do
        local p = DEVICE.PARAM_LIST[pidx]
        if p ~= nil and string.sub(p.name,1,2) == page_str and p.allowed_mask > 0 then
            table.insert(CURSOR.param_list, pidx)
        end
    end
    CURSOR.param_cnt = #CURSOR.param_list
end


local function drawPageEdit(page_str)
    local y = LAYOUT.HEADER_Y
    if page_str == "Tx" then
        if DEVICE.INFO ~= nil then
            lcd.drawText(5, y, "Tx - "..tostring(DEVICE.INFO.tx_config_id)..":", THEME.textColor)
        else
            lcd.drawText(5, y, "Tx - ?:", THEME.textColor)
        end
    else
        lcd.drawText(5, y, page_str..":", THEME.textColor)
    end

    y = LAYOUT.EDIT_START_Y
    local dy = LAYOUT.TEXT_DY
    local y0 = y
    local text_attr = THEME.textColor + LAYOUT.TEXT_SIZE

    if CURSOR.idx < CURSOR.top_idx then CURSOR.top_idx = CURSOR.idx end
    if CURSOR.idx >= CURSOR.top_idx + LAYOUT.PAGE_N then CURSOR.top_idx = CURSOR.idx - LAYOUT.PAGE_N + 1 end

    CURSOR.param_cnt = #CURSOR.param_list
    if CURSOR.param_list[CURSOR.idx + 1] then
        CURSOR.pidx = CURSOR.param_list[CURSOR.idx + 1]
    end

    local loop_end = CURSOR.top_idx + LAYOUT.PAGE_N
    if loop_end > CURSOR.param_cnt then loop_end = CURSOR.param_cnt end

    for idx = CURSOR.top_idx, loop_end - 1 do
        local pidx = CURSOR.param_list[idx + 1]
        local p = DEVICE.PARAM_LIST[pidx]
        local name = string.sub(p.name, 4)

        local shifted_idx = idx - CURSOR.top_idx
        y = y0 + shifted_idx * dy

        local xofs = 0
        local col2_offset = 230
        local rows_in_col1 = LAYOUT.PAGE_N1
        if THEME.screenSize == 800480 then
            col2_offset = 380
        end
        -- balance columns: Tx on all larger screens, Rx only on 800x480
        local should_balance = (page_str == "Tx" and THEME.screenSize ~= 320240) or
                               (page_str == "Rx" and THEME.screenSize == 800480)
        if should_balance then
            local items_on_page = math.min(LAYOUT.PAGE_N, CURSOR.param_cnt - CURSOR.top_idx)
            rows_in_col1 = math.ceil(items_on_page / 2)
        end
        if shifted_idx >= rows_in_col1 then y = y - rows_in_col1*dy; xofs = col2_offset end

        lcd.drawText(10+xofs, y, name, text_attr)
        if p.typ < MB.TYPE_LIST then
            lcd.drawText(LAYOUT.TEXT_VAL_X+xofs, y, p.value.." "..p.unit, cur_attr_p(idx, pidx) + LAYOUT.TEXT_SIZE)
        elseif p.typ == MB.TYPE_LIST then
            lcd.drawText(LAYOUT.TEXT_VAL_X+xofs, y, p.options[p.value+1], cur_attr_p(idx, pidx) + LAYOUT.TEXT_SIZE)
        end
    end

    local x_mid = LAYOUT.W_HALF - 5
    if CURSOR.top_idx > 0 then
        local y_base = y0 - 4
        --lcd.drawFilledTriangle(x_mid-6, y_base, x_mid+6, y_base, x_mid, y_base-6, THEME.textColor)
        drawFilledTriangle(x_mid-6, y_base, x_mid+6, y_base, x_mid, y_base-6, THEME.textColor)
    end
    if CURSOR.param_cnt > CURSOR.top_idx + LAYOUT.PAGE_N then
        local y_base = y0 + LAYOUT.PAGE_N1*dy + 4
        --lcd.drawFilledTriangle(x_mid-6, y_base, x_mid+6, y_base, x_mid, y_base+6, THEME.textColor)
        drawFilledTriangle(x_mid-6, y_base, x_mid+6, y_base, x_mid, y_base+6, THEME.textColor)
    end
end


local function doPageEdit(event, page_str)
    lcd.drawFilledRectangle(0, 0, LCD_W, 30, THEME.titleBgColor)
    lcd.drawText(5, 5, "mLRS Configurator: Edit "..page_str, THEME.menuTitleColor)

    if event == EVT_VIRTUAL_EXIT and not CURSOR.edit then
        CURSOR.page = UI.PAGE_MAIN
        CURSOR.idx = UI.IDX_EDIT_TX
        return
    end

    drawPageEdit(page_str) -- call before event handling, ensures that CURSOR.param_cnt, CURSOR.pidx are set

    if not CURSOR.edit then
        if event == EVT_VIRTUAL_EXIT then
        elseif event == EVT_VIRTUAL_ENTER then
            CURSOR.edit = true
        elseif event == EVT_VIRTUAL_NEXT then
            CURSOR.idx = CURSOR.idx + 1
            if CURSOR.idx >= CURSOR.param_cnt then CURSOR.idx = CURSOR.param_cnt - 1 end
        elseif event == EVT_VIRTUAL_PREV then
            CURSOR.idx = CURSOR.idx - 1
            if CURSOR.idx < 0 then CURSOR.idx = 0 end
        end
    else
        if event == EVT_VIRTUAL_EXIT then
            sendParamSet(CURSOR.pidx)
            CURSOR.edit = false
        elseif event == EVT_VIRTUAL_ENTER then
            sendParamSet(CURSOR.pidx)
            CURSOR.edit = false
        elseif event == EVT_VIRTUAL_NEXT then
            param_value_inc(CURSOR.pidx)
        elseif event == EVT_VIRTUAL_PREV then
            param_value_dec(CURSOR.pidx)
        end
    end

end


----------------------------------------------------------------------
-- Page Tools
----------------------------------------------------------------------

local function drawPageTools()
    local x, y;

    local module1 = model.getModule(1)
    local isExternal = (module1.Type == 5)
    local hasWifi = hasTxEspWifiParams()
    local showFlashEsp = isExternal and hasWifi

    y = LAYOUT.EDIT_START_Y
    lcd.drawText(10, y, "System Bootloader", cur_attr(UI.IDX_TOOLS_BOOT) + LAYOUT.TEXT_SIZE)
    
    y = y + LAYOUT.TEXT_DY + LAYOUT.MENU_GAP
    if showFlashEsp then
        lcd.drawText(10, y, "Flash ESP", cur_attr(UI.IDX_TOOLS_FLASH_ESP) + LAYOUT.TEXT_SIZE)
        y = y + LAYOUT.TEXT_DY + LAYOUT.MENU_GAP
    end

    local debug_status = DEBUG.enabled and "ON" or "OFF"
    lcd.drawText(10, y, "Debug Stats: "..debug_status, cur_attr(UI.IDX_TOOLS_DEBUG_STATS) + LAYOUT.TEXT_SIZE)
end


local function doPageTools(event)
    lcd.drawFilledRectangle(0, 0, LCD_W, 30, THEME.titleBgColor)
    lcd.drawText(5, 5, "mLRS Configurator: Tools Page", THEME.menuTitleColor)

    local module1 = model.getModule(1)
    local isExternal = (module1.Type == 5)
    local hasWifi = hasTxEspWifiParams()
    local showFlashEsp = isExternal and hasWifi

    drawPageTools()

    if event == EVT_VIRTUAL_ENTER then
        if CURSOR.idx == UI.IDX_TOOLS_BOOT then -- Boot
            CURSOR.page = UI.PAGE_MAIN
            CURSOR.idx = UI.IDX_EDIT_TX
            sendBoot()
        elseif showFlashEsp and CURSOR.idx == UI.IDX_TOOLS_FLASH_ESP then -- flash esp
            CURSOR.page = UI.PAGE_MAIN
            CURSOR.idx = UI.IDX_EDIT_TX
            sendFlashEsp()
        elseif CURSOR.idx == UI.IDX_TOOLS_DEBUG_STATS then -- toggle debug stats
            DEBUG.enabled = not DEBUG.enabled
        end
    elseif event == EVT_VIRTUAL_EXIT then
        CURSOR.page = UI.PAGE_MAIN
        CURSOR.idx = UI.IDX_EDIT_TX
        return
    elseif event == EVT_VIRTUAL_NEXT then
        CURSOR.idx = CURSOR.idx + 1
        if not showFlashEsp and CURSOR.idx == UI.IDX_TOOLS_FLASH_ESP then CURSOR.idx = CURSOR.idx + 1 end
        if CURSOR.idx > UI.TOOLS_CURSOR_MAX then CURSOR.idx = UI.TOOLS_CURSOR_MAX end
    elseif event == EVT_VIRTUAL_PREV then
        CURSOR.idx = CURSOR.idx - 1
        if not showFlashEsp and CURSOR.idx == UI.IDX_TOOLS_FLASH_ESP then CURSOR.idx = CURSOR.idx - 1 end
        if CURSOR.idx < 0 then CURSOR.idx = 0 end
    end
end


----------------------------------------------------------------------
-- Page Main
----------------------------------------------------------------------

local function drawSetuplayoutWarning(txt)
    local y = LAYOUT.INFO_Y
    lcd.drawFilledRectangle(LAYOUT.WARN_X-2, y, LAYOUT.WARN_W+4, LAYOUT.WARN_H+4, THEME.textColor) --TITLE_BGCOLOR)
    lcd.drawFilledRectangle(LAYOUT.WARN_X, y+2, LAYOUT.WARN_W, LAYOUT.WARN_H, THEME.titleBgColor) --TEXT_BGCOLOR) --TITLE_BGCOLOR)
    local attr = THEME.menuTitleColor+CENTER
    local s = txt
    local i = string.find(s, "\n")
    local lines = 0
    while i ~= nil do
        local s1 = string.sub(s, 1,i-1)
        lcd.drawText(LAYOUT.W_HALF, y+6 + lines, s1, attr)
        s = string.sub(s, i+1)
        i = string.find(s, "\n")
        lines = lines + LAYOUT.LINE_HEIGHT
    end
    lcd.drawText(LAYOUT.W_HALF, y+6 + lines, s, attr)
end


local function drawPageMain()
    lcd.setColor(CUSTOM_COLOR, RED)

    local y = LAYOUT.HEADER_Y
    lcd.drawText(5, y, "Tx:", THEME.textColor)
    if DEVICE.ITEM_TX == nil then
        lcd.drawText(35, y, "---", THEME.textColor)
    else
        lcd.drawText(35, y, DEVICE.ITEM_TX.name, THEME.textColor+SMLSIZE)
        lcd.drawText(35, y + LAYOUT.HEADER_LINE_SMALL, DEVICE.ITEM_TX.version_str, THEME.textColor+SMLSIZE)
        if DEVICE.INFO ~= nil then
            lcd.drawText(35, y + 2*LAYOUT.HEADER_LINE_SMALL, "ConfigId "..tostring(DEVICE.INFO.tx_config_id), THEME.textColor+SMLSIZE)
        end
    end

    lcd.drawText(LAYOUT.W_HALF, y, "Rx:", THEME.textColor)
    --if not DEVICE.PARAM_LIST_complete then
        -- don't do anything
    if not CONN.active then
        lcd.drawText(LAYOUT.W_HALF+30, y, "not connected", THEME.textColor)
    elseif DEVICE.ITEM_RX == nil then
        lcd.drawText(LAYOUT.W_HALF+30, y, "---", THEME.textColor)
    else
        lcd.drawText(LAYOUT.W_HALF+30, y, DEVICE.ITEM_RX.name, THEME.textColor+SMLSIZE)
        lcd.drawText(LAYOUT.W_HALF+30, y + LAYOUT.HEADER_LINE_SMALL, DEVICE.ITEM_RX.version_str, THEME.textColor+SMLSIZE)
    end

    local version_error = false
    -- skip version check if version is 0 (uninitialized/default)
    if DEVICE.ITEM_TX ~= nil and DEVICE.ITEM_TX.version_int > 0 and DEVICE.ITEM_TX.version_int < VERSION.REQUIRED_TX then
        version_error = true
        POPUP.text = "Tx version not supported\nby this Lua script!"
    end
    if DEVICE.ITEM_RX ~= nil and CONN.active and DEVICE.ITEM_RX.version_int > 0 and DEVICE.ITEM_RX.version_int < VERSION.REQUIRED_RX then
        version_error = true
        POPUP.text = "Rx version not supported\nby this Lua script!"
    end
    if version_error then
        drawPopup()
        return
    end

    y = LAYOUT.MAIN_START_Y
    lcd.drawText(10, y, "Bind Phrase", THEME.textColor + LAYOUT.TEXT_SIZE)
    if DEVICE.PARAM_LIST ~= nil and DEVICE.PARAM_LIST[0] ~= nil then
        local x = LAYOUT.TEXT_VAL_X
        for i = 1,6 do
            local c = string.sub(DEVICE.PARAM_LIST[0].value, i, i) -- param_idx = 0 = BindPhrase
            local attr = cur_attr_x(0, i-1) + LAYOUT.TEXT_SIZE
            lcd.drawText(x, y, c, attr)
            --x = x + lcd.getTextWidth(c,1,attr)+1
            x = x + getCharWidth(c) + 1
            if i == 6 and DEVICE.PARAM_LIST[2] ~= nil and DEVICE.PARAM_LIST[2].value == 0 then -- do only for 2.4GHz band
                lcd.drawText(LAYOUT.TEXT_VAL_X + LAYOUT.EXCEPT_STR_OFFSET, y, getExceptStrFromChar(c), THEME.textColor + LAYOUT.TEXT_SIZE)
            end
        end
    end

    lcd.drawText(10, y + LAYOUT.LINE_HEIGHT, "Mode", THEME.textColor + LAYOUT.TEXT_SIZE)
    if DEVICE.PARAM_LIST ~= nil and DEVICE.PARAM_LIST[1] ~= nil then
        local p = DEVICE.PARAM_LIST[1] -- param_idx = 1 = Mode
        if p.options[p.value+1] ~= nil then
            lcd.drawText(LAYOUT.TEXT_VAL_X, y + LAYOUT.LINE_HEIGHT, p.options[p.value+1], cur_attr_p(UI.IDX_MODE,1) + LAYOUT.TEXT_SIZE)
        end
    end

    lcd.drawText(10, y + 2*LAYOUT.LINE_HEIGHT, "RF Band", THEME.textColor + LAYOUT.TEXT_SIZE)
    if DEVICE.PARAM_LIST ~= nil and DEVICE.PARAM_LIST[2] ~= nil then
        local p = DEVICE.PARAM_LIST[2] -- param_idx = 2 = RfBand
        if p.options[p.value+1] ~= nil then
            --lcd.drawText(240+80, y, p.options[p.value+1], cur_attr(2))
            if p.value <= #freq_band_list then
                lcd.drawText(LAYOUT.TEXT_VAL_X, y + 2*LAYOUT.LINE_HEIGHT, freq_band_list[p.value], cur_attr_p(UI.IDX_RFBAND,2) + LAYOUT.TEXT_SIZE)
            else
                lcd.drawText(LAYOUT.TEXT_VAL_X, y + 2*LAYOUT.LINE_HEIGHT, p.options[p.value+1], cur_attr_p(UI.IDX_RFBAND,2) + LAYOUT.TEXT_SIZE)
            end
        end
    end

    if DEVICE.PARAM_LIST ~= nil and DEVICE.PARAM_LIST[3] ~= nil and DEVICE.PARAM_LIST[3].allowed_mask > 0 then
        local y_ortho = y
        if THEME.screenSize == 320240 then y_ortho = y + 2*LAYOUT.LINE_HEIGHT end

        lcd.drawText(LAYOUT.W_HALF+30, y_ortho, "Ortho", THEME.textColor + LAYOUT.TEXT_SIZE)
        local p = DEVICE.PARAM_LIST[3] -- param_idx = 3 = RfOrtho
        if p.options[p.value+1] ~= nil then
            lcd.drawText(LAYOUT.W_HALF+90, y_ortho, p.options[p.value+1], cur_attr_p(UI.IDX_RFORTHO,3) + LAYOUT.TEXT_SIZE)
        end
    end

    y = LAYOUT.BUTTONS_Y
    lcd.drawText(LAYOUT.EDIT_TX_X, y, "Edit Tx", cur_attr(UI.IDX_EDIT_TX) + LAYOUT.TEXT_SIZE)
    if not CONN.active then
        lcd.drawText(LAYOUT.EDIT_RX_X, y, "Edit Rx", THEME.textDisableColor + LAYOUT.TEXT_SIZE)
    else
        lcd.drawText(LAYOUT.EDIT_RX_X, y, "Edit Rx", cur_attr(UI.IDX_EDIT_RX) + LAYOUT.TEXT_SIZE)
    end
    lcd.drawText(LAYOUT.SAVE_X, y, "Save", cur_attr(UI.IDX_SAVE) + LAYOUT.TEXT_SIZE)
    lcd.drawText(LAYOUT.RELOAD_X, y, "Reload", cur_attr(UI.IDX_RELOAD) + LAYOUT.TEXT_SIZE)
    lcd.drawText(LAYOUT.BIND_X, y, "Bind", cur_attr(UI.IDX_BIND) + LAYOUT.TEXT_SIZE)
    lcd.drawText(LAYOUT.TOOLS_X, y, "Tools", cur_attr(UI.IDX_TOOLS) + LAYOUT.TEXT_SIZE)

    -- show overview of some selected parameters
    y = LAYOUT.INFO_Y --210 --05
    lcd.setColor(CUSTOM_COLOR, GREY)
    lcd.drawFilledRectangle(0, y-5, LCD_W, 1, CUSTOM_COLOR)

    --if not LOAD.complete then
    if LOAD.is_running then
        --lcd.drawText(130, y+20, "parameters loading ...", THEME.textColor+BLINK+INVERS)
        --return
    end

    lcd.drawText(LAYOUT.INFO_LEFT_X, y, "Tx Power", THEME.textColor + LAYOUT.TEXT_SIZE)
    lcd.drawText(LAYOUT.INFO_LEFT_X, y+LAYOUT.INFO_DY, "Tx Diversity", THEME.textColor + LAYOUT.TEXT_SIZE)
    if DEVICE.INFO ~= nil then
        lcd.drawText(LAYOUT.INFO_LEFT_VAL_X, y, tostring(DEVICE.INFO.tx_power_dbm).." dBm", THEME.textColor + LAYOUT.TEXT_SIZE)
        if DEVICE.INFO.tx_diversity <= #diversity_list then
            lcd.drawText(LAYOUT.INFO_LEFT_VAL_X, y+LAYOUT.INFO_DY, diversity_list[DEVICE.INFO.tx_diversity], THEME.textColor + LAYOUT.TEXT_SIZE)
        else
            lcd.drawText(LAYOUT.INFO_LEFT_VAL_X, y+LAYOUT.INFO_DY, "?", THEME.textColor + LAYOUT.TEXT_SIZE)
        end
    else
        lcd.drawText(LAYOUT.INFO_LEFT_VAL_X, y, "---", THEME.textColor + LAYOUT.TEXT_SIZE)
        lcd.drawText(LAYOUT.INFO_LEFT_VAL_X, y+LAYOUT.INFO_DY, "---", THEME.textColor + LAYOUT.TEXT_SIZE)
    end

    local rx_attr = THEME.textColor + LAYOUT.TEXT_SIZE
    if not CONN.active then
        rx_attr = THEME.textDisableColor + LAYOUT.TEXT_SIZE
    end
    lcd.drawText(LAYOUT.INFO_RIGHT_X, y, "Rx Power", rx_attr)
    lcd.drawText(LAYOUT.INFO_RIGHT_X, y+LAYOUT.INFO_DY, "Rx Diversity", rx_attr)
    if DEVICE.INFO ~= nil and CONN.active then
        lcd.drawText(LAYOUT.INFO_RIGHT_VAL_X, y, tostring(DEVICE.INFO.rx_power_dbm).." dBm", rx_attr)
        if DEVICE.INFO.rx_diversity <= #diversity_list then
            lcd.drawText(LAYOUT.INFO_RIGHT_VAL_X, y+LAYOUT.INFO_DY, diversity_list[DEVICE.INFO.rx_diversity], rx_attr)
        else
            lcd.drawText(LAYOUT.INFO_RIGHT_VAL_X, y+LAYOUT.INFO_DY, "?", rx_attr)
        end
    else
        lcd.drawText(LAYOUT.INFO_RIGHT_VAL_X, y, "---", rx_attr)
        lcd.drawText(LAYOUT.INFO_RIGHT_VAL_X, y+LAYOUT.INFO_DY, "---", rx_attr)
    end

    y = y + 2*LAYOUT.INFO_DY
    lcd.drawText(LAYOUT.INFO_LEFT_X, y, "Sensitivity", THEME.textColor + LAYOUT.TEXT_SIZE)
    if DEVICE.INFO ~= nil then
        lcd.drawText(LAYOUT.INFO_LEFT_VAL_X, y, tostring(DEVICE.INFO.receiver_sensitivity).." dBm", THEME.textColor + LAYOUT.TEXT_SIZE)
    else
        lcd.drawText(LAYOUT.INFO_LEFT_VAL_X, y, "---", THEME.textColor + LAYOUT.TEXT_SIZE)
    end

    if LOAD.is_running then
        local txt = "Tx params..."
        if DEVICE.PARAM_LIST ~= nil then
            local p = DEVICE.PARAM_LIST[LOAD.expected_index - 1]
            if p ~= nil and string.sub(p.name, 1, 3) == "Rx " then
                txt = "Rx params..."
            end
        end
        lcd.drawText(LAYOUT.INFO_RIGHT_X, y, txt , THEME.textColor + LAYOUT.TEXT_SIZE + BLINK)
    end

    -- setup layout warning
    if DEVICE.ITEM_TX ~= nil and DEVICE.ITEM_RX ~= nil and CONN.active and LOAD.complete and
           (DEVICE.ITEM_TX.setuplayout_int > 515 or DEVICE.ITEM_RX.setuplayout_int > 515) then -- 515 is old 335
        if DEVICE.ITEM_TX.setuplayout_int < DEVICE.ITEM_RX.setuplayout_int then
            drawSetuplayoutWarning("Tx param version smaller than Rx param version.\nPlease update Tx firmware!")
        elseif DEVICE.ITEM_RX.setuplayout_int < DEVICE.ITEM_TX.setuplayout_int then
            drawSetuplayoutWarning("Rx param version smaller than Tx param version.\nPlease update Rx firmware!")
        end
    end
end


local function doPageMain(event)
    lcd.drawFilledRectangle(0, 0, LCD_W, 30, THEME.titleBgColor)
    lcd.drawText(5, 5, "mLRS Configurator: Main Page", THEME.menuTitleColor)
    lcd.drawText(LCD_W-1, 0, VERSION.SCRIPT, THEME.menuTitleColor+TINSIZE+RIGHT)
    
    -- debug stats display (togglable via tools menu)
    if DEBUG.enabled then
        -- line 1: parameter metrics - response times (min/avg/max), consecutive retries, duplicates
        local avg_response = 0
        if DEBUG.responseCount > 0 then
            avg_response = math.floor(DEBUG.responseTimeSum / DEBUG.responseCount)
        end
        lcd.drawText(LCD_W-1, 10, "RT:"..tostring(DEBUG.responseTimeMin).."/"..tostring(avg_response).."/"..tostring(DEBUG.responseTimeMax).."  C:"..tostring(DEBUG.consecutive).."  D:"..tostring(DEBUG.duplicates), THEME.menuTitleColor+TINSIZE+RIGHT)
        -- line 2: overall metrics - retries, total, full restarts, load time (10ms units)
        local load_sec = string.format("%.1f", DEBUG.loadDuration / 100)
        lcd.drawText(LCD_W-1, 20, "R:"..tostring(DEBUG.retryAttempt).."  T:"..tostring(DEBUG.retryCount).."  S:"..tostring(DEBUG.restartCount).."  Lt:"..load_sec.."s", THEME.menuTitleColor+TINSIZE+RIGHT)
    end

    if not CURSOR.edit then
        if event == EVT_VIRTUAL_EXIT then
            -- nothing to do
        elseif event == EVT_VIRTUAL_ENTER then
            if CURSOR.idx == UI.IDX_EDIT_TX then -- EditTX pressed
                CURSOR.page = UI.PAGE_EDIT_TX
                buildPageParamList("Tx")
                CURSOR.idx = 0
                CURSOR.top_idx = 0
                return
            elseif CURSOR.idx == UI.IDX_EDIT_RX then -- EditRX pressed
                CURSOR.page = UI.PAGE_EDIT_RX
                buildPageParamList("Rx")
                CURSOR.idx = 0
                CURSOR.top_idx = 0
                return
            elseif CURSOR.idx == UI.IDX_SAVE then -- Save pressed
                sendParamStore()
                clearParams()
            elseif CURSOR.idx == UI.IDX_RELOAD then -- Reload pressed
                clearParams()
                resetDebugStats()  -- reset all debug counters on manual reload
            elseif CURSOR.idx == UI.IDX_BIND then -- Bind pressed
                sendBind()
            elseif CURSOR.idx == UI.IDX_TOOLS then -- Tools pressed
                CURSOR.page = UI.PAGE_TOOLS
                CURSOR.idx = 0
                CURSOR.top_idx = 0
                return
            elseif DEVICE.PARAM_LIST ~= nil then -- edit option, weaker check
                CURSOR.x_idx = 0
                CURSOR.edit = true
            end
        elseif event == EVT_VIRTUAL_NEXT then -- and DEVICE.PARAM_LIST_complete then
            CURSOR.idx = CURSOR.idx + 1
            if CURSOR.idx > UI.MAIN_CURSOR_MAX then CURSOR.idx = UI.MAIN_CURSOR_MAX end

            if CURSOR.idx == UI.IDX_MODE and not param_focusable(1) then CURSOR.idx = CURSOR.idx + 1 end
            if CURSOR.idx == UI.IDX_RFBAND and not param_focusable(2) then CURSOR.idx = CURSOR.idx + 1 end
            if CURSOR.idx == UI.IDX_RFORTHO and not param_focusable(3) then CURSOR.idx = CURSOR.idx + 1 end
            if CURSOR.idx == UI.IDX_EDIT_RX and not CONN.active then CURSOR.idx = CURSOR.idx + 1 end
        elseif event == EVT_VIRTUAL_PREV then -- and DEVICE.PARAM_LIST_complete then
            CURSOR.idx = CURSOR.idx - 1
            if CURSOR.idx < 0 then CURSOR.idx = 0 end

            if CURSOR.idx == UI.IDX_EDIT_RX and not CONN.active then CURSOR.idx = CURSOR.idx - 1 end
            if CURSOR.idx == UI.IDX_RFORTHO and not param_focusable(3) then CURSOR.idx = CURSOR.idx - 1 end
            if CURSOR.idx == UI.IDX_RFBAND and not param_focusable(2) then CURSOR.idx = CURSOR.idx - 1 end
            if CURSOR.idx == UI.IDX_MODE and not param_focusable(1) then CURSOR.idx = CURSOR.idx - 1 end
        end
    else
        if event == EVT_VIRTUAL_EXIT then
            if CURSOR.idx <= UI.COMMON_PARAM_MAX then -- BindPhrase, Mode, RF Band, RF Ortho
                sendParamSet(CURSOR.idx)
            end
            CURSOR.edit = false
        elseif event == EVT_VIRTUAL_ENTER then
            if CURSOR.idx == UI.IDX_BINDPHRASE then -- BindPhrase
                if param_str6_next(0) then
                    sendParamSet(0)
                    CURSOR.edit = false
                end
            elseif CURSOR.idx <= UI.COMMON_PARAM_MAX then -- Mode, RF Band, RF Ortho
                sendParamSet(CURSOR.idx)
                CURSOR.edit = false
            else
                CURSOR.edit = false
            end
        elseif event == EVT_VIRTUAL_NEXT then
            if CURSOR.idx == UI.IDX_BINDPHRASE then -- BindPhrase
                param_str6_inc(0)
            elseif CURSOR.idx <= UI.COMMON_PARAM_MAX then -- Mode, RF Band, RF Ortho
                param_value_inc(CURSOR.idx)
            end
        elseif event == EVT_VIRTUAL_PREV then
            if CURSOR.idx == UI.IDX_BINDPHRASE then -- BindPhrase
                param_str6_dec(0)
            elseif CURSOR.idx <= UI.COMMON_PARAM_MAX then -- Mode, RF Band, RF Ortho
                param_value_dec(CURSOR.idx)
            end
        end
    end

    drawPageMain()
end


----------------------------------------------------------------------
-- main loop
----------------------------------------------------------------------

-- add runtime flags to existing tables
THEME.isEdgeTx = false
POPUP.pending_rx_connected = false  -- delay popup until after screen is drawn

local function Do(event)
    doConnected()

    if CONN.just_connected then
        CONN.just_connected = false  -- reset so we don't keep calling clearParams() every loop
        -- only trigger RX-only reload for late connection (after TX-only load completed)
        -- skip if: already downloading, or already have full params (RX was connected at start)
        if LOAD.tx_only_done and LOAD.rx_first_idx >= 0 then
            -- late RX connection after TX-only load - fetch just RX params
            clearParams(true)  -- RX-only reload
            POPUP.pending_rx_connected = true  -- show POPUP.active after main screen is drawn
        elseif not LOAD.is_running and LOAD.complete then
            -- params already loaded - don't reload, just show POPUP.active
            POPUP.pending_rx_connected = true
        end
        -- if still downloading, do nothing - let current load continue
        clearPopupIfBlocked()
    end
    if CONN.just_disconnected then
        if not POPUP.active then setPopupWTmo("Receiver\nhas disconnected!", 100) end
    end
    if not CONN.active and CURSOR.page == UI.PAGE_EDIT_RX then
        CURSOR.page = UI.PAGE_MAIN
        CURSOR.idx = UI.IDX_EDIT_TX
    end

    doParamLoop()

    -- EdgeTx: don't display in param upload, EdgeTx is super slow
    if THEME.isEdgeTx and LOAD.is_running then return end

    lcd.clear()

    if CURSOR.page == UI.PAGE_EDIT_TX then
        doPageEdit(event,"Tx")
    elseif CURSOR.page == UI.PAGE_EDIT_RX then
        doPageEdit(event,"Rx")
    elseif CURSOR.page == UI.PAGE_TOOLS then
        doPageTools(event)
    else
        doPageMain(event)
    end

    checkBind()
    checkLQ()
    
    -- show receiver connected POPUP.active after main screen is drawn (not before)
    if POPUP.pending_rx_connected and not POPUP.active then
        POPUP.pending_rx_connected = false
        setPopupWTmo("Receiver connected!", 100)
    end
    
    doPopup()  -- draw POPUP.active on top of main screen
end


----------------------------------------------------------------------
-- Script OTX Interface
----------------------------------------------------------------------

local function scriptInit()
    local ver, radio, maj, minor, rev, osname = getVersion()
    THEME.isEdgeTx = (osname == 'EdgeTX')

    setupScreen()
    setupColors()
    setupBridge()

    LOAD.is_running = true -- we start the script with this
end


local function scriptRun(event)
    if event == nil then
        error("Cannot be run as a model script!")
        return 2
    end
    if mbridge == nil or not mbridge.enabled() then
        local module0 = model.getModule(0)
        local module1 = model.getModule(1)
        local isInternal = (module0.Type == 5)  -- crsf on Internal
        local isExternal = (module1.Type == 5)  -- crsf on External RF
        
        if not isInternal and not isExternal then
            lcd.clear()
            setPopupBlocked("Enable CRSF\nMDL > Internal/External RF")
            doPopup()
            if event == EVT_VIRTUAL_EXIT then return 2 end
            return 0
        end
    end

    if isConnected == nil or cmdPush == nil or cmdPop == nil then --just to be sure for sure
        error("Unclear issue with mBridge or CRSF!")
        return 2
    end

    if not CURSOR.edit and CURSOR.page == UI.PAGE_MAIN then
        if event == EVT_VIRTUAL_EXIT then
            return 2
        end
    end

    Do(event)

    return 0
end

return { init=scriptInit, run=scriptRun }
