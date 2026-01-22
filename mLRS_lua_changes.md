# Changes in `lua/mLRS.lua` vs `origin/main`

### 1. Global State Refactoring (Major Architectural Change)
The script has moved away from loose global variables to structured state tables.
*   **`UI` Table**: Replaced standalone constants (e.g., `PAGE_MAIN`, `BindPhrase_idx`) with `UI.PAGE_MAIN`, `UI.IDX_BINDPHRASE`, etc.
*   **`CURSOR` Table**: Replaced `cursor_idx`, `page_nr`, `edit`, `param_cnt`, `top_idx`. Now attributes like `CURSOR.idx`, `CURSOR.page` manage navigation.
*   **`CONN` Table**: Replaced `connected`, `has_connected`, `has_disconnected`. Adds `CONN.just_connected` and `CONN.just_disconnected` for event handling.
*   **`LOAD` Table**: Replaced `DEVICE_DOWNLOAD_is_running`, `DEVICE_SAVE_t_last`. Controls the loading state machine (`LOAD.is_running`, `LOAD.complete`, `LOAD.errors`).
*   **`THEME` Table**: Replaced display attributes like `screenSize`, `textColor`, `isEdgeTx`.
*   **`POPUP` Table**: Replaced `popup` (boolean) and `popup_text` with `POPUP.active`, `POPUP.text`, `POPUP.t_end_10ms`.
*   **`DEVICE` Table**: Now includes `DEVICE.request_count` to track timeouts.

### 2. Startup Checks & Safety
*   **CRSF Enable Check**: In `scriptRun()`, added logic to check `model.getModule(0)` and `(1)`. If neither is type 5 (CRSF), it blocks execution and displays: *"Enable CRSF / MDL > Internal/External RF"*.
*   **Script Version**: Updated `VERSION.SCRIPT` to `2026-01-22.00`.

### 3. Connection & Parameter Loading Optimization
*   **Smart "Lazy" Loading**:
    *   **TX-Only Fast Load**: If `DEVICE.INFO.rx_available == 0` (no RX), `LOAD.skip_rx` is set.
    *   **Abort on RX Param**: Loading stops immediately upon encountering "Rx " parameters in skip mode, enabling the TX menu instantly.
    *   **Resume/Late Load**: connecting a receiver later triggers `clearParams(true)`, which preserves TX params and fetches only the missing RX params (starting from `LOAD.rx_first_idx`).
*   **Info Refresh**: Explicitly calls `cmdPush(MB.CMD_REQUEST_INFO, {})` when resuming RX load to ensure receiver name/version are fetched.
*   **Wait Logic Removal**: Removed the startup delay logic (`DEFAULTS.SAVE_DELAY`) from `scriptInit`, allowing immediate operation.
*   **Performance Optimization**: Significantly increased the main loop refresh rate. Reduced the cycle interval from ~330ms to 40ms (idle) and 0ms (during parameter download) to improve UI responsiveness and download speed.

### 4. Robust Retry Logic (New Feature)
A dedicated retry mechanism has been implemented to handle packet loss during parameter loading:
*   **Timeout Monitoring**: `checkAndHandleRetry` monitors `LOAD.request_t_last` to detect stalled requests.
*   **Smart Retries**: `performRetry` re-requests the stalled parameter (current or expected index).
*   **Duplicate Handling**: The loader now detects and silently ignores duplicate parameter indices (`DEBUG.duplicates`) to prevent state corruption if a retry arrives late.
*   **Failure Handling**: `handleRetryLimitExceeded` triggers a soft restart (`clearParams`) or full abort if retries fail repeatedly.

### 5. UI & Navigation Changes
*   **800x480 Support**: `setupScreen()` supports `TX16S MK3` (800x480).
    *   Sets `LAYOUT.TEXT_SIZE = MIDSIZE`.
    *   Increases `page_N` (items per page) to 30.
    *   Adjusts hardcoded coordinates for the larger layout.
*   **Balanced Columns**: Implemented dynamic column balancing for the "Tx" page (and "Rx" page on 800x480 screens). Instead of filling the left column first, items are now distributed evenly between left and right columns based on the total number of visible items.
*   **Virtual Input Handlers**: All `EVT_VIRTUAL_` checks use `CURSOR` and `UI` tables.
*   **Popup Visibility**: Implemented `pending_rx_connected` logic to ensure the "Receiver connected" popup is drawn *after* the main screen refresh, preventing it from being overwritten immediately upon connection.
*   **Focus Helper**: Added `param_focusable(idx)` validation.

### 6. Tools Page Updates
*   **Flash ESP**: Added `UI.IDX_TOOLS_FLASH_ESP` and `sendFlashEsp()` command availability logic (External module + Wifi params).
*   **Debug Stats**: Added `UI.IDX_TOOLS_DEBUG_STATS` to toggle `DEBUG.enabled`, displaying detailed metrics (retry counts, response times) on screen.

### 7. MBridge / Protocol
*   **New Commands**: Added `CMD_SYSTEM_BOOTLOADER` (17) and `CMD_FLASH_ESP` (18).
