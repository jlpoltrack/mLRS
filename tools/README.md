# mLRS - Tools #

This folder includes scripts, libraries, and so on, which help with maintaining and developping the code. 

Normally they should not be needed then working on the existing code base. 


## ELRS Receivers ##

ESP receiver targets are taken from the ELRS targets repo (submodule `tools/elrs/targets`). `elrs_targets.py` resolves a board's layout and overlay into defines for `mLRS/Common/hal/esp/rx-hal-elrs-generic.h`, and runs as PlatformIO pre script for the `rx-elrs-*` envs in `platformio_elrs.ini`.

- `python3 tools/elrs/elrs_targets.py --ini` regenerates `platformio_elrs.ini`, run it after updating the submodule (`run_setup.py` and `run_make_esp_firmwares.py` also do it, the latter also warns if the file changed and needs to be committed)
- `python3 tools/elrs/elrs_targets.py --list` lists the supported boards, `--skipped` the unsupported ones with the reason
- `python3 tools/elrs/elrs_targets.py --show <target>` shows the defines for a board, e.g. `betafpv.rx_900.nano`

Boards with PWM outputs or VTx are skipped. Power levels and chip settings are ELRS's, the default power is always the lowest level.

Env names are `rx-elrs-<vendor>-<device>-<band>-<chip>`, e.g. `rx-elrs-radiomaster-xr4-dual-esp32`. The chip suffix is used by the mLRS Web Flasher. `run_make_esp_firmwares.py -t <text>` builds only the envs containing `<text>`, e.g. `-t elrs` or `-t xr4`.

Flashing with the PlatformIO extension in VS Code:
1. Put the receiver into bootloader mode (hold the button while powering up) and connect it to a USB-UART adapter.
2. Click the environment name in the status bar (Switch PlatformIO Project Environment), type part of the name, e.g. `xr4`, to filter, and select it.
3. Select the serial port in the status bar if more than one is connected.
4. Click Upload.

If the environment isn't listed or an `UnknownEnvNamesError` shows up, the extension still has an old env list. `--ini` touches `platformio.ini` so the extension reloads, otherwise click Refresh in Project Tasks or run Developer: Reload Window.
