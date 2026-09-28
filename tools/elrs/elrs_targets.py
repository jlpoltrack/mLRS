#!/usr/bin/env python
'''
*******************************************************
 Copyright (c) MLRS project
 GPL3
 https://www.gnu.org/licenses/gpl-3.0.de.html
*******************************************************
 elrs_targets.py
 resolves ELRS receiver targets (submodule tools/elrs/targets)
 into defines for mLRS/Common/hal/esp/rx-hal-elrs-generic.h
 as PlatformIO pre script, target from custom_elrs_target in platformio_elrs.ini
 standalone:
   elrs_targets.py --ini       regenerates platformio_elrs.ini, run after updating the submodule
   elrs_targets.py --list
   elrs_targets.py --show betafpv.rx_900.nano
********************************************************
'''
import os
import re
import json
import sys
import argparse


try:
    ELRS_DIR = os.path.dirname(os.path.abspath(__file__))
except NameError: # SCons doesn't set __file__
    ELRS_DIR = os.path.join(os.getcwd(), 'tools', 'elrs')
ELRS_TARGETS_DIR = os.path.join(ELRS_DIR, 'targets')
MLRS_PROJECT_DIR = os.path.normpath(os.path.join(ELRS_DIR, '..', '..'))
PIO_INI = os.path.join(MLRS_PROJECT_DIR, 'platformio_elrs.ini')

PIO_BASE_ENVS = {
    'esp8285': ('env_common_rx_esp82xx', 'esp8285', False),
    'esp32':   ('env_common_rx_esp32', 'pico32', True),
    'esp32c3': ('env_common_rx_esp32c3', 'esp32-c3-devkitm-1', True),
    'esp32s3': ('env_common_rx_esp32s3', 'esp32-s3-devkitc-1', True),
}

# ELRS power levels 10, 25, 50, 100, 250, 500, 1000, 2000 mW, as index
ELRS_POWER_LEVEL_NUM = 8

PLATFORMS = {
    'ESP8285': 'esp8285',
    'ESP32': 'esp32',
    'ESP32C3': 'esp32c3',
    'ESP32S3': 'esp32s3',
}

RADIOS = {
    '900':  { 'chip': 'SX127x', 'lib': 'sx127x.cpp', 'spi_freq': 10000000,
              'bands': ['FREQUENCY_BAND_868_MHZ', 'FREQUENCY_BAND_915_MHZ_FCC'] },
    '2400': { 'chip': 'SX128x', 'lib': 'sx128x.cpp', 'spi_freq': 18000000,
              'bands': ['FREQUENCY_BAND_2P4_GHZ'] },
    # bands depend on which power tables the target has
    'LR1121': { 'chip': 'LR11xx', 'lib': 'lr11xx.cpp', 'spi_freq': 16000000, 'bands': [] },
}
BANDS_LF = ['FREQUENCY_BAND_868_MHZ', 'FREQUENCY_BAND_915_MHZ_FCC']
BANDS_HF = ['FREQUENCY_BAND_2P4_GHZ']

# layout keys which carry no information mLRS uses, battery voltage (vbat*, vsrc*) is ignored too
IGNORED_KEYS = { '//', 'power_high', 'power_lna_gain', 'power_default' }
IGNORED_PREFIXES = ( 'vbat', 'vsrc' )

# features mLRS doesn't support, boards having them are skipped
SKIPPED_FEATURES = ( 'pwm_outputs', 'vtx_nss', 'gyro_type', 'i2c_sda' )


class Unsupported(Exception):
    pass


class Layout:
    '''tracks which keys have been consumed, so unknown keys can be reported'''
    def __init__(self, d):
        self.d = d
        self.used = set()

    def has(self, k):
        return k in self.d

    def get(self, k, default=None):
        self.used.add(k)
        return self.d.get(k, default)

    def req(self, k):
        if k not in self.d:
            raise Unsupported('missing key ' + k)
        return self.get(k)

    def expect(self, k, value):
        if self.has(k) and self.get(k) != value:
            raise Unsupported('%s=%s, only %s supported' % (k, self.d[k], value))

    def unused(self):
        return sorted(k for k in set(self.d.keys()) - self.used - IGNORED_KEYS if not k.startswith(IGNORED_PREFIXES))


def parse_firmware(fw):
    m = re.match(r'Unified_(ESP8285|ESP32|ESP32C3|ESP32S3)_(900|2400|LR1121)_RX$', fw)
    if not m:
        raise Unsupported('firmware ' + fw)
    return PLATFORMS[m.group(1)], m.group(2)


SUBMODULE_MISSING = 'ELRS targets submodule tools/elrs/targets is missing, run run_setup.py or git submodule update --init'


def submodule_missing():
    return not os.path.exists(os.path.join(ELRS_TARGETS_DIR, 'targets.json'))


def load_products():
    '''returns {target_id: product} for all generic esp rx products, target_id = vendor.band_key.dev'''
    if submodule_missing():
        sys.exit(SUBMODULE_MISSING)
    targets = json.load(open(os.path.join(ELRS_TARGETS_DIR, 'targets.json')))
    products = {}
    for vendor, vd in targets.items():
        for band_key, bd in vd.items():
            if not band_key.startswith('rx_') or not isinstance(bd, dict):
                continue
            for dev, dd in bd.items():
                lf = dd.get('layout_file', '')
                if not lf:
                    continue
                try:
                    platform, band = parse_firmware(dd.get('firmware', ''))
                except Unsupported:
                    continue
                products['%s.%s.%s' % (vendor, band_key, dev)] = {
                    'layout': lf, 'overlay': dd.get('overlay') or {},
                    'platform': platform, 'band': band,
                    'name': dd.get('lua_name', ''), 'product_name': dd.get('product_name', ''),
                }
    return products


#-------------------------------------------------------
# resolve into defines
#-------------------------------------------------------

def resolve_radio(L, D, band, n):
    sfx = '' if n == 1 else '_2'
    p = 'ELRS_RADIO_' if n == 1 else 'ELRS_RADIO2_'
    is_sx127x = (band == '900')

    D[p + 'NSS'] = L.req('radio_nss' + sfx)
    D[p + 'RST'] = L.req('radio_rst' + sfx)
    if is_sx127x:
        D[p + 'DIO'] = L.req('radio_dio0' + sfx)
        L.get('radio_dio1' + sfx) # sx127x uses DIO0 only
    else:
        D[p + 'DIO'] = L.req('radio_dio1' + sfx)
        D[p + 'BUSY'] = L.req('radio_busy' + sfx)
    for k, name in (('power_txen', 'TXEN'), ('power_rxen', 'RXEN')):
        if L.has(k + sfx):
            D[p + name] = L.get(k + sfx)
    if n == 1 and L.has('ant_ctrl'):
        # diversity switch not supported by mLRS on ESP, antenna1 only
        D['ELRS_RADIO_ANT'] = L.get('ant_ctrl')
    if n == 1 and band == 'LR1121':
        rfsw = L.get('radio_rfsw_ctrl')
        if rfsw is not None:
            if len(rfsw) != 8:
                raise Unsupported('radio_rfsw_ctrl length %d' % len(rfsw))
            D['ELRS_RFSW_CTRL'] = ','.join(str(v) for v in rfsw)
    if L.get('radio_dcdc', False) and not is_sx127x: # ELRS ignores it for sx127x
        D['SX_USE_REGULATOR_MODE_DCDC' if n == 1 else 'SX2_USE_REGULATOR_MODE_DCDC'] = None


def resolve_power(L, D, band):
    '''uses ELRS calibrated chip settings per ELRS power level'''
    L.expect('power_control', 0)
    pmin = L.req('power_min')
    pmax = L.req('power_max')
    if not (0 <= pmin <= pmax < ELRS_POWER_LEVEL_NUM):
        raise Unsupported('power_min/max')
    if L.has('power_values2'):
        raise Unsupported('power_values2')
    D['ELRS_POWER_MIN'] = pmin
    D['ELRS_POWER_MAX'] = pmax

    def check_len(values):
        if len(values) != pmax - pmin + 1:
            raise Unsupported('power_values length mismatch')
        return values

    if band == 'LR1121':
        # ELRS gives LR1121 power in dBm for sub-GHz (power_values) and 2.4 GHz (power_values_dual)
        lf = L.get('power_values')
        hf = L.get('power_values_dual')
        if lf is None and hf is None:
            raise Unsupported('no power_values')
        lp_pa = L.get('radio_rfo_hf', False)
        if lp_pa:
            D['SX_USE_LP_PA'] = None
        # clamp to the PA ranges, as ELRS does
        lf_range = (-17, 14) if lp_pa else (-9, 22)
        if lf is not None:
            lf = [min(max(v, lf_range[0]), lf_range[1]) for v in check_len(lf)]
        if hf is not None:
            hf = [min(max(v, -18), 13) for v in check_len(hf)]
        # a missing table is never used, the other one is used as filler
        D['ELRS_SX_POWER_LIST_LF'] = ','.join(str(v) for v in (lf or hf))
        D['ELRS_SX_POWER_LIST_HF'] = ','.join(str(v) for v in (hf or lf))
        return (BANDS_LF if lf is not None else []) + (BANDS_HF if hf is not None else [])

    values = check_len(L.req('power_values'))
    if L.has('power_values_dual'):
        raise Unsupported('power_values_dual')

    if band == '900':
        # ELRS writes PaConfig = PA_BOOST | value, with PA_BOOST MaxPower has no effect
        # so only the OutputPower nibble is taken
        if any(v < 0 or v > 0x7F for v in values):
            raise Unsupported('sx127x power values %s' % values)
        if L.get('radio_rfo_hf', False):
            # RFO: Pout = 10.8 + 0.6 * MaxPower - (15 - OutputPower), mLRS uses MaxPower = 1 (11.4 dBm)
            # so shift OutputPower by the MaxPower difference, error is < 0.6 dB
            D['SX_USE_RFO'] = None
            sx_values = [min(max(int((v & 0x0F) + 0.6 * (((v >> 4) & 0x07) - 1) + 0.5 + 16) - 16, 0), 15) for v in values]
        else:
            sx_values = [v & 0x0F for v in values]
    else:
        # ELRS gives SX1280 power in dBm (-18 .. 13), mLRS sx_power = dBm + 18
        sx_values = [v + 18 for v in values]

    D['ELRS_SX_POWER_LIST'] = ','.join(str(v) for v in sx_values)
    return RADIOS[band]['bands']


def resolve(product):
    '''returns list of (define, value), value None for flag defines'''
    layout = json.load(open(os.path.join(ELRS_TARGETS_DIR, 'RX', product['layout'])))
    # overlay values of -1 or null remove the layout key
    merged = { k: v for k, v in dict(layout, **product['overlay']).items() if v is not None and v != -1 }
    L = Layout(merged)
    platform, band = product['platform'], product['band']
    D = {}

    for k in SKIPPED_FEATURES:
        if L.has(k):
            raise Unsupported(k)
    name = product['name']
    if not name or len(name) > 20:
        raise Unsupported('lua_name "%s"' % name)

    D['RX_ELRS_TARGET'] = None
    D['DEVICE_NAME'] = '"%s"' % name
    D['DEVICE_IS_RECEIVER'] = None
    D['DEVICE_HAS_' + RADIOS[band]['chip']] = None

    # serial
    if platform == 'esp8285':
        # ESP8285 Serial is fixed to UART0 pins, SPI to HSPI pins
        L.expect('serial_rx', 3); L.expect('serial_tx', 1)
        L.expect('radio_miso', 12); L.expect('radio_mosi', 13); L.expect('radio_sck', 14)
    else:
        tx, rx = L.req('serial_tx'), L.req('serial_rx')
        if platform != 'esp32' or (tx, rx) != (1, 3):
            D['ELRS_SERIAL_TX'] = tx
            D['ELRS_SERIAL_RX'] = rx
        D['ELRS_SPI_MISO'] = L.req('radio_miso')
        D['ELRS_SPI_MOSI'] = L.req('radio_mosi')
        D['ELRS_SPI_SCK'] = L.req('radio_sck')
    if L.has('serial1_tx'):
        if platform == 'esp8285':
            raise Unsupported('serial1 on esp8285')
        D['ELRS_OUT_TX'] = L.get('serial1_tx')
        L.get('serial1_rx') # out port is tx only
    elif L.has('serial1_rx'):
        raise Unsupported('serial1_rx without serial1_tx')

    # radios
    D['ELRS_SPI_FREQUENCY'] = RADIOS[band]['spi_freq']
    if band == 'LR1121' and platform == 'esp8285':
        raise Unsupported('lr1121 on esp8285')
    if platform == 'esp8285' and band == '2400':
        D['ELRS_SPI_FREQUENCY'] = 16000000
    resolve_radio(L, D, band, 1)
    if L.has('radio_nss_2'):
        if platform == 'esp8285':
            raise Unsupported('true diversity on esp8285')
        resolve_radio(L, D, band, 2)
    if L.has('ant_ctrl_2') or L.has('power_apc2'):
        raise Unsupported('ant_ctrl_2/power_apc2')

    # button, leds
    if L.has('button'):
        D['ELRS_BUTTON'] = L.get('button')
    if L.has('led_rgb') and L.has('led'):
        raise Unsupported('both led and led_rgb')
    if L.has('led_rgb'):
        if platform == 'esp8285':
            raise Unsupported('led_rgb on esp8285')
        L.expect('led_rgb_isgrb', True)
        L.expect('ledidx_rgb_status', [0])
        L.expect('ledidx_rgb_boot', [0])
        D['ELRS_LED_RGB'] = L.get('led_rgb')
    elif L.has('led'):
        for k in ('led_rgb_isgrb', 'ledidx_rgb_status', 'ledidx_rgb_boot'):
            L.get(k) # leftovers when an overlay replaces led_rgb by led
        D['ELRS_LED'] = L.get('led')
        if L.get('led_red_invert', False):
            D['ELRS_LED_INVERTED'] = None
    else:
        raise Unsupported('no led')

    for b in resolve_power(L, D, band):
        D[b] = None

    unused = L.unused()
    if unused:
        raise Unsupported('unhandled keys ' + ', '.join(unused))
    return list(D.items())


def env_name(tid, product):
    # chipset suffix is used by the web flasher
    vendor, band_key, dev = tid.split('.')
    return 'rx-elrs-' + re.sub(r'[^a-z0-9]+', '-', '%s-%s-%s' % (vendor, dev, band_key[3:])) + '-' + product['platform']


def write_pio_ini(products):
    ini = [
        '; auto-generated by tools/elrs/elrs_targets.py --ini, do not edit',
        '; ELRS receivers, configured from the ELRS targets repo (tools/elrs/targets)',
        '',
    ]
    for platform, (base, board, rgb) in PIO_BASE_ENVS.items():
        ini += ['[env_elrs_rx_%s]' % platform, 'extends = ' + base, 'board = ' + board]
        if rgb:
            ini += ['lib_deps =', '  makuna/NeoPixelBus@^2.8.3']
        ini += ['extra_scripts = pre:tools/elrs/elrs_targets.py', '']
    ini.append('')
    names = set()
    for tid, p in sorted(products.items()):
        try:
            resolve(p)
        except Unsupported:
            continue
        name = env_name(tid, p)
        if name in names:
            raise Exception('duplicate env name ' + name)
        names.add(name)
        ini += ['[env:%s]' % name, 'extends = env_elrs_rx_' + p['platform'], 'custom_elrs_target = ' + tid, '']
    with open(PIO_INI, 'w') as F:
        F.write('\n'.join(ini))
    # the VS Code extension only reloads envs when platformio.ini changes
    os.utime(os.path.join(MLRS_PROJECT_DIR, 'platformio.ini'))
    print('wrote %d envs to %s' % (len(names), os.path.normpath(PIO_INI)))


#-------------------------------------------------------
# main
#-------------------------------------------------------

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ini', action='store_true', help='regenerate platformio_elrs.ini')
    parser.add_argument('--list', action='store_true', help='list supported targets')
    parser.add_argument('--skipped', action='store_true', help='list skipped targets with reason')
    parser.add_argument('--show', metavar='TARGET', help='show defines for a target')
    args = parser.parse_args()

    products = load_products()
    if args.ini:
        write_pio_ini(products)
        return
    if args.show:
        p = products[args.show]
        print('# %s, ELRS layout: %s%s, env: %s' % (
              p['product_name'], p['layout'], ' + overlay' if p['overlay'] else '', env_name(args.show, p)))
        for k, v in resolve(p):
            print('-D %s' % k if v is None else '-D %s=%s' % (k, v))
        return
    for tid, p in sorted(products.items()):
        try:
            resolve(p)
            if args.list:
                print('%-45s %-45s "%s"' % (tid, env_name(tid, p), p['name']))
        except Unsupported as ex:
            if args.skipped:
                print('%-45s %s' % (tid, ex))


def pio_pre_script(env):
    tid = env.GetProjectOption('custom_elrs_target', '')
    if submodule_missing():
        sys.stderr.write(SUBMODULE_MISSING + '\n')
        env.Exit(1)
    products = load_products()
    if tid not in products:
        sys.stderr.write('ELRS target "%s" not found, run tools/elrs/elrs_targets.py --ini\n' % tid)
        env.Exit(1)
    p = products[tid]
    try:
        defines = resolve(p)
    except Unsupported as ex:
        sys.stderr.write('ELRS target "%s" not supported: %s\n' % (tid, ex))
        env.Exit(1)
    print('ELRS target %s: %s' % (tid, p['product_name']))
    env.Append(CPPDEFINES=[k if v is None else (k, env.StringifyMacro(v[1:-1]) if k == 'DEVICE_NAME' else v) for k, v in defines])
    env.Append(SRC_FILTER=['+<modules/sx12xx-lib/src/%s>' % RADIOS[p['band']]['lib']])


if 'Import' in globals(): # run as PlatformIO extra script
    Import('env') # noqa: F821
    pio_pre_script(env) # noqa: F821
elif __name__ == '__main__':
    main()
