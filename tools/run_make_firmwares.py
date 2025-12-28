#!/usr/bin/env python
'''
*******************************************************
 Copyright (c) MLRS project
 GPL3
 https://www.gnu.org/licenses/gpl-3.0.de.html
 OlliW @ www.olliw.eu
*******************************************************
 run_make_firmwares.py
 3rd version, doesn't use make but calls gnu directly
 gave up on cmake, hence naive by hand
 version 02.01.2026
********************************************************

COMMAND LINE ARGUMENTS:
  --target, -t, -T <target_name>
      build only the specified target (or targets matching the pattern).
      pattern matches against both the base target name and the full build name
      (which includes the appendix like -default, -siktelem, -can, etc.).
      use '!' prefix to exclude targets (e.g., '!rx-' to exclude all RX targets).
      if omitted, all targets are built.
      note: matching is case-sensitive. all target names use lowercase (rx-, tx-).
      examples: 
        --target rx-matek-mr900-22
        --target tx-
        --target '!rx-'
        --target tx-matek-mr24-30-g431kb-default    # Selects only the -default variant
        --target tx-matek-mr24-30-g431kb-siktelem   # Selects only the -siktelem variant

  --define, -d, -D <DEFINE>
      add a preprocessor define to the build (can be specified multiple times).
      each occurrence adds another define to the compilation.
      example: --define DEBUG_MODE --define ENABLE_FEATURE_X

  --no-pause, --nopause, -np
      skip the "Press Enter to continue..." prompt at the end of execution.
      useful for automated builds or CI/CD pipelines.

  --version, -v, -V <version_string>
      override the version string from common_conf.h.
      if omitted, the version is read from VERSIONONLYSTR in common_conf.h.
      example: --version 1.2.3

  --flash, -f, -F
      automatically detect and flash via DFU or SWD after successful build.
      tries SWD first (faster detection), falls back to DFU if not found.
      requires exactly one target to be built.
      uses STM32CubeProgrammer CLI for flashing.

  --list-targets, -lt, -LT
      list all available STM32 targets and exit without building.
      prints target names from TLIST including all variants (default, siktelem, can, etc.).
      use this to discover correct target names for --target flag.

  --sequential-files, -sf, -SF
      disable parallel file compilation within each target.
      by default, files are compiled in parallel using all available CPU cores.
      use this flag to compile files sequentially (slower, useful for debugging).

  --sequential-targets, -st, -ST
      build targets one at a time instead of in parallel.
      by default, when building multiple targets (>1), they are built in parallel.
      use this flag to build targets sequentially.

  --no-clean, -nc
      preserve build artifacts for incremental builds.
      by default, the build directory is cleaned before building.
      use this flag for faster rebuilds during development.

USAGE EXAMPLES:
  # build all targets with default settings (parallel files + parallel targets)
  python run_make_firmwares.py

  # build and flash a specific target with auto-detection (recommended)
  python run_make_firmwares.py --target rx-matek-mr900-22 --flash

  # build all TX targets with a custom define
  python run_make_firmwares.py --target tx- --define CUSTOM_FEATURE

  # build with sequential file compilation (for debugging)
  python run_make_firmwares.py --sequential-files --nopause

********************************************************
'''
import os
import pathlib
import shutil
import re
import sys
import subprocess
import multiprocessing
from concurrent.futures import ThreadPoolExecutor, as_completed
import time
import hashlib

# global thread pool for all compilation jobs across all targets
# using 2x cpu_count allows I/O overlap while preventing excessive context switching
# this single pool replaces nested thread pools + semaphore for cleaner architecture
GLOBAL_COMPILE_POOL = None  # Initialized in main when __name__ == "__main__"
COMPILE_POOL_SIZE = multiprocessing.cpu_count() * 2


#-- installation dependent
# effort at finding this automatically

#ST_DIR = os.path.join("C:/",'ST','STM32CubeIDE','STM32CubeIDE','plugins')
#GNU_DIR = 'com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.10.3-2021.10.win32_1.0.0.202111181127'

def findSTM32CubeIDEGnuTools(search_root, is_macos_app=False):
    st_dir = ''
    st_cubeide_dir = ''
    st_cubeide_ver_nr = 0

    # for macOS .app bundle, go directly to plugins
    if is_macos_app:
        st_dir = os.path.join(search_root, 'Contents', 'Eclipse', 'plugins')
        if not os.path.exists(st_dir):
            print(search_root, 'plugins not found!')
            return '', ''
    else:
        try:
            for file in os.listdir(search_root):
                if 'stm32cubeide' in file.lower(): # makes it work on both win and linux
                    if '_' in file:
                        ver = file[13:].split('.')
                        ver_nr = int(ver[0])*10000 + int(ver[1])*100 + int(ver[2])
                        if ver_nr > st_cubeide_ver_nr:
                            st_cubeide_ver_nr = ver_nr
                            st_cubeide_dir = file
                    else:
                        st_cubeide_dir = file
                        st_cubeide_ver_nr = 0
        except:
            print(search_root,'not found!')
            return '', ''
        if st_cubeide_dir != '':
            if sys.platform == 'linux': # install paths are os dependent
                st_dir = os.path.join(search_root,st_cubeide_dir,'plugins')
            else:
                st_dir = os.path.join(search_root,st_cubeide_dir,'stm32cubeide','plugins')
        else:
            print('STM32CubeIDE not found!')
            return '', ''

    # determine OS-specific gnu-tools directory suffix
    gnu_dir_os_name = 'win32'
    if sys.platform == 'darwin':
        gnu_dir_os_name = 'macos64'
    elif sys.platform == 'linux':
        gnu_dir_os_name = 'linux'

    gnu_dir = ''
    ver_nr = 0
    try:
        for dirpath in os.listdir(st_dir):
            if 'mcu.externaltools.gnu-tools-for-stm32' in dirpath and gnu_dir_os_name in dirpath:
                # the numbers after the string 'gnu-tools-for-stm32' contains the gnutools ver number, like .11.3
                gnuver = int(dirpath.split('gnu-tools-for-stm32',1)[1][1:3])
                if gnuver >= 12:
                    print("WARNING: gnu-tools ver >= 12 found but skipped")
                    continue
                # the string after the last . contains a datum plus some other number
                ver = int(dirpath[dirpath.rindex('.')+1:])
                if ver > ver_nr:
                    ver_nr = ver
                    gnu_dir = dirpath
    except:
        print('STM32CubeIDE not found!')
        return '', ''

    return st_dir, gnu_dir


ST_DIR,GNU_DIR = '', ''

# do this only when called from main context
if __name__ == "__main__":
    st_root = os.path.join("C:/",'ST')
    if sys.platform == 'darwin':  # macOS
        # check for STM32CubeIDE.app in /Applications
        macos_app_path = '/Applications/STM32CubeIDE.app'
        if os.path.exists(macos_app_path):
            ST_DIR, GNU_DIR = findSTM32CubeIDEGnuTools(macos_app_path, is_macos_app=True)
        else:
            print(macos_app_path, 'not found!')
    elif sys.platform == 'linux':
        st_root = os.path.join("/opt",'st')
        ST_DIR,GNU_DIR = findSTM32CubeIDEGnuTools(st_root)
    else:  # Windows
        ST_DIR,GNU_DIR = findSTM32CubeIDEGnuTools(st_root)
    if os.getenv("MLRS_ST_DIR"):
        ST_DIR = os.getenv("MLRS_ST_DIR")
    if os.getenv("MLRS_GNU_DIR"):
        GNU_DIR = os.getenv("MLRS_GNU_DIR")

    if ST_DIR == '' or GNU_DIR == '' or not os.path.exists(os.path.join(ST_DIR,GNU_DIR)):
        print('ERROR: gnu-tools not found!')
        exit(1)

    print('STM32CubeIDE found in:', ST_DIR)
    print('gnu-tools found in:', GNU_DIR)
    print('------------------------------------------------------------')


#-- GCC preliminaries

GCC_DIR = os.path.join(ST_DIR,GNU_DIR,'tools','bin')

# we need to modify the PATH so that the correct toolchain/compiler is used
# why does sys.path.insert(0,xxx) not work?
# no, not needed anymore as we can call arm-none-eabi directly
#envpath = os.environ["PATH"]
#envpath = GCC_DIR + ';' + envpath
#os.environ["PATH"] = envpath


#-- mLRS directories

MLRS_PROJECT_DIR = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

MLRS_DIR = os.path.join(MLRS_PROJECT_DIR,'mLRS')

MLRS_TOOLS_DIR = os.path.join(MLRS_PROJECT_DIR,'tools')
MLRS_BUILD_DIR = os.path.join(MLRS_PROJECT_DIR,'tools','build')


#-- current version and branch

VERSIONONLYSTR = ''
BRANCHSTR = ''
HASHSTR = ''

def mlrs_set_version():
    global VERSIONONLYSTR
    F = open(os.path.join(MLRS_DIR,'Common','common_conf.h'), mode='r')
    content = F.read()
    F.close()

    if VERSIONONLYSTR != '':
        print('VERSIONONLYSTR =', VERSIONONLYSTR)
        return

    v = re.search(r'VERSIONONLYSTR\s+"(\S+)"', content)
    if v:
        VERSIONONLYSTR = v.groups()[0]
        print('VERSIONONLYSTR =', VERSIONONLYSTR)
    else:
        print('----------------------------------------')
        print('ERROR: VERSIONONLYSTR not found')
        os.system('pause')
        exit()


def mlrs_set_branch_hash(version_str):
    global BRANCHSTR
    global HASHSTR

    git_branch = subprocess.getoutput("git branch --show-current")
    if not git_branch == 'main':
        BRANCHSTR = '-'+git_branch
    if BRANCHSTR != '':
        print('BRANCHSTR =', BRANCHSTR)

    git_hash = subprocess.getoutput("git rev-parse --short HEAD")
    v_patch = int(version_str.split('.')[2])
    if v_patch % 2 == 1: # odd firmware patch version, so is dev, so add git hash
        HASHSTR = '-@'+git_hash
    if HASHSTR != '':
        print('HASHSTR =', HASHSTR)


#-- helper

def remake_dir(path):
    if os.path.exists(path):
        shutil.rmtree(path, ignore_errors=True)

def make_dir(path):
    os.makedirs(path, exist_ok=True)

def create_dir(path):
    os.makedirs(path, exist_ok=True)

def erase_dir(path):
    if os.path.exists(path):
        shutil.rmtree(path, ignore_errors=True)

def create_clean_dir(path):
    if os.path.exists(path):
        shutil.rmtree(path, ignore_errors=True)
    os.makedirs(path, exist_ok=True)


def printWarning(txt):
    print('\033[93m'+txt+'\033[0m') # light Yellow


def printError(txt):
    print('\033[91m'+txt+'\033[0m') # light Red


def validate_arguments():
    """Validate command-line arguments and suggest corrections for typos"""
    
    # Define all valid flags (both long and short forms)
    valid_flags = {
        '--target', '-t', '-T',
        '--define', '-d', '-D',
        '--no-pause', '--nopause', '-np',
        '--version', '-v', '-V',
        '--flash', '-f', '-F',
        '--list-targets', '-lt', '-LT',
        '--sequential-files', '-sf', '-SF',
        '--sequential-targets', '-st', '-ST',
        '--no-clean', '-nc',
    }
    
    # Flags that expect a value argument
    value_flags = {
        '--target', '-t', '-T',
        '--define', '-d', '-D',
        '--version', '-v', '-V',
    }
    
    # Track which arguments are expected values (not flags)
    skip_next = False
    unknown_args = []
    
    for i, arg in enumerate(sys.argv[1:], 1):  # Skip script name
        if skip_next:
            skip_next = False
            continue
            
        # Check if this looks like a flag
        if arg.startswith('-'):
            if arg not in valid_flags:
                unknown_args.append((i, arg))
            elif arg in value_flags:
                skip_next = True  # Next arg is the value
    
    if unknown_args:
        printError('Error: Unrecognized command-line argument(s):')
        for pos, arg in unknown_args:
            printError(f'  {arg}')
        
        print()
        printError('See script header for valid arguments and usage examples')
        sys.exit(1)


#-- incremental build support

def parse_dependency_file(dep_file_path):
    """Parse a GCC .d dependency file and return list of dependencies.
    
    Format: target: dep1 dep2 \
                    dep3 dep4
    
    Also handles phony targets added by -MP flag (lines like "header.h:")
    """
    if not os.path.exists(dep_file_path):
        return None
    
    try:
        with open(dep_file_path, 'r') as f:
            content = f.read()
        
        # remove line continuations
        content = content.replace('\\\n', ' ')
        
        # find the colon separating target from dependencies
        colon_pos = content.find(':')
        if colon_pos == -1:
            return None
        
        # extract dependencies (everything after the colon)
        deps_str = content[colon_pos + 1:]
        
        # split on whitespace, filter empty strings
        deps = []
        for d in deps_str.split():
            d = d.strip()
            if not d:
                continue
            # GCC -MP adds phony targets like "header.h:" - strip trailing colon
            if d.endswith(':'):
                continue  # skip phony target lines entirely
            deps.append(d)
        
        return deps
    except:
        return None


def compute_flags_hash(cmd_args):
    """Compute a hash of compiler flags for detecting flag changes."""
    # join all args and hash
    flags_str = ' '.join(sorted(cmd_args))
    return hashlib.md5(flags_str.encode()).hexdigest()[:16]


def needs_recompile(source_path, object_path, dep_file_path, flags_file_path, current_flags_hash):
    """Check if a source file needs recompilation.
    
    Returns True if:
    - Object file doesn't exist
    - Dependency file doesn't exist (can't check deps, must rebuild)
    - Flags have changed
    - Source file is newer than object
    - Any header dependency is newer than object
    - Any dependency file is missing (deleted header)
    """
    # no object file -> must compile
    if not os.path.exists(object_path):
        return True
    
    # check if flags changed
    if os.path.exists(flags_file_path):
        try:
            with open(flags_file_path, 'r') as f:
                stored_hash = f.read().strip()
            if stored_hash != current_flags_hash:
                return True
        except:
            return True
    else:
        # no flags file -> must compile (first time or flags file deleted)
        return True
    
    # parse dependency file
    deps = parse_dependency_file(dep_file_path)
    if deps is None:
        # no valid dep file -> must compile
        return True
    
    obj_mtime = os.path.getmtime(object_path)
    
    # check each dependency
    for dep in deps:
        if not os.path.exists(dep):
            # dependency was deleted -> must recompile (will fail if truly missing)
            return True
        if os.path.getmtime(dep) > obj_mtime:
            # dependency is newer than object -> must recompile
            return True
    
    # all checks passed -> no recompilation needed
    return False


def save_flags_hash(flags_file_path, flags_hash):
    """Save the flags hash to a file."""
    try:
        os.makedirs(os.path.dirname(flags_file_path), exist_ok=True)
        with open(flags_file_path, 'w') as f:
            f.write(flags_hash)
    except:
        pass  # non-fatal, will just recompile next time


#--------------------------------------------------
# build system
#--------------------------------------------------

#-- source & include files, HAL, CubeMX, target independ

MLRS_SOURCES_HAL_STM32F1 = [
    os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_hal.c'),
    os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_hal_cortex.c'),
    os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_hal_can.c'),
    os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_hal_dma.c'),
    os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_hal_flash.c'),
    os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_hal_flash_ex.c'),
    os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_hal_i2c.c'),
    os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_hal_pwr.c'),
    os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_hal_rcc.c'),
    os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_hal_rcc_ex.c'),
    os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_ll_adc.c'),
    os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_ll_crc.c'),
    os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_ll_dac.c'),
    os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_ll_dma.c'),
    os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_ll_exti.c'),
    os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_ll_fsmc.c'),
    os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_ll_gpio.c'),
    os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_ll_i2c.c'),
    os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_ll_pwr.c'),
    os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_ll_rcc.c'),
    os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_ll_rtc.c'),
    os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_ll_sdmmc.c'),
    os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_ll_spi.c'),
    os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_ll_tim.c'),
    os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_ll_usart.c'),
    os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_ll_usb.c'),
    os.path.join('Drivers','STM32F1xx_HAL_Driver','Src','stm32f1xx_ll_utils.c'),
    ]

MLRS_SOURCES_HAL_STM32G4 = [
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_hal.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_hal_cortex.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_hal_dma.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_hal_dma_ex.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_hal_fdcan.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_hal_pcd.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_hal_pcd_ex.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_hal_flash.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_hal_flash_ex.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_hal_i2c.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_hal_i2c_ex.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_hal_pwr.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_hal_pwr_ex.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_hal_rcc.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_hal_rcc_ex.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_adc.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_comp.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_cordic.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_crc.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_crs.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_dac.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_dma.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_exti.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_fmac.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_fmc.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_gpio.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_hrtim.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_i2c.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_lptim.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_lpuart.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_opamp.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_pwr.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_rcc.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_rng.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_rtc.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_spi.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_tim.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_ucpd.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_usart.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_usb.c'),
    os.path.join('Drivers','STM32G4xx_HAL_Driver','Src','stm32g4xx_ll_utils.c'),
    ]

MLRS_SOURCES_HAL_STM32WL = [
    os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_hal.c'),
    os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_hal_cortex.c'),
    os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_hal_dma.c'),
    os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_hal_dma_ex.c'),
    os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_hal_flash.c'),
    os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_hal_flash_ex.c'),
    os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_hal_i2c.c'),
    os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_hal_i2c_ex.c'),
    os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_hal_pwr.c'),
    os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_hal_pwr_ex.c'),
    os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_hal_rcc.c'),
    os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_hal_rcc_ex.c'),
    os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_ll_adc.c'),
    os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_ll_comp.c'),
    os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_ll_crc.c'),
    os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_ll_dac.c'),
    os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_ll_dma.c'),
    os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_ll_exti.c'),
    os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_ll_gpio.c'),
    os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_ll_i2c.c'),
    os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_ll_lptim.c'),
    os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_ll_lpuart.c'),
    os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_ll_pka.c'),
    os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_ll_pwr.c'),
    os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_ll_rcc.c'),
    os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_ll_rng.c'),
    os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_ll_rtc.c'),
    os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_ll_spi.c'),
    os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_ll_tim.c'),
    os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_ll_usart.c'),
    os.path.join('Drivers','STM32WLxx_HAL_Driver','Src','stm32wlxx_ll_utils.c'),
    ]

MLRS_SOURCES_HAL_STM32L4 = [
    os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_hal.c'),
    os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_hal_cortex.c'),
    os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_hal_flash.c'),
    os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_hal_flash_ex.c'),
    os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_hal_i2c.c'),
    os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_hal_pwr.c'),
    os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_hal_pwr_ex.c'),
    os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_hal_rcc.c'),
    os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_hal_rcc_ex.c'),
    os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_adc.c'),
    os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_comp.c'),
    os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_crc.c'),
    os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_crs.c'),
    os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_dac.c'),
    os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_dma.c'),
    os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_dma2d.c'),
    os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_exti.c'),
    os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_fmc.c'),
    os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_gpio.c'),
    os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_i2c.c'),
    os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_lptim.c'),
    os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_lpuart.c'),
    os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_opamp.c'),
    os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_pka.c'),
    os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_pwr.c'),
    os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_rcc.c'),
    os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_rng.c'),
    os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_rtc.c'),
    os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_sdmmc.c'),
    os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_spi.c'),
    os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_swpmi.c'),
    os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_tim.c'),
    os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_usart.c'),
    os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_usb.c'),
    os.path.join('Drivers','STM32L4xx_HAL_Driver','Src','stm32l4xx_ll_utils.c'),
    ]

MLRS_SOURCES_HAL_STM32F0 = [
    os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_hal.c'),
    os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_hal_cortex.c'),
    os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_hal_dma.c'),
    os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_hal_flash.c'),
    os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_hal_flash_ex.c'),
    os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_hal_i2c.c'),
    os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_hal_i2c_ex.c'),
    os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_hal_pwr.c'),
    os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_hal_pwr_ex.c'),
    os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_hal_rcc.c'),
    os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_hal_rcc_ex.c'),
    os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_hal_pcd.c'),
    os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_hal_pcd_ex.c'),
    os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_ll_adc.c'),
    os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_ll_comp.c'),
    os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_ll_crc.c'),
    os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_ll_crs.c'),
    os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_ll_dac.c'),
    os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_ll_dma.c'),
    os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_ll_exti.c'),
    os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_ll_gpio.c'),
    os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_ll_i2c.c'),
    os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_ll_pwr.c'),
    os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_ll_rcc.c'),
    os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_ll_rtc.c'),
    os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_ll_spi.c'),
    os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_ll_tim.c'),
    os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_ll_usart.c'),
    os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_ll_usb.c'),
    os.path.join('Drivers','STM32F0xx_HAL_Driver','Src','stm32f0xx_ll_utils.c'),
    ]

MLRS_SOURCES_HAL_STM32F3 = [
    os.path.join('Drivers','STM32F3xx_HAL_Driver','Src','stm32f3xx_hal.c'),
    os.path.join('Drivers','STM32F3xx_HAL_Driver','Src','stm32f3xx_hal_cortex.c'),
    os.path.join('Drivers','STM32F3xx_HAL_Driver','Src','stm32f3xx_hal_dma.c'),
    os.path.join('Drivers','STM32F3xx_HAL_Driver','Src','stm32f3xx_hal_flash.c'),
    os.path.join('Drivers','STM32F3xx_HAL_Driver','Src','stm32f3xx_hal_flash_ex.c'),
    os.path.join('Drivers','STM32F3xx_HAL_Driver','Src','stm32f3xx_hal_i2c.c'),
    os.path.join('Drivers','STM32F3xx_HAL_Driver','Src','stm32f3xx_hal_i2c_ex.c'),
    os.path.join('Drivers','STM32F3xx_HAL_Driver','Src','stm32f3xx_hal_pwr.c'),
    os.path.join('Drivers','STM32F3xx_HAL_Driver','Src','stm32f3xx_hal_pwr_ex.c'),
    os.path.join('Drivers','STM32F3xx_HAL_Driver','Src','stm32f3xx_hal_rcc.c'),
    os.path.join('Drivers','STM32F3xx_HAL_Driver','Src','stm32f3xx_hal_rcc_ex.c'),
    os.path.join('Drivers','STM32F3xx_HAL_Driver','Src','stm32f3xx_hal_pcd.c'),
    os.path.join('Drivers','STM32F3xx_HAL_Driver','Src','stm32f3xx_hal_pcd_ex.c'),
    os.path.join('Drivers','STM32F3xx_HAL_Driver','Src','stm32f3xx_ll_adc.c'),
    os.path.join('Drivers','STM32F3xx_HAL_Driver','Src','stm32f3xx_ll_comp.c'),
    os.path.join('Drivers','STM32F3xx_HAL_Driver','Src','stm32f3xx_ll_crc.c'),
    os.path.join('Drivers','STM32F3xx_HAL_Driver','Src','stm32f3xx_ll_dac.c'),
    os.path.join('Drivers','STM32F3xx_HAL_Driver','Src','stm32f3xx_ll_dma.c'),
    os.path.join('Drivers','STM32F3xx_HAL_Driver','Src','stm32f3xx_ll_exti.c'),
    os.path.join('Drivers','STM32F3xx_HAL_Driver','Src','stm32f3xx_ll_fmc.c'),
    os.path.join('Drivers','STM32F3xx_HAL_Driver','Src','stm32f3xx_ll_gpio.c'),
    os.path.join('Drivers','STM32F3xx_HAL_Driver','Src','stm32f3xx_ll_hrtim.c'),
    os.path.join('Drivers','STM32F3xx_HAL_Driver','Src','stm32f3xx_ll_i2c.c'),
    os.path.join('Drivers','STM32F3xx_HAL_Driver','Src','stm32f3xx_ll_opamp.c'),
    os.path.join('Drivers','STM32F3xx_HAL_Driver','Src','stm32f3xx_ll_pwr.c'),
    os.path.join('Drivers','STM32F3xx_HAL_Driver','Src','stm32f3xx_ll_rcc.c'),
    os.path.join('Drivers','STM32F3xx_HAL_Driver','Src','stm32f3xx_ll_rtc.c'),
    os.path.join('Drivers','STM32F3xx_HAL_Driver','Src','stm32f3xx_ll_spi.c'),
    os.path.join('Drivers','STM32F3xx_HAL_Driver','Src','stm32f3xx_ll_tim.c'),
    os.path.join('Drivers','STM32F3xx_HAL_Driver','Src','stm32f3xx_ll_usart.c'),
    os.path.join('Drivers','STM32F3xx_HAL_Driver','Src','stm32f3xx_ll_usb.c'),
    os.path.join('Drivers','STM32F3xx_HAL_Driver','Src','stm32f3xx_ll_utils.c'),
    ]

MLRS_SOURCES_CORE = [ # the ?? are going to be replaced with mcu_family label, f1, g4, wl, l4
    os.path.join('Core','Src','main.cpp'),
    os.path.join('Core','Src','stm32??xx_hal_msp.c'),
    os.path.join('Core','Src','stm32??xx_it.c'),
    os.path.join('Core','Src','syscalls.c'),
    os.path.join('Core','Src','sysmem.c'),
    os.path.join('Core','Src','system_stm32??xx.c'),
    ]

MLRS_INCLUDES = [ # the ?? are going to be replaced with mcu_HAL label, STM32F1xx, STM32G4xx, STM32WLxx, STM32L4xx
    os.path.join('Core','Inc'),
    os.path.join('Drivers','??_HAL_Driver','Inc'),
    os.path.join('Drivers','??_HAL_Driver','Inc','Legacy'),
    os.path.join('Drivers','CMSIS','Device','ST','??','Include'),
    os.path.join('Drivers','CMSIS','Include'),
    ]


#-- source & include files, target independent/common

MLRS_SOURCES_MODULES = [
    os.path.join('modules','stm32-dronecan-lib','libcanard','canard.c'),
    os.path.join('modules','stm32-dronecan-lib','stm32-dronecan-driver-f1.c'),
    os.path.join('modules','stm32-dronecan-lib','stm32-dronecan-driver-g4.c'),
    os.path.join('modules','sx12xx-lib','src','sx126x.cpp'),
    os.path.join('modules','sx12xx-lib','src','sx127x.cpp'),
    os.path.join('modules','sx12xx-lib','src','sx128x.cpp'),
    os.path.join('modules','sx12xx-lib','src','lr11xx.cpp'),
    os.path.join('modules','stm32ll-lib','src','stdstm32.c'),
    ]

MLRS_SOURCES_COMMON = [
    os.path.join('Common','thirdparty','gdisp.c'),
    os.path.join('Common','thirdparty','thirdparty.cpp'),
    os.path.join('Common','libs','filters.cpp'),
    os.path.join('Common','channel_order.cpp'),
    os.path.join('Common','common_stats.cpp'),
    os.path.join('Common','common_types.cpp'),
    os.path.join('Common','setup_types.cpp'),
    os.path.join('Common','diversity.cpp'),
    os.path.join('Common','fhss.cpp'),
    os.path.join('Common','link_types.cpp'),
    os.path.join('Common','lq_counter.cpp'),
    os.path.join('Common','while.cpp'),
    os.path.join('Common','tasks.cpp'),
    ]

#add Common/dronecan/out/src/*.c if they exists # TODO: add a function to include them all
MLRS_SOURCES_COMMON.append(os.path.join('Common','dronecan','out','src','dronecan.sensors.rc.RCInput.c'))
MLRS_SOURCES_COMMON.append(os.path.join('Common','dronecan','out','src','uavcan.protocol.dynamic_node_id.Allocation.c'))
MLRS_SOURCES_COMMON.append(os.path.join('Common','dronecan','out','src','uavcan.protocol.GetNodeInfo_req.c'))
MLRS_SOURCES_COMMON.append(os.path.join('Common','dronecan','out','src','uavcan.protocol.GetNodeInfo_res.c'))
MLRS_SOURCES_COMMON.append(os.path.join('Common','dronecan','out','src','uavcan.protocol.HardwareVersion.c'))
MLRS_SOURCES_COMMON.append(os.path.join('Common','dronecan','out','src','uavcan.protocol.NodeStatus.c'))
MLRS_SOURCES_COMMON.append(os.path.join('Common','dronecan','out','src','uavcan.protocol.SoftwareVersion.c'))
MLRS_SOURCES_COMMON.append(os.path.join('Common','dronecan','out','src','uavcan.tunnel.Protocol.c'))
MLRS_SOURCES_COMMON.append(os.path.join('Common','dronecan','out','src','uavcan.tunnel.Targetted.c'))
MLRS_SOURCES_COMMON.append(os.path.join('Common','dronecan','out','src','dronecan.protocol.FlexDebug.c'))

MLRS_SOURCES_RX = [
    os.path.join('CommonRx','mlrs-rx.cpp'),
    os.path.join('CommonRx','out.cpp'),
    ]

MLRS_SOURCES_TX = [
    os.path.join('CommonTx','config_id.cpp'),
    os.path.join('CommonTx','in.cpp'),
    os.path.join('CommonTx','mlrs-tx.cpp'),
    ]


MLRS_SOURCES_USB = [
    os.path.join('Drivers','STM32_USB_Device_Library','Class','CDC','Src','usbd_cdc.c'),
    os.path.join('Drivers','STM32_USB_Device_Library','Core','Src','usbd_core.c'),
    os.path.join('Drivers','STM32_USB_Device_Library','Core','Src','usbd_ctlreq.c'),
    os.path.join('Drivers','STM32_USB_Device_Library','Core','Src','usbd_ioreq.c'),
    os.path.join('..','modules','stm32-usb-device','usbd_cdc_if.c'),
    os.path.join('..','modules','stm32-usb-device','usbd_conf.c'),
    os.path.join('..','modules','stm32-usb-device','usbd_desc.c'),
    ]

MLRS_INCLUDES_USB = [
    os.path.join('Drivers','STM32_USB_Device_Library','Class','CDC','Inc'),
    os.path.join('Drivers','STM32_USB_Device_Library','Core','Inc'),
    os.path.join('..','modules','stm32-usb-device'),
    ]


#-- target class to handle targets
# target:       name of target
# target_D:     define for target in the code
# mcu_D:        define for mcu in  command line, e.g. -DSTM32WLE5xx
# mcu_HAL:      mcu part in the folder name to HAL drivers, e.g. .../Drivers/STM32WLxx_HAL_Driver/...

class cTarget:
    def __init__(self, target, target_D, mcu_D, mcu_HAL, startup_script, linker_script, mcu_option_list, extra_D_list, build_dir, elf_name):
        self.target = target
        self.target_D = target_D
        self.mcu_D = mcu_D
        self.mcu_HAL = mcu_HAL
        self.startup_script = startup_script
        self.linker_script = linker_script
        self.mcu_option_list = mcu_option_list
        self.extra_D_list = extra_D_list
        self.build_dir = build_dir
        self.elf_name = elf_name

        self.mcu_family = ''
        if 'F1' in self.mcu_D and 'F1' in self.mcu_HAL:
            self.mcu_family = 'f1'
        elif 'G4' in self.mcu_D and 'G4' in self.mcu_HAL:
            self.mcu_family = 'g4'
        elif 'L4' in self.mcu_D and 'L4' in self.mcu_HAL:
            self.mcu_family = 'l4'
        elif 'WL' in self.mcu_D and 'WL' in self.mcu_HAL:
            self.mcu_family = 'wl'
        elif 'F0' in self.mcu_D and 'F0' in self.mcu_HAL:
            self.mcu_family = 'f0'
        elif 'F3' in self.mcu_D and 'F3' in self.mcu_HAL:
            self.mcu_family = 'f3'
        else:
            printError('ERROR: Unsupported MCU family detected')
            print('mcu_D',self.mcu_D)
            print('mcu_HAL',self.mcu_HAL)
            exit(1)

        self.rx_or_tx = ''
        self.is_rx = False
        self.is_tx = False
        if target[:3] == 'rx-' and target_D[:3] == 'RX_':
            self.rx_or_tx = 'rx'
            self.is_rx = True
        elif target[:3] == 'tx-' and target_D[:3] == 'TX_':
            self.rx_or_tx = 'tx'
            self.is_tx = True
        else:
            printError('ERROR: Invalid target name - must start with rx- or tx-')
            exit(1)

        self.D_list = ['USE_HAL_DRIVER', 'USE_FULL_LL_DRIVER']
        if self.mcu_family == 'wl':
            self.D_list.append('CORE_CM4')

        self.MLRS_SOURCES_HAL = []
        if self.mcu_family == 'f1':
            self.MLRS_SOURCES_HAL = MLRS_SOURCES_HAL_STM32F1
        elif self.mcu_family == 'g4':
            self.MLRS_SOURCES_HAL = MLRS_SOURCES_HAL_STM32G4
        elif self.mcu_family == 'wl':
            self.MLRS_SOURCES_HAL = MLRS_SOURCES_HAL_STM32WL
        elif self.mcu_family == 'l4':
            self.MLRS_SOURCES_HAL = MLRS_SOURCES_HAL_STM32L4
        elif self.mcu_family == 'f0':
            self.MLRS_SOURCES_HAL = MLRS_SOURCES_HAL_STM32F0
        elif self.mcu_family == 'f3':
            self.MLRS_SOURCES_HAL = MLRS_SOURCES_HAL_STM32F3

        self.MLRS_SOURCES_CORE = []
        for file in MLRS_SOURCES_CORE:
            self.MLRS_SOURCES_CORE.append(file.replace('??',self.mcu_family))

        self.MLRS_INCLUDES = []
        for file in MLRS_INCLUDES:
            self.MLRS_INCLUDES.append(file.replace('??',self.mcu_HAL))

        self.MLRS_SOURCES_EXTRA = []
        if 'STDSTM32_USE_USB' in self.extra_D_list:
            for file in MLRS_SOURCES_USB:
                self.MLRS_SOURCES_EXTRA.append(file)
            for file in MLRS_INCLUDES_USB:
                self.MLRS_INCLUDES.append(file)
        else: # add stm32-usb-device sources to every target (we could have excluded them in the IDE, but are too lazy LOL)
            for file in MLRS_SOURCES_USB:
                if 'modules' in file:
                    self.MLRS_SOURCES_EXTRA.append(file)


#-- compiler & linker

def mlrs_compile_file(target, file):
    """Compile a single file. Called from global thread pool.
    
    Supports incremental builds by checking:
    - If object file exists and is newer than source
    - If all header dependencies are older than object
    - If compiler flags haven't changed
    """
    file_path = os.path.dirname(file)
    file_name = os.path.splitext(file)[0]
    file_ext = os.path.splitext(file)[1]

    is_cpp = False
    is_asm = False
    if file_ext == '.cpp': is_cpp = True
    if file_ext == '.s': is_asm = True

    # define output paths
    object_path = os.path.join(MLRS_BUILD_DIR, target.build_dir, file_name) + '.o'
    dep_file_path = os.path.join(MLRS_BUILD_DIR, target.build_dir, file_name) + '.d'
    flags_file_path = os.path.join(MLRS_BUILD_DIR, target.build_dir, file_name) + '.flags'
    source_path = os.path.join(MLRS_DIR, file)

    # construct command line as a list for subprocess.run
    cmd = []
    if is_cpp:
        cmd.append(os.path.join(GCC_DIR, 'arm-none-eabi-g++'))
    else:
        cmd.append(os.path.join(GCC_DIR, 'arm-none-eabi-gcc'))

    if not is_asm:
        cmd.append(source_path)
        if is_cpp:
            cmd.append('-std=gnu++14')
        else:
            cmd.append('-std=gnu11')

    cmd.append('-c')

    for mcu_option in target.mcu_option_list:
        cmd.append(mcu_option)
    cmd.append('-mthumb')
    cmd.append('--specs=nano.specs')

    if not is_asm:
        for d in target.D_list:
            cmd.append('-D' + d)
        cmd.append('-D' + target.mcu_D)
        if is_cpp:
            cmd.append('-D' + target.target_D)
        for extra_D in target.extra_D_list:
            cmd.append('-D' + extra_D)

        for file_inc in target.MLRS_INCLUDES:
            cmd.append('-I' + os.path.join(MLRS_DIR, target.target, file_inc))

        cmd.append('-Os')
        cmd.append('-ffunction-sections')
        cmd.append('-fdata-sections')
        cmd.append('-Wall')
        cmd.append('-fstack-usage')
        if is_cpp:
            cmd.append('-fno-exceptions')
            cmd.append('-fno-rtti')
            cmd.append('-fno-use-cxa-atexit')
    else:
        cmd.append('-x')
        cmd.append('assembler-with-cpp')

    cmd.append('-MMD')
    cmd.append('-MP')
    cmd.append('-MF' + dep_file_path)
    cmd.append('-MT' + object_path)
    cmd.append('-o')
    cmd.append(object_path)

    if is_asm:
        cmd.append(source_path)

    # compute flags hash for incremental build detection
    # exclude paths that might change (use only flags and defines)
    flags_for_hash = [arg for arg in cmd[1:] if not arg.startswith('-I') and not arg.startswith('-MF') and not arg.startswith('-MT') and not arg.startswith('-o') and not arg.startswith('/')]
    current_flags_hash = compute_flags_hash(flags_for_hash)

    # check if recompilation is needed
    if not needs_recompile(source_path, object_path, dep_file_path, flags_file_path, current_flags_hash):
        return {'file': file, 'success': True, 'skipped': True}

    # create folder as needed
    buildpath = os.path.join(MLRS_BUILD_DIR, target.build_dir, file_path)
    create_dir(buildpath)

    # execute using subprocess for better error handling
    result = subprocess.run(cmd, capture_output=True, text=True)
    
    if result.returncode != 0:
        return {
            'file': file,
            'success': False,
            'returncode': result.returncode,
            'stderr': result.stderr,
            'stdout': result.stdout
        }
    
    # save flags hash for next incremental build check
    save_flags_hash(flags_file_path, current_flags_hash)
    
    return {'file': file, 'success': True, 'skipped': False}



def mlrs_link_target(target):
    # generate object list from actually created .o files
    objlist = []
    for path, subdirs, files in os.walk(os.path.join(MLRS_BUILD_DIR,target.build_dir)):
        for file in files:
            if os.path.splitext(file)[1] == '.o':
                obj = os.path.join(path,file).replace(os.path.join(MLRS_BUILD_DIR,target.build_dir), '')
                objlist.append(obj.replace('\\','/'))

    # always use generated object list (ensures newly added source files are included)
    F = open(os.path.join(MLRS_BUILD_DIR,target.build_dir,'objects.list'), mode='w')
    for obj in sorted(objlist): # we use sorted, this at least makes it that it is somehow standardized, thus repeatable
        F.write('"'+os.path.join(MLRS_BUILD_DIR,target.build_dir).replace('\\','/')+obj+'"\n')
    F.close()

    # generate command line as a list for subprocess.run
    cmd = [
        os.path.join(GCC_DIR, 'arm-none-eabi-g++'),
        '-o', os.path.join(MLRS_BUILD_DIR, target.build_dir, target.elf_name + '.elf'),
        '@' + os.path.join(MLRS_BUILD_DIR, target.build_dir, 'objects.list'),
        '-T' + os.path.join(MLRS_DIR, target.target, target.linker_script),
    ]
    cmd.extend(target.mcu_option_list)
    cmd.extend([
        '-mthumb',
        '--specs=nano.specs',
        '--specs=nosys.specs',
        '-static',
        '-Wl,-Map=' + os.path.join(MLRS_BUILD_DIR, target.build_dir, target.target + '.map'),
        '-Wl,--gc-sections',
        '-Wl,--start-group', '-lc', '-lm', '-lstdc++', '-lsupc++', '-Wl,--end-group',
    ])

    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        printError(f'Linking failed for {target.target}')
        if result.stderr:
            print(result.stderr)
        if result.stdout:
            print(result.stdout)
        return False
    return True


def mlrs_build_target(target, cmdline_D_list, sequential=False, skip_hex=False):
    if cmdline_D_list != []:
        #target.extra_D_list = cmdline_D_list
        target.extra_D_list += cmdline_D_list
    print('target', target.target, target.extra_D_list)

    buildpath = os.path.join(MLRS_BUILD_DIR,target.build_dir)
    # use create_dir instead of create_clean_dir for incremental builds
    create_dir(buildpath)

#    mlrs_compile_file(target, MLRS_SOURCES_MODULES[0])
#    return
#    mlrs_compile_file(target, os.path.join(target.target,MLRS_SOURCES_HAL_STM32F1[0]))
#    mlrs_compile_file(target, os.path.join(target.target,MLRS_STARTUP_SCRIPT_STM32F1[0]))

    # collect all files to compile
    files_to_compile = []
    
    # add startup script
    files_to_compile.append(os.path.join(target.target,'Core','Startup',target.startup_script))
    
    # add HAL sources
    for file in target.MLRS_SOURCES_HAL:
        files_to_compile.append(os.path.join(target.target,file))
    
    # add Core sources
    for file in target.MLRS_SOURCES_CORE:
        files_to_compile.append(os.path.join(target.target,file))
    
    # add module sources
    for file in MLRS_SOURCES_MODULES:
        files_to_compile.append(file)
    
    # add common sources
    for file in MLRS_SOURCES_COMMON:
        files_to_compile.append(file)
    
    # add extra sources
    for file in target.MLRS_SOURCES_EXTRA:
        files_to_compile.append(os.path.join(target.target,file))
    
    # add RX or TX sources
    MLRS_SOURCES_RXTX = []
    if target.rx_or_tx == 'rx':
        MLRS_SOURCES_RXTX = MLRS_SOURCES_RX
    elif target.rx_or_tx == 'tx':
        MLRS_SOURCES_RXTX = MLRS_SOURCES_TX
    else:
        printError('ERROR: Target must be either rx or tx')
        exit(1)
    for file in MLRS_SOURCES_RXTX:
        files_to_compile.append(file)
    
    # compile files - either sequentially or in parallel
    compilation_start = time.time()
    skipped_count = 0
    compiled_count = 0
    
    if sequential:
        # sequential compilation (original behavior)
        print(f'compiling {target.target} (sequential) - {len(files_to_compile)} files')
        for file in files_to_compile:
            result = mlrs_compile_file(target, file)
            if not result['success']:
                printError(f"ERROR compiling {result['file']}")
                if result.get('stderr'):
                    print(result['stderr'])
                if result.get('stdout'):
                    print(result['stdout'])
                exit(1)
            if result.get('skipped'):
                skipped_count += 1
            else:
                compiled_count += 1
    else:
        # parallel compilation using global thread pool
        # all targets share the pool, which provides natural load balancing
        print(f'compiling {target.target} (parallel): {len(files_to_compile)} files')
        
        # submit all compilation tasks to global pool
        futures = [GLOBAL_COMPILE_POOL.submit(mlrs_compile_file, target, f) 
                   for f in files_to_compile]
        
        # collect results as they complete
        compilation_failed = False
        for future in as_completed(futures):
            try:
                result = future.result()
                if not result['success']:
                    printError(f"ERROR compiling {result['file']}")
                    if result.get('stderr'):
                        print(result['stderr'])
                    if result.get('stdout'):
                        print(result['stdout'])
                    compilation_failed = True
                else:
                    if result.get('skipped'):
                        skipped_count += 1
                    else:
                        compiled_count += 1
            except Exception as exc:
                printError(f'ERROR: compilation generated an exception: {exc}')
                compilation_failed = True
        
        if compilation_failed:
            printError('Compilation failed, aborting')
            exit(1)
    
    compilation_time = time.time() - compilation_start


    if skipped_count > 0:
        print(f'linking {target.target} (compiled: {compiled_count}, skipped: {skipped_count})')
    else:
        print(f'linking {target.target}')

    if not mlrs_link_target(target):
        exit(1)
    
    # capture size output instead of printing it
    size_output = subprocess.getoutput(
        os.path.join(GCC_DIR,'arm-none-eabi-size')+' '+os.path.join(MLRS_BUILD_DIR,target.build_dir,target.elf_name+'.elf')
    )

    if 'MLRS_FEATURE_ELRS_BOOTLOADER' in target.extra_D_list:
        subprocess.run([
            os.path.join(GCC_DIR, 'arm-none-eabi-objcopy'),
            '-O', 'binary',
            os.path.join(MLRS_BUILD_DIR, target.build_dir, target.elf_name + '.elf'),
            os.path.join(MLRS_BUILD_DIR, target.build_dir, target.elf_name + '.elrs')
        ], check=True)
    elif not skip_hex:
        # generate .hex for distribution (skip when flashing directly from .elf)
        subprocess.run([
            os.path.join(GCC_DIR, 'arm-none-eabi-objcopy'),
            '-O', 'ihex',
            os.path.join(MLRS_BUILD_DIR, target.build_dir, target.elf_name + '.elf'),
            os.path.join(MLRS_BUILD_DIR, target.build_dir, target.elf_name + '.hex')
        ], check=True)

    
    # return build info for later display
    return {
        'target': target,
        'compilation_time': compilation_time,
        'size_output': size_output,
        'compiled_count': compiled_count,
        'skipped_count': skipped_count
    }


#-- mcu family generic targets

class cTargetF1(cTarget):
    def __init__(self, target, target_D, mcu_D, startup_script, linker_script, extra_D_list, build_dir, elf_name):
        super().__init__(
            target, target_D,
            mcu_D, 'STM32F1xx',
            startup_script, linker_script,
            ['-mcpu=cortex-m3', '-mfloat-abi=soft'],
            extra_D_list, build_dir, elf_name)

class cTargetG4(cTarget):
    def __init__(self, target, target_D, mcu_D, startup_script, linker_script, extra_D_list, build_dir, elf_name):
        super().__init__(
            target, target_D,
            mcu_D, 'STM32G4xx',
            startup_script, linker_script,
            ['-mcpu=cortex-m4', '-mfpu=fpv4-sp-d16', '-mfloat-abi=hard'],
            extra_D_list, build_dir, elf_name)

class cTargetWL(cTarget):
    def __init__(self, target, target_D, mcu_D, startup_script, linker_script, extra_D_list, build_dir, elf_name):
        super().__init__(
            target, target_D,
            mcu_D, 'STM32WLxx',
            startup_script, linker_script,
            ['-mcpu=cortex-m4', '-mfloat-abi=soft'],
            extra_D_list, build_dir, elf_name)

class cTargetL4(cTarget):
    def __init__(self, target, target_D, mcu_D, startup_script, linker_script, extra_D_list, build_dir, elf_name):
        super().__init__(
            target, target_D,
            mcu_D, 'STM32L4xx',
            startup_script, linker_script,
            ['-mcpu=cortex-m4', '-mfpu=fpv4-sp-d16', '-mfloat-abi=hard'],
            extra_D_list, build_dir, elf_name)

class cTargetF0(cTarget):
    def __init__(self, target, target_D, mcu_D, startup_script, linker_script, extra_D_list, build_dir, elf_name):
        super().__init__(
            target, target_D,
            mcu_D, 'STM32F0xx',
            startup_script, linker_script,
            ['-mcpu=cortex-m0', '-mfloat-abi=soft'],
            extra_D_list, build_dir, elf_name)

class cTargetF3(cTarget):
    def __init__(self, target, target_D, mcu_D, startup_script, linker_script, extra_D_list, build_dir, elf_name):
        super().__init__(
            target, target_D,
            mcu_D, 'STM32F3xx',
            startup_script, linker_script,
            ['-mcpu=cortex-m4', '-mfpu=fpv4-sp-d16', '-mfloat-abi=hard'],
            extra_D_list, build_dir, elf_name)


#-- mcu specific targets

class cTargetF103C8(cTargetF1):
    def __init__(self, target, target_D, extra_D_list, build_dir, elf_name, package):
        if package == '': package = 'tx'
        super().__init__(
            target, target_D,
            'STM32F103xB', 'startup_stm32f103c8'+package.lower()+'.s', 'STM32F103C8'+package.upper()+'_FLASH.ld',
            extra_D_list, build_dir, elf_name)

class cTargetF103CB(cTargetF1):
    def __init__(self, target, target_D, extra_D_list, build_dir, elf_name, package):
        if package == '': package = 'tx'
        super().__init__(
            target, target_D,
            'STM32F103xB', 'startup_stm32f103cb'+package.lower()+'.s', 'STM32F103CB'+package.upper()+'_FLASH.ld',
            extra_D_list, build_dir, elf_name)

class cTargetF103RB(cTargetF1):
    def __init__(self, target, target_D, extra_D_list, build_dir, elf_name, package):
        if package == '': package = 'hx'
        super().__init__(
            target, target_D,
            'STM32F103xB', 'startup_stm32f103rb'+package.lower()+'.s', 'STM32F103RB'+package.upper()+'_FLASH.ld',
            extra_D_list, build_dir, elf_name)


class cTargetG431KB(cTargetG4):
    def __init__(self, target, target_D, extra_D_list, build_dir, elf_name, package):
        if package == '': package = 'ux'
        super().__init__(
            target, target_D,
            'STM32G431xx', 'startup_stm32g431kb'+package.lower()+'.s', 'STM32G431KB'+package.upper()+'_FLASH.ld',
            extra_D_list, build_dir, elf_name)

class cTargetG441KB(cTargetG4): #is code compatible to G431KB!?
    def __init__(self, target, target_D, extra_D_list, build_dir, elf_name, package):
        if package == '': package = 'ux'
        super().__init__(
            target, target_D,
            'STM32G441xx',  'startup_stm32g441kb'+package.lower()+'.s', 'STM32G441KB'+package.upper()+'_FLASH.ld',
            extra_D_list, build_dir, elf_name)

class cTargetG431CB(cTargetG4):
    def __init__(self, target, target_D, extra_D_list, build_dir, elf_name, package):
        if package == '': package = 'ux'
        super().__init__(
            target, target_D,
            'STM32G431xx', 'startup_stm32g431cb'+package.lower()+'.s', 'STM32G431CB'+package.upper()+'_FLASH.ld',
            extra_D_list, build_dir, elf_name)

class cTargetG491RE(cTargetG4):
    def __init__(self, target, target_D, extra_D_list, build_dir, elf_name, package):
        if package == '': package = 'tx'
        super().__init__(
            target, target_D,
            'STM32G491xx', 'startup_stm32g491re'+package.lower()+'.s', 'STM32G491RE'+package.upper()+'_FLASH.ld',
            extra_D_list, build_dir, elf_name)

class cTargetG474CE(cTargetG4):
    def __init__(self, target, target_D, extra_D_list, build_dir, elf_name, package):
        if package == '': package = 'ux'
        super().__init__(
            target, target_D,
            'STM32G474xx', 'startup_stm32g474ce'+package.lower()+'.s', 'STM32G474CE'+package.upper()+'_FLASH.ld',
            extra_D_list, build_dir, elf_name)


class cTargetWLE5CC(cTargetWL):
    def __init__(self, target, target_D, extra_D_list, build_dir, elf_name):
        super().__init__(
            target, target_D,
            'STM32WLE5xx', 'startup_stm32wle5ccux.s', 'STM32WLE5CCUX_FLASH.ld',
            extra_D_list, build_dir, elf_name)

class cTargetWLE5JC(cTargetWL):
    def __init__(self, target, target_D, extra_D_list, build_dir, elf_name):
        super().__init__(
            target, target_D,
            'STM32WLE5xx', 'startup_stm32wle5jcix.s', 'STM32WLE5JCIX_FLASH.ld',
            extra_D_list, build_dir, elf_name)


class cTargetL433CB(cTargetL4):
    def __init__(self, target, target_D, extra_D_list, build_dir, elf_name, package):
        s_file = 'startup_stm32l433cb'+package.lower()+'.s'
        ld_file = 'STM32L433CB'+package.upper()+'_FLASH.ld'
        super().__init__(
            target, target_D,
            'STM32L433xx', s_file, ld_file,
            extra_D_list, build_dir, elf_name)


class cTargetF072CB(cTargetF0):
    def __init__(self, target, target_D, extra_D_list, build_dir, elf_name):
        super().__init__(
            target, target_D,
            'STM32F072xB', 'startup_stm32f072cbtx.s', 'STM32F072CBTX_FLASH.ld',
            extra_D_list, build_dir, elf_name)


class cTargetF303CC(cTargetF3):
    def __init__(self, target, target_D, extra_D_list, build_dir, elf_name, package):
        if package == '': package = 'tx'
        super().__init__(
            target, target_D,
            'STM32F303xC', 'startup_stm32f303cc'+package.lower()+'.s', 'STM32F303CC'+package.upper()+'_FLASH.ld',
            extra_D_list, build_dir, elf_name)


#--------------------------------------------------
# application
#--------------------------------------------------

#-- list of targets

TLIST = [
    {
#-- MatekSys mLRS devices
        'target' : 'rx-matek-mr24-30-g431kb',           'target_D' : 'RX_MATEK_MR24_30_G431KB',
        'extra_D_list' : [], 'appendix' : '',
    },{
        'target' : 'rx-matek-mr900-30-g431kb',          'target_D' : 'RX_MATEK_MR900_30_G431KB',
        'extra_D_list' : [], 'appendix' : '',
    },{
        'target' : 'rx-matek-mr900-22-wle5cc',          'target_D' : 'RX_MATEK_MR900_22_WLE5CC',
        'extra_D_list' : [], 'appendix' : '',
    },{

        'target' : 'rx-matek-mr24-30-g431kb',           'target_D' : 'RX_MATEK_MR24_30_G431KB',
        'extra_D_list' : ['MLRS_FEATURE_CAN'], 'appendix' : '-can',
    },{
        'target' : 'rx-matek-mr900-30-g431kb',          'target_D' : 'RX_MATEK_MR900_30_G431KB',
        'extra_D_list' : ['MLRS_FEATURE_CAN'], 'appendix' : '-can',
    },{

        'target' : 'tx-matek-mr24-30-g431kb',           'target_D' : 'TX_MATEK_MR24_30_G431KB',
        'extra_D_list' : ['STDSTM32_USE_USB'], 'appendix' : '-default',
    },{
        'target' : 'tx-matek-mr24-30-g431kb',           'target_D' : 'TX_MATEK_MR24_30_G431KB',
        'extra_D_list' : ['STDSTM32_USE_USB','MLRS_FEATURE_MATEK_TXMODULE_SIKTELEM'], 'appendix' : '-siktelem',
    },{
#        'target' : 'tx-matek-mr24-30-g431kb',           'target_D' : 'TX_MATEK_MR24_30_G431KB',
#        'extra_D_list' : ['STDSTM32_USE_USB','MLRS_FEATURE_MATEK_TXMODULE_MOD','MLRS_FEATURE_HC04_MODULE','MLRS_FEATURE_COM_ON_USB','MLRS_FEATURE_OLED'],
#        'appendix' : '-oled',
#    },{

        'target' : 'rx-matek-mr900-30c-g431kb',         'target_D' : 'RX_MATEK_MR900_30C_G431KB',
        'extra_D_list' : [], 'appendix' : '',
    },{
        'target' : 'rx-matek-mr900-td30-g474ce',        'target_D' : 'RX_MATEK_MR900_TD30_G474CE',
        'extra_D_list' : [], 'appendix' : '',
    },{

        'target' : 'tx-matek-mr900-30-g431kb',          'target_D' : 'TX_MATEK_MR900_30_G431KB',
        'extra_D_list' : ['STDSTM32_USE_USB'], 'appendix' : '-default',
    },{
        'target' : 'tx-matek-mr900-30-g431kb',          'target_D' : 'TX_MATEK_MR900_30_G431KB',
        'extra_D_list' : ['STDSTM32_USE_USB','MLRS_FEATURE_MATEK_TXMODULE_SIKTELEM'], 'appendix' : '-siktelem',
    },{
#        'target' : 'tx-matek-mr900-30-g431kb',          'target_D' : 'TX_MATEK_MR900_30_G431KB',
#        'extra_D_list' : ['STDSTM32_USE_USB','MLRS_FEATURE_MATEK_TXMODULE_MOD','MLRS_FEATURE_HC04_MODULE','MLRS_FEATURE_COM_ON_USB','MLRS_FEATURE_OLED'],
#        'appendix' : '-oled',
#    },{

        'target' : 'tx-matek-mtx-db30-g474ce',          'target_D' : 'TX_MATEK_MTX_DB30_G474CE',
        'extra_D_list' : ['STDSTM32_USE_USB'], 'appendix' : '-default',
    },{
  
#-- FrSky R9
        'target' : 'rx-R9M-f103c8',                     'target_D' : 'RX_R9M_868_F103C8',
        'extra_D_list' : [], 'appendix' : '',
    },{
        'target' : 'rx-R9M-f103c8',                     'target_D' : 'RX_R9M_868_F103C8',
        'extra_D_list' : ['MLRS_FEATURE_ELRS_BOOTLOADER'],
        'appendix' : '-elrs-bl',
    },{
        'target' : 'rx-R9MM-f103rb',                    'target_D' : 'RX_R9MM_868_F103RB',
        'extra_D_list' : [], 'appendix' : '',
    },{
        'target' : 'rx-R9MM-f103rb',                    'target_D' : 'RX_R9MM_868_F103RB',
        'extra_D_list' : ['MLRS_FEATURE_ELRS_BOOTLOADER'],
        'appendix' : '-elrs-bl',
    },{
        'target' : 'rx-R9MX-l433cb',                    'target_D' : 'RX_R9MX_868_L433CB',
        'package' : 'ux',
        'extra_D_list' : [], 'appendix' : '',
    },{
        'target' : 'rx-R9MX-l433cb',                    'target_D' : 'RX_R9MX_868_L433CB',
        'package' : 'ux',
        'extra_D_list' : ['MLRS_FEATURE_ELRS_BOOTLOADER'],
        'appendix' : '-elrs-bl',
    },{
        'target' : 'rx-R9MLitePro-v15-f303cc',          'target_D' : 'RX_R9MLITEPRO_F303CC',
        'extra_D_list' : [], 'appendix' : '',
    },{

        'target' : 'tx-R9M-f103c8',                     'target_D' : 'TX_R9M_868_F103C8',
        'fclass': 'FrSky R9', 'fname': 'R9M',
        'extra_D_list' : [], 'appendix' : '',
    },{
        'target' : 'tx-R9M-f103c8',                     'target_D' : 'TX_R9M_868_F103C8',
        'extra_D_list' : ['MLRS_FEATURE_ELRS_BOOTLOADER'],
        'appendix' : '-elrs-bl',
    },{
        'target' : 'tx-R9MX-l433cb',                    'target_D' : 'TX_R9MX_868_L433CB',
        'package' : 'ux',
        'extra_D_list' : [], 'appendix' : '',
    },{
        'target' : 'tx-R9MX-l433cb',                    'target_D' : 'TX_R9MX_868_L433CB',
        'package' : 'ux',
        'extra_D_list' : ['MLRS_FEATURE_ELRS_BOOTLOADER'],
        'appendix' : '-elrs-bl',
    },{

#-- FlySky FRM303
#        'target' : 'rx-FRM303-f072cb',                  'target_D' : 'RX_FRM303_F072CB',
#        'extra_D_list' : [], 'appendix' : '',
#
#    },{
#        'target' : 'tx-FRM303-f072cb',                  'target_D' : 'TX_FRM303_F072CB',
#        'extra_D_list' : ['STDSTM32_USE_USB'],
#        'appendix' : '-usb',
#    },{
#        'target' : 'tx-FRM303-f072cb',                  'target_D' : 'TX_FRM303_F072CB',
#        'extra_D_list' : ['STDSTM32_USE_USB','MLRS_FEATURE_OLED'],
#        'appendix' : '-oled',
#    },{

#RX
#-- rx diy
        'target' : 'rx-diy-board01-f103cb',             'target_D' : 'RX_DIY_BOARD01_F103CB',
        'extra_D_list' : [], 'appendix' : ''
    },{
        'target' : 'rx-diy-e22-g441kb',                 'target_D' : 'RX_DIY_E22_G441KB',
        'extra_D_list' : [], 'appendix' : ''
    },{
        'target' : 'rx-diy-e28dual-board02-f103cb',     'target_D' : 'RX_DIY_E28DUAL_BOARD02_F103CB',
        'extra_D_list' : [], 'appendix' : ''
    },{
        'target' : 'rx-diy-e28-g441kb',                 'target_D' : 'RX_DIY_E28_G441KB',
        'extra_D_list' : [], 'appendix' : ''
    },{
        'target' : 'rx-diy-WioE5-E22-dual-wle5jc',      'target_D' : 'RX_DIY_WIOE5_E22_WLE5JC',
        'extra_D_list' : [], 'appendix' : ''
    },{
#-- rx WioE5 Mini, Grove
        'target' : 'rx-Wio-E5-Mini-wle5jc',             'target_D' : 'RX_WIO_E5_MINI_WLE5JC',
        'extra_D_list' : [], 'appendix' : '',
    },{
        'target' : 'rx-Wio-E5-Grove-wle5jc',            'target_D' : 'RX_WIO_E5_GROVE_WLE5JC',
        'extra_D_list' : [], 'appendix' : '',
    },{
#-- rx E77 MBL
        'target' : 'rx-E77-MBLKit-wle5cc',              'target_D' : 'RX_E77_MBLKIT_WLE5CC',
        'extra_D_list' : ['MLRS_FEATURE_868_MHZ','MLRS_FEATURE_915_MHZ_FCC'],
        'appendix' : '-900-tcxo',
    },{
        'target' : 'rx-E77-MBLKit-wle5cc',              'target_D' : 'RX_E77_MBLKIT_WLE5CC',
        'extra_D_list' : ['MLRS_FEATURE_433_MHZ'],
        'appendix' : '-400-tcxo',
    },{
        'target' : 'rx-E77-MBLKit-wle5cc',              'target_D' : 'RX_E77_MBLKIT_WLE5CC',
        'extra_D_list' : ['MLRS_FEATURE_868_MHZ','MLRS_FEATURE_915_MHZ_FCC','MLRS_FEATURE_E77_XTAL'],
        'appendix' : '-900-xtal',
    },{
        'target' : 'rx-E77-MBLKit-wle5cc',              'target_D' : 'RX_E77_MBLKIT_WLE5CC',
        'extra_D_list' : ['MLRS_FEATURE_433_MHZ','MLRS_FEATURE_E77_XTAL'],
        'appendix' : '-400-xtal',
    },{

#TX
#-- tx diy
        'target' : 'tx-diy-e22dual-module02-g491re',    'target_D' : 'TX_DIY_E22DUAL_MODULE02_G491RE',
        'extra_D_list' : [] , 'appendix' : ''
    },{
        'target' : 'tx-diy-e22-g431kb',                 'target_D' : 'TX_DIY_E22_G431KB',
        'extra_D_list' : [], 'appendix' : ''
    },{
        'target' : 'tx-diy-e28dual-board02-f103cb',     'target_D' : 'TX_DIY_E28DUAL_BOARD02_F103CB',
        'extra_D_list' : [], 'appendix' : ''
    },{
        'target' : 'tx-diy-e28dual-module02-g491re',    'target_D' : 'TX_DIY_E28DUAL_MODULE02_G491RE',
        'extra_D_list' : [], 'appendix' : ''
    },{
        'target' : 'tx-diy-sxdual-module02-g491re',     'target_D' : 'TX_DIY_SXDUAL_MODULE02_G491RE',
        'extra_D_list' : [], 'appendix' : ''
    },{
        'target' : 'tx-diy-WioE5-E22-dual-wle5jc',      'target_D' : 'TX_DIY_WIOE5_E22_WLE5JC',
        'extra_D_list' : [], 'appendix' : ''
    },{
#-- tx WioE5 Mini
        'target' : 'tx-Wio-E5-Mini-wle5jc',             'target_D' : 'TX_WIO_E5_MINI_WLE5JC',
        'extra_D_list' : [], 'appendix' : '',
    },{
#-- tx E77 MBL
        'target' : 'tx-E77-MBLKit-wle5cc',              'target_D' : 'TX_E77_MBLKIT_WLE5CC',
        'extra_D_list' : ['MLRS_FEATURE_868_MHZ','MLRS_FEATURE_915_MHZ_FCC'],
        'appendix' : '-900-tcxo',
    },{
        'target' : 'tx-E77-MBLKit-wle5cc',              'target_D' : 'TX_E77_MBLKIT_WLE5CC',
        'extra_D_list' : ['MLRS_FEATURE_433_MHZ'],
        'appendix' : '-400-tcxo',
    },{
        'target' : 'tx-E77-MBLKit-wle5cc',              'target_D' : 'TX_E77_MBLKIT_WLE5CC',
        'extra_D_list' : ['MLRS_FEATURE_868_MHZ','MLRS_FEATURE_915_MHZ_FCC','MLRS_FEATURE_E77_XTAL'],
        'appendix' : '-900-xtal',
    },{
        'target' : 'tx-E77-MBLKit-wle5cc',              'target_D' : 'TX_E77_MBLKIT_WLE5CC',
        'extra_D_list' : ['MLRS_FEATURE_433_MHZ','MLRS_FEATURE_E77_XTAL'],
        'appendix' : '-400-xtal',
    },{

#-- rx easytosolder E77 E22
        'target' : 'rx-easysolder-E77-E22-dual-wle5cc', 'target_D' : 'RX_DIY_E77_E22_WLE5CC',
        'extra_D_list' : ['MLRS_FEATURE_NO_DIVERSITY'],
        'appendix' : '-tcxo',
    },{
        'target' : 'rx-easysolder-E77-E22-dual-wle5cc', 'target_D' : 'RX_DIY_E77_E22_WLE5CC',
        'extra_D_list' : ['MLRS_FEATURE_DIVERSITY'],
        'appendix' : '-diversity-tcxo',
    },{
        'target' : 'rx-easysolder-E77-E22-dual-wle5cc', 'target_D' : 'RX_DIY_E77_E22_WLE5CC',
        'extra_D_list' : ['MLRS_FEATURE_NO_DIVERSITY','MLRS_FEATURE_E77_XTAL'],
        'appendix' : '-xtal',
    },{
        'target' : 'rx-easysolder-E77-E22-dual-wle5cc', 'target_D' : 'RX_DIY_E77_E22_WLE5CC',
        'extra_D_list' : ['MLRS_FEATURE_DIVERSITY','MLRS_FEATURE_E77_XTAL'],
        'appendix' : '-diversity-xtal',
    },{
#-- tx easytosolder E77 E22
        'target' : 'tx-easysolder-E77-E22-dual-wle5cc', 'target_D' : 'TX_DIY_E77_E22_WLE5CC',
        'extra_D_list' : ['MLRS_FEATURE_NO_DIVERSITY'],
        'appendix' : '-tcxo',
    },{
        'target' : 'tx-easysolder-E77-E22-dual-wle5cc', 'target_D' : 'TX_DIY_E77_E22_WLE5CC',
        'extra_D_list' : ['MLRS_FEATURE_DIVERSITY'],
        'appendix' : '-diversity-tcxo',
    },{
        'target' : 'tx-easysolder-E77-E22-dual-wle5cc', 'target_D' : 'TX_DIY_E77_E22_WLE5CC',
        'extra_D_list' : ['MLRS_FEATURE_NO_DIVERSITY','MLRS_FEATURE_E77_XTAL'],
        'appendix' : '-xtal',
    },{
        'target' : 'tx-easysolder-E77-E22-dual-wle5cc', 'target_D' : 'TX_DIY_E77_E22_WLE5CC',
        'extra_D_list' : ['MLRS_FEATURE_DIVERSITY','MLRS_FEATURE_E77_XTAL'],
        'appendix' : '-diversity-xtal',
    },{
#-- easytosolder E77 E28/E22 dualband
        'target' : 'rx-easysolder-E77-E28-dualband-wle5cc', 'target_D' : 'RX_DIY_E77_E28_DUALBAND_WLE5CC',
        'extra_D_list' : [],
        'appendix' : '-tcxo',
    },{
        'target' : 'rx-easysolder-E77-E28-dualband-wle5cc', 'target_D' : 'RX_DIY_E77_E28_DUALBAND_WLE5CC',
        'extra_D_list' : ['MLRS_FEATURE_E77_XTAL'],
        'appendix' : '-xtal',
    },{
        'target' : 'rx-easysolder-E77-E22-dual-wle5cc', 'target_D' : 'RX_DIY_E77_E22_DUALBAND_WLE5CC',
        'extra_D_list' : [],
        'appendix' : '-dualband-tcxo',
    },{
        'target' : 'rx-easysolder-E77-E22-dual-wle5cc', 'target_D' : 'RX_DIY_E77_E22_DUALBAND_WLE5CC',
        'extra_D_list' : ['MLRS_FEATURE_E77_XTAL'],
        'appendix' : '-dualband-xtal',

    },{
        'target' : 'tx-easysolder-E77-E28-dualband-wle5cc', 'target_D' : 'TX_DIY_E77_E28_DUALBAND_WLE5CC',
        'extra_D_list' : [],
        'appendix' : '-tcxo',
    },{
        'target' : 'tx-easysolder-E77-E28-dualband-wle5cc', 'target_D' : 'TX_DIY_E77_E28_DUALBAND_WLE5CC',
        'extra_D_list' : ['MLRS_FEATURE_E77_XTAL'],
        'appendix' : '-xtal',

    },{
        'target' : 'tx-easysolder-E77-E22-dual-wle5cc', 'target_D' : 'TX_DIY_E77_E22_DUALBAND_WLE5CC',
        'extra_D_list' : [],
        'appendix' : '-dualband-tcxo',
    },{
        'target' : 'tx-easysolder-E77-E22-dual-wle5cc', 'target_D' : 'TX_DIY_E77_E22_DUALBAND_WLE5CC',
        'extra_D_list' : ['MLRS_FEATURE_E77_XTAL'],
        'appendix' : '-dualband-xtal',

    }
    ]


def mlrs_create_targetlist(appendix, extra_D_list):
    # mcu pattern to class mapping
    # format: 'mcu_pattern': (TargetClass, requires_package_param)
    MCU_CLASS_MAP = {
        'f103c8': (cTargetF103C8, True),
        'f103cb': (cTargetF103CB, True),
        'f103rb': (cTargetF103RB, True),
        'g431kb': (cTargetG431KB, True),
        'g441kb': (cTargetG441KB, True),
        'g431cb': (cTargetG431CB, True),
        'g491re': (cTargetG491RE, True),
        'g474ce': (cTargetG474CE, True),
        'wle5cc': (cTargetWLE5CC, False),
        'wle5jc': (cTargetWLE5JC, False),
        'l433cb': (cTargetL433CB, True),
        'f072cb': (cTargetF072CB, False),
        'f303cc': (cTargetF303CC, True),
    }
    
    tlist = []
    for t in TLIST:
        build_dir = t['target']+t['appendix']
        elf_name = t['target']+t['appendix']+appendix
        package = t.get('package', '')  # More Pythonic than checking keys()
        
        # find matching MCU class
        mcu_class = None
        requires_package = False
        for mcu_pattern, (target_class, needs_package) in MCU_CLASS_MAP.items():
            if mcu_pattern in t['target']:
                mcu_class = target_class
                requires_package = needs_package
                break
        
        if mcu_class:
            # instantiate target with or without package parameter
            if requires_package:
                tlist.append(mcu_class(t['target'], t['target_D'], t['extra_D_list'], build_dir, elf_name, package))
            else:
                tlist.append(mcu_class(t['target'], t['target_D'], t['extra_D_list'], build_dir, elf_name))
        else:
            printError('ERROR: Unknown MCU type in target name: ' + t['target'])
            exit(1)
    return tlist


def mlrs_copy_all_hex_etc():
    print('copying .hex files')
    firmwarepath = os.path.join(MLRS_BUILD_DIR,'firmware')
    create_clean_dir(firmwarepath)
    for path, subdirs, files in os.walk(MLRS_BUILD_DIR):
        for file in files:
            if 'firmware' in path:
                continue
            if os.path.splitext(file)[1] == '.hex':
                shutil.copy(os.path.join(path,file), os.path.join(firmwarepath,file))
            if os.path.splitext(file)[1] == '.elrs':
                shutil.copy(os.path.join(path,file), os.path.join(firmwarepath,file))


def find_stm32_cube_programmer():
    """Find STM32CubeProgrammer CLI. Checks CubeCLT first, then CubeIDE bundled version."""
    
    if sys.platform == 'darwin':
        # check for standalone CubeCLT installation first
        cube_clt_paths = [
            '/opt/ST/STM32CubeCLT/STM32CubeProgrammer/bin/STM32_Programmer_CLI',
            '/Applications/STM32CubeCLT/STM32CubeProgrammer/bin/STM32_Programmer_CLI',
            '/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/Resources/bin/STM32_Programmer_CLI',
        ]
        for path in cube_clt_paths:
            if os.path.exists(path):
                return path
        
        # check for CubeProgrammer bundled with CubeIDE
        macos_app_path = '/Applications/STM32CubeIDE.app'
        if os.path.exists(macos_app_path):
            plugins_dir = os.path.join(macos_app_path, 'Contents', 'Eclipse', 'plugins')
            try:
                for dirpath in os.listdir(plugins_dir):
                    if 'cubeprogrammer' in dirpath.lower() and 'macos64' in dirpath:
                        cli_path = os.path.join(plugins_dir, dirpath, 'tools', 'bin', 'STM32_Programmer_CLI')
                        if os.path.exists(cli_path):
                            return cli_path
            except OSError:
                pass
    
    elif sys.platform == 'win32':
        # windows paths
        cube_clt_paths = [
            'C:\\ST\\STM32CubeCLT\\STM32CubeProgrammer\\bin\\STM32_Programmer_CLI.exe',
            'C:\\Program Files\\STMicroelectronics\\STM32Cube\\STM32CubeProgrammer\\bin\\STM32_Programmer_CLI.exe',
        ]
        for path in cube_clt_paths:
            if os.path.exists(path):
                return path
    
    elif sys.platform == 'linux':
        # linux paths
        cube_clt_paths = [
            '/opt/st/stm32cubeclt/STM32CubeProgrammer/bin/STM32_Programmer_CLI',
            '/usr/local/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI',
        ]
        for path in cube_clt_paths:
            if os.path.exists(path):
                return path
    
    return None


def flash_via_dfu(cube_programmer_cli, elf_file, verify=True):
    """Flash firmware via DFU using STM32CubeProgrammer CLI
    
    Note: Reset is not supported via DFU interface. Device must be manually
    reset or power cycled after flashing.
    """
    # validate file exists
    if not os.path.exists(elf_file):
        printError(f'[ERROR] Firmware file not found: {elf_file}')
        return False
    
    print(f'Using: STM32CubeProgrammer CLI')
    print(f'  Path: {cube_programmer_cli}')
    print(f'Firmware: {elf_file}')
    
    file_size = os.path.getsize(elf_file)
    print(f'Size: {file_size} bytes ({file_size/1024:.1f} KB)')
    print()
    
    # build CubeProgrammer command for DFU
    # STM32_Programmer_CLI -c port=USB1 -w <file> -v
    # note: -rst is not included as reset is not supported via DFU
    cmd = [
        cube_programmer_cli,
        '-c', 'port=USB1',  # Connect via DFU (USB)
        '-w', elf_file,  # Write elf file (addresses embedded)
    ]
    
    if verify:
        cmd.append('-v')  # Verify after programming
    
    print(f'Executing: {" ".join(cmd)}')
    print()
    
    start_time = time.time()
    result = subprocess.run(cmd, capture_output=True, text=True)
    elapsed = time.time() - start_time
    
    # print output
    if result.stdout:
        print(result.stdout)
    if result.stderr:
        print(result.stderr)
    
    # check for success
    string_match = 'File download complete' in result.stdout
    if result.returncode == 0:
        success = True
    elif string_match:
        # returncode failed but output indicates success - warn about this
        print('[WARNING] Return code indicates failure but output shows success')
        success = True
    else:
        success = False
    
    if success:
        print()
        print(f'[SUCCESS] Flash completed successfully! (took {elapsed:.1f}s)')
        print('  Please manually reset or power cycle the device to run the new firmware.')
        return True
    else:
        print()
        printError(f'[ERROR] Flash failed!')
        printError('Make sure:')
        printError('  - Device is in DFU mode (BOOT0 button held during power-on/reset)')
        printError('  - Device is connected via USB')
        printError('  - No other programmer software is running')
        return False


def flash_via_swd(cube_programmer_cli, elf_file, verify=True, reset=True):
    """Flash firmware via SWD using STM32CubeProgrammer CLI"""
    # validate file exists
    if not os.path.exists(elf_file):
        printError(f'[ERROR] Firmware file not found: {elf_file}')
        return False
    
    print(f'Using: STM32CubeProgrammer CLI')
    print(f'  Path: {cube_programmer_cli}')
    print(f'Firmware: {elf_file}')
    
    file_size = os.path.getsize(elf_file)
    print(f'Size: {file_size} bytes ({file_size/1024:.1f} KB)')
    print()
    
    # connection options are space-separated
    cmd = [
        cube_programmer_cli,
        '-c', 'port=SWD',  # Connect via SWD
        '-w', elf_file,  # Write elf file (addresses embedded)
    ]
    
    if verify:
        cmd.append('-v')  # Verify after programming
    
    if reset:
        cmd.append('-rst')  # Reset after programming
    
    print(f'Executing: {" ".join(cmd)}')
    print()
    
    start_time = time.time()
    result = subprocess.run(cmd, capture_output=True, text=True)
    elapsed = time.time() - start_time
    
    # print output
    if result.stdout:
        print(result.stdout)
    if result.stderr:
        print(result.stderr)
    
    # check for success
    string_match = 'File download complete' in result.stdout
    if result.returncode == 0:
        success = True
    elif string_match:
        # returncode failed but output indicates success - warn about this
        print('[WARNING] Return code indicates failure but output shows success')
        success = True
    else:
        success = False
    
    if success:
        print()
        print(f'[SUCCESS] Flash completed successfully! (took {elapsed:.1f}s)')
        if reset:
            print('  Device has been reset and should be running the new firmware.')
        return True
    else:
        print()
        printError(f'[ERROR] Flash failed!')
        printError('Make sure:')
        printError('  - ST-LINK debugger is connected to SWD pins')
        printError('  - Device has power')
        printError('  - No other debugger software is running')
        return False


def flash_auto(cube_programmer_cli, elf_file, verify=True, reset=True):
    """Automatically detect and flash firmware via SWD or DFU
    
    This function tries to detect which flashing method is available and uses it.
    Based on testing, SWD detection fails faster (~0.115s) than DFU (~0.380s),
    so we try SWD first for better user experience.
    
    Args:
        cube_programmer_cli: Path to STM32_Programmer_CLI
        elf_file: Path to the .elf file to flash
        verify: Whether to verify after programming
        reset: Whether to reset after programming (only applies to SWD)
    
    Returns:
        True if flash succeeded, False otherwise
    """
    print('Auto-detecting flash method...')
    print()
    
    # Try SWD first (faster detection when not present)
    print('[1/2] Checking for SWD (ST-Link) connection...')
    cmd_swd = [cube_programmer_cli, '-c', 'port=SWD']
    
    start = time.time()
    try:
        result = subprocess.run(cmd_swd, capture_output=True, text=True, timeout=10)
        elapsed = time.time() - start
        
        if result.returncode == 0 or 'ST-LINK' in result.stdout:
            print(f'  ✓ SWD detected (took {elapsed:.1f}s)')
            print()
            print('Using SWD flashing method...')
            print('=' * 60)
            return flash_via_swd(cube_programmer_cli, elf_file, verify=verify, reset=reset)
        else:
            print(f'  ✗ SWD not found (took {elapsed:.1f}s)')
            print()
    except subprocess.TimeoutExpired:
        elapsed = time.time() - start
        print(f'  ✗ SWD detection timed out (took {elapsed:.1f}s)')
        print()
    
    # Try DFU as fallback
    print('[2/2] Checking for DFU (USB) connection...')
    cmd_dfu = [cube_programmer_cli, '-c', 'port=USB1']
    
    start = time.time()
    try:
        result = subprocess.run(cmd_dfu, capture_output=True, text=True, timeout=10)
        elapsed = time.time() - start
        
        if result.returncode == 0 or 'DFU' in result.stdout:
            print(f'  ✓ DFU detected (took {elapsed:.1f}s)')
            print()
            print('Using DFU flashing method...')
            print('=' * 60)
            return flash_via_dfu(cube_programmer_cli, elf_file, verify=verify)
        else:
            print(f'  ✗ DFU not found (took {elapsed:.1f}s)')
            print()
    except subprocess.TimeoutExpired:
        elapsed = time.time() - start
        print(f'  ✗ DFU detection timed out (took {elapsed:.1f}s)')
        print()
    
    # Neither method available
    print()
    printError('[ERROR] No flashing method detected!')
    printError('Please ensure one of the following:')
    printError('  • ST-LINK debugger is connected via SWD')
    printError('  • Device is in DFU mode and connected via USB')
    printError('    (Hold BOOT0 button during power-on/reset for DFU mode)')
    return False


def display_build_summary(build_info_list, build_start_time, failed_targets=None):
    """Display build summary with compilation times and firmware sizes.
    
    Args:
        build_info_list: List of dicts with 'target', 'compilation_time', and 'size_output'
        build_start_time: Start time for total elapsed calculation
        failed_targets: Optional list of failed targets (for parallel builds)
    """
    print('------------------------------------------------------------')
    print('BUILD SUMMARY')
    print('------------------------------------------------------------')
    for build_info in build_info_list:
        target = build_info['target']
        print(f"{target.target}: {build_info['compilation_time']:.2f}s")
    print(f"Total time: {time.time() - build_start_time:.2f}s")

    print('------------------------------------------------------------')
    print('FIRMWARE SIZES')
    print('------------------------------------------------------------')
    header_printed = False
    for build_info in build_info_list:
        lines = build_info['size_output'].splitlines()
        if len(lines) >= 2:
            if not header_printed:
                print(lines[0])
                header_printed = True
            print(lines[1])
    print('------------------------------------------------------------')
    
    # handle success/failure reporting
    if failed_targets is not None:
        # parallel build mode with failure tracking
        failed_count = len(failed_targets)
        built_count = len(build_info_list)
        if failed_count > 0:
            printError(f'Build completed with {failed_count} failures:')
            for target in failed_targets:
                printError(f'  - {target.target}')
        else:
            print(f'[OK] All {built_count} targets built successfully')
    else:
        # sequential build mode - all succeeded if we got here
        print(f'[OK] All {len(build_info_list)} targets built successfully')
    print('------------------------------------------------------------')


#-- here we go
if __name__ == "__main__":

    cmdline_target = ''
    cmdline_D_list = []
    cmdline_nopause = False
    cmdline_version = ''
    cmdline_flash = False
    cmdline_list_targets = False
    cmdline_sequential_files = False
    cmdline_sequential_targets = False
    cmdline_no_clean = False

    cmd_pos = -1
    for cmd in sys.argv:
        cmd_pos += 1
        if cmd == '--target' or cmd == '-t' or cmd == '-T':
            if sys.argv[cmd_pos+1] != '':
                cmdline_target = sys.argv[cmd_pos+1]
        if cmd == '--define' or cmd == '-d' or cmd == '-D':
            if sys.argv[cmd_pos+1] != '':
                cmdline_D_list.append(sys.argv[cmd_pos+1])
        if cmd == '--no-pause' or cmd == '--nopause' or cmd == '-np':
                cmdline_nopause = True
        if cmd == '--version' or cmd == '-v' or cmd == '-V':
            if sys.argv[cmd_pos+1] != '':
                cmdline_version = sys.argv[cmd_pos+1]
        if cmd == '--flash' or cmd == '-f' or cmd == '-F':
                cmdline_flash = True
        if cmd == '--list-targets' or cmd == '-lt' or cmd == '-LT':
                cmdline_list_targets = True
        if cmd == '--sequential-files' or cmd == '-sf' or cmd == '-SF':
                cmdline_sequential_files = True
        if cmd == '--sequential-targets' or cmd == '-st' or cmd == '-ST':
                cmdline_sequential_targets = True
        if cmd == '--no-clean' or cmd == '-nc':
                cmdline_no_clean = True

    #cmdline_target = 'tx-diy-e22dual-module02-g491re'
    #cmdline_target = 'tx-diy-sxdualXXX'

    # Validate arguments before doing anything expensive
    validate_arguments()

    if cmdline_version == '':
        mlrs_set_version()
        mlrs_set_branch_hash(VERSIONONLYSTR)
    else:
        VERSIONONLYSTR = cmdline_version

    # initialize global compilation thread pool
    GLOBAL_COMPILE_POOL = ThreadPoolExecutor(max_workers=COMPILE_POOL_SIZE)

    try:
        # detect build staleness (branch/commit changes)
        build_info_file = os.path.join(MLRS_BUILD_DIR, '.last_build_info')
        force_clean = False
        
        if cmdline_no_clean and os.path.exists(build_info_file):
            # check if build is stale due to branch/commit change
            try:
                with open(build_info_file, 'r') as f:
                    last_branch = f.readline().strip()
                    last_hash = f.readline().strip()
                
                # use subprocess.run for proper error handling
                result = subprocess.run(["git", "branch", "--show-current"],
                                        capture_output=True, text=True)
                current_branch = result.stdout.strip() if result.returncode == 0 else ""
                
                result = subprocess.run(["git", "rev-parse", "HEAD"],
                                        capture_output=True, text=True)
                current_hash = result.stdout.strip() if result.returncode == 0 else ""
                
                # only compare if we got valid git info (current_hash is always set if in a repo)
                if current_hash and (last_branch != current_branch or last_hash != current_hash):
                    force_clean = True
                    print('------------------------------------------------------------')
                    print('Build staleness detected:')
                    if last_branch != current_branch:
                        print(f'  Branch changed: {last_branch} → {current_branch}')
                    if last_hash != current_hash:
                        print(f'  Commit changed: {last_hash[:8]} → {current_hash[:8]}')
                    print('  Forcing clean build to avoid stale artifacts')
                    print('------------------------------------------------------------')
            except (IOError, OSError):
                # if we can't read the file, be safe and force clean
                force_clean = True
        
        # clean build directory unless --no-clean was specified (and build is not stale)
        if cmdline_no_clean and not force_clean:
            create_dir(MLRS_BUILD_DIR)
        else:
            create_clean_dir(MLRS_BUILD_DIR)
        
        # save current build info for next time
        try:
            result = subprocess.run(["git", "branch", "--show-current"],
                                    capture_output=True, text=True)
            current_branch = result.stdout.strip() if result.returncode == 0 else ""
            
            result = subprocess.run(["git", "rev-parse", "HEAD"],
                                    capture_output=True, text=True)
            current_hash = result.stdout.strip() if result.returncode == 0 else ""
            
            # only write if we got valid git info
            if current_hash:
                with open(build_info_file, 'w') as f:
                    f.write(f'{current_branch}\n')
                    f.write(f'{current_hash}\n')
        except (IOError, OSError):
            pass  # non-fatal if we can't write the marker file

        # handle --list-targets early exit
        if cmdline_list_targets:
            print('------------------------------------------------------------')
            print('Available STM32 targets:')
            print('------------------------------------------------------------')
            # create a minimal target list just to get target names
            # we don't need the full appendix/version for listing
            targets = {}
            for t in TLIST:
                base_target = t['target']
                appendix = t['appendix']
                full_name = base_target + appendix
                
                # group by base target
                if base_target not in targets:
                    targets[base_target] = []
                targets[base_target].append(full_name)
            
            # print grouped by base target
            for base_target in sorted(targets.keys()):
                variants = targets[base_target]
                if len(variants) == 1:
                    print(f'  {variants[0]}')
                else:
                    print(f'  {base_target}')
                    for variant in sorted(variants):
                        if variant != base_target:  # don't print duplicate if appendix is empty
                            print(f'    └─ {variant}')
            
            print('------------------------------------------------------------')
            print(f'Total: {sum(len(v) for v in targets.values())} targets')
            print('------------------------------------------------------------')
            
            if not cmdline_nopause:
                input("Press Enter to continue...")
            exit(0)

        targetlist = mlrs_create_targetlist('-'+VERSIONONLYSTR+BRANCHSTR+HASHSTR, [])

        # filter targets to build
        # match against both target.target (base name) and target.build_dir (includes appendix)
        # this allows selecting specific variants like 'tx-matek-mr24-30-g431kb-default'
        targets_to_build = []
        for target in targetlist:
            # check if pattern matches target name or full build name (with appendix)
            matches_target = cmdline_target in target.target
            matches_build_dir = cmdline_target in target.build_dir
            
            if cmdline_target == '':
                # no filter - build all
                targets_to_build.append(target)
            elif cmdline_target[0] == '!':
                # exclusion pattern: exclude if matches target OR build_dir
                pattern = cmdline_target[1:]
                if not (pattern in target.target or pattern in target.build_dir):
                    targets_to_build.append(target)
            else:
                # inclusion pattern: include if matches target OR build_dir
                if matches_target or matches_build_dir:
                    targets_to_build.append(target)
        
        target_cnt = len(targets_to_build)
        built_targets = []
        
        # auto-enable parallel targets for multiple targets (unless explicitly disabled)
        use_parallel_targets = target_cnt > 1 and not cmdline_sequential_targets
        
        # print build info
        print('------------------------------------------------------------')
        print(f'Found {target_cnt} targets to build')
        if use_parallel_targets:
            print(f'Building targets in parallel (Use --sequential-targets to build sequentially)')
        else:
            print(f'Building targets sequentially')
        print('------------------------------------------------------------')
        
        # build targets - either in parallel or sequentially
        build_start_time = time.time()
        if use_parallel_targets:
            
            def build_single_target(target):
                try:
                    skip_hex = cmdline_flash
                    build_info = mlrs_build_target(target, cmdline_D_list, sequential=cmdline_sequential_files, skip_hex=skip_hex)
                    return (True, build_info, None)
                except Exception as e:
                    return (False, target, str(e))
            
            # use ThreadPoolExecutor to orchestrate multiple targets concurrently.
            # this allows target linking to overlap with other targets' compilation.
            # actual compilation work is handled by the shared GLOBAL_COMPILE_POOL.
            max_workers = min(multiprocessing.cpu_count() * 2, target_cnt)
            
            built_count = 0
            failed_count = 0
            failed_targets = []
            build_info_list = []
            
            with ThreadPoolExecutor(max_workers=max_workers) as executor:
                futures = {executor.submit(build_single_target, target): target 
                          for target in targets_to_build}
                
                for future in as_completed(futures):
                    success, result, error = future.result()
                    if success:
                        built_count += 1
                        build_info_list.append(result)
                        built_targets.append(result['target'])
                    else:
                        failed_count += 1
                        failed_targets.append(result)
                        printError(f'[FAIL] [{built_count + failed_count}/{target_cnt}] {result.target} FAILED')
                        if error:
                            print(f'  Error: {error[:200]}')
            
            # display build summary using helper function
            display_build_summary(build_info_list, build_start_time, failed_targets)
        else:
            build_info_list = []
            for idx, target in enumerate(targets_to_build, 1):
                build_info = mlrs_build_target(target, cmdline_D_list, sequential=cmdline_sequential_files, skip_hex=cmdline_flash)
                build_info_list.append(build_info)
                built_targets.append(target)
            
            # display build summary using helper function
            display_build_summary(build_info_list, build_start_time)

        if cmdline_target == '' or target_cnt > 0:
            mlrs_copy_all_hex_etc()

        # flash with auto-detection if requested
        if cmdline_flash and len(built_targets) == 1:
            target = built_targets[0]
            elf_file = os.path.join(MLRS_BUILD_DIR, target.build_dir, target.elf_name + '.elf')
            if os.path.exists(elf_file):
                print('------------------------------------------------------------')
                print('Auto-detecting flash method (STM32CubeProgrammer CLI)...')
                print('------------------------------------------------------------')
                
                # find STM32CubeProgrammer CLI
                cube_programmer_cli = find_stm32_cube_programmer()
                if not cube_programmer_cli:
                    printError('Error: STM32CubeProgrammer CLI not found!')
                    print('Please install STM32CubeCLT or STM32CubeIDE')
                else:
                    flash_auto(cube_programmer_cli, elf_file)
            else:
                printError(f'Error: .elf file not found: {elf_file}')
        elif cmdline_flash and len(built_targets) != 1:
            printError('Warning: --flash requires exactly one target. Built ' + str(len(built_targets)) + ' targets.')

        if not cmdline_nopause:
            input("Press Enter to continue...")
    
    finally:
        # always clean up global thread pool, even if an exception occurred
        GLOBAL_COMPILE_POOL.shutdown(wait=True)
