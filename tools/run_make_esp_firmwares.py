#!/usr/bin/env python
'''
*******************************************************
 Copyright (c) MLRS project
 GPL3
 https://www.gnu.org/licenses/gpl-3.0.de.html
 OlliW @ www.olliw.eu
*******************************************************
 run_make_esp_firmwares.py
 generate esp fimrware files
 renames and copies files into tools/esp-build/firmware
 version 27.12.2025
********************************************************

COMMAND LINE ARGUMENTS:
  --target, -t, -T <target_name>
      Build only the specified target environment.
      If omitted, all environments defined in platformio.ini are built.
      Note: Currently parsed but not fully implemented in build logic.
      Example: --target esp32-wroom

  --define, -d, -D <DEFINE>
      Add a preprocessor define to the build (can be specified multiple times).
      Each occurrence adds another define to the compilation.
      Note: Currently parsed but not fully implemented in build logic.
      Example: --define DEBUG_MODE

  --nopause, -np
      Skip the "Press Enter to continue..." prompt at the end of execution.
      Useful for automated builds or CI/CD pipelines.

  --version, -v, -V <version_string>
      Override the version string from common_conf.h.
      If omitted, the version is read from VERSIONONLYSTR in common_conf.h.
      The version is used in the output .bin filename.
      Example: --version 1.2.3

  --no-clean, -nc
      Skip the full clean step before building.
      By default, a full clean is performed before each build.
      Use this flag for incremental builds (faster but may have stale artifacts).

  --file-jobs, -fj <number>
      Number of parallel file compilation jobs per target.
      Controls file-level parallelism within each PlatformIO environment.
      Default: min(cpu_count, 8)
      Example: --file-jobs 4

  --target-jobs, -tj <number>
      Number of parallel target builds.
      Controls how many PlatformIO environments are built concurrently.
      Default: min(4, cpu_count) - capped at 4 to avoid overwhelming memory/disk I/O.
      Example: --target-jobs 2

  --list-targets, -lt, -LT
      List all available ESP32 targets and exit without building.
      Prints environment names from platformio.ini.
      Use this to discover correct target names for --target flag.

  --flash, -f, -F
      Flash the built firmware via USB serial after successful build.
      Automatically uses PlatformIO's upload command.
      Requires exactly one target to be built (use --target flag).
      Flashing baud rate: 921600 (configured in platformio.ini).
      Example: --target rx-generic-2400-d-pa --flash

USAGE EXAMPLES:
  # Build all ESP targets with default settings (clean + parallel)
  python run_make_esp_firmwares.py

  # Build with incremental compilation (no clean)
  python run_make_esp_firmwares.py --no-clean

  # Build with custom parallelism settings
  python run_make_esp_firmwares.py --file-jobs 4 --target-jobs 2

  # Build with custom version and no pause (for CI/CD)
  python run_make_esp_firmwares.py --version 1.2.3 --nopause

  # Fast incremental build
  python run_make_esp_firmwares.py --no-clean --nopause

  # Build and flash a specific target
  python run_make_esp_firmwares.py --target rx-generic-2400-d-pa --flash

NOTES:
  - Uses PlatformIO to build all environments defined in platformio.ini
  - Compiled .bin files are copied to tools/esp-build/firmware/ with version suffix
  - Parallel builds significantly speed up compilation time
  - Clean builds ensure no stale artifacts but take longer

********************************************************
'''
import os
import shutil
import re
import sys
import subprocess
import multiprocessing
import time
import configparser
from concurrent.futures import ThreadPoolExecutor, as_completed


#-- installation dependent
# Auto-detect platformio

GLOBAL_BUILD_POOL = None
BUILD_POOL_SIZE = max(2, multiprocessing.cpu_count() // 2)

def find_platformio():
    """Find platformio executable in system PATH or common locations"""
    
    # First try to find in PATH
    pio_cmd = shutil.which('platformio')
    if pio_cmd:
        return 'platformio'
    
    pio_cmd = shutil.which('pio')
    if pio_cmd:
        return 'pio'
    
    # Try common install locations
    if os.name == 'posix':  # Mac/Linux
        common_paths = [
            os.path.expanduser('~/.platformio/penv/bin/platformio'),
            os.path.expanduser('~/.local/bin/platformio'),
            '/usr/local/bin/platformio',
        ]
    else:  # Windows
        common_paths = [
            os.path.join("C:/", 'Users', os.getenv('USERNAME', 'Olli'), '.platformio', 'penv', 'Scripts', 'platformio.exe'),
        ]
    
    for path in common_paths:
        if os.path.exists(path):
            return path
    
    return None


PIO_CMD = find_platformio()

if PIO_CMD is None:
    print('ERROR: platformio not found in PATH')
    print('Install with: pip install platformio')
    print('Or: brew install platformio (macOS)')
    sys.exit(1)

print(f'Using platformio: {PIO_CMD}')



#-- mLRS directories

MLRS_PROJECT_DIR = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

MLRS_DIR = os.path.join(MLRS_PROJECT_DIR,'mLRS')

MLRS_TOOLS_DIR = os.path.join(MLRS_PROJECT_DIR,'tools')
MLRS_BUILD_DIR = os.path.join(MLRS_PROJECT_DIR,'tools','build3')

MLRS_PIO_BUILD_DIR = os.path.join(MLRS_PROJECT_DIR,'.pio','build')
MLRS_ESP_BUILD_DIR = os.path.join(MLRS_PROJECT_DIR,'tools','esp-build')



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
    import subprocess

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
        '--nopause', '-np',
        '--version', '-v', '-V',
        '--no-clean', '-nc',
        '--file-jobs', '-fj',
        '--target-jobs', '-tj',
        '--list-targets', '-lt', '-LT',
        '--flash', '-f', '-F',
    }
    
    # Flags that expect a value argument
    value_flags = {
        '--target', '-t', '-T',
        '--define', '-d', '-D',
        '--version', '-v', '-V',
        '--file-jobs', '-fj',
        '--target-jobs', '-tj',
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


#--------------------------------------------------
# build system
#--------------------------------------------------

def get_platformio_environments():
    """Parse platformio.ini to get list of build environments"""
    
    config = configparser.ConfigParser()
    config.read(os.path.join(MLRS_PROJECT_DIR, 'platformio.ini'))
    
    environments = []
    for section in config.sections():
        if section.startswith('env:'):
            env_name = section[4:]  # Remove 'env:' prefix
            environments.append(env_name)
    
    return environments


def build_single_environment(env_name, file_jobs, clean):
    """Build a single PlatformIO environment"""
    start_time = time.time()
    try:
        if clean:
            # "Fast Clean": directly delete the environment's build directory in .pio/build.
            # This avoids the PlatformIO "startup tax" (2-4s) of a separate 'pio run -t clean' call.
            env_build_dir = os.path.join(MLRS_PIO_BUILD_DIR, env_name)
            if os.path.exists(env_build_dir):
                shutil.rmtree(env_build_dir, ignore_errors=True)
        
        # Build the environment with file-level parallelism
        build_cmd = [PIO_CMD, 'run', '--project-dir', MLRS_PROJECT_DIR, 
                    '-e', env_name, '-j', str(file_jobs)]
        
        result = subprocess.run(build_cmd, capture_output=True, text=True)
        
        elapsed = time.time() - start_time
        if result.returncode == 0:
            return (True, env_name, None, elapsed)
        else:
            return (False, env_name, result.stderr, elapsed)
    except Exception as e:
        elapsed = time.time() - start_time
        return (False, env_name, str(e), elapsed)


def display_build_summary(build_info_list, build_start_time):
    """Display build summary with compilation times.
    
    Args:
        build_info_list: List of dicts with 'env_name', 'compilation_time', and 'success'
        build_start_time: Start time for total elapsed calculation
    """
    total_time = time.time() - build_start_time
    
    print('------------------------------------------------------------')
    print('BUILD SUMMARY')
    print('------------------------------------------------------------')
    for info in build_info_list:
        status = '[OK]' if info['success'] else '[FAIL]'
        print(f"{status} {info['env_name']}: {info['compilation_time']:.2f}s")
    
    print('------------------------------------------------------------')
    
    failed_envs = [info['env_name'] for info in build_info_list if not info['success']]
    if failed_envs:
        printError(f'Build completed with {len(failed_envs)} failures in {total_time:.2f}s:')
        for env in failed_envs:
            printError(f'  - {env}')
    else:
        print(f'[OK] All {len(build_info_list)} environments built successfully in {total_time:.2f}s')
    print('------------------------------------------------------------')


def mlrs_esp_compile_all(clean=False, file_jobs=None, target_jobs=None, target_filter=''):
    """Compile all ESP targets using platformio with parallel builds
    
    Args:
        clean: Whether to do a full clean before building
        file_jobs: Number of parallel file compilation jobs per target
        target_jobs: Number of parallel target builds
        target_filter: Optional filter to build only specific targets (environment names)
    
    Returns:
        List of successfully built environment names
    """
    # Get all environments from platformio.ini
    environments = get_platformio_environments()
    
    # Filter environments if target_filter is specified
    if target_filter:
        environments = [env for env in environments if target_filter in env]
        if not environments:
            printError(f'No environments match target filter: {target_filter}')
            return []
    
    # Coordinated parallelism: limit total concurrent jobs to cpu_count * 2
    # This allows I/O overlap while preventing excessive context switching
    total_concurrent = multiprocessing.cpu_count() * 2
    
    # Calculate parallelism based on number of environments
    num_envs = len(environments)
    
    if target_jobs is None:
        # Target-level parallelism: scale based on CPU count
        # Using cpu_count // 2 balances target vs file parallelism across different machines
        target_jobs = max(2, min(num_envs, multiprocessing.cpu_count() // 2))
    
    if file_jobs is None:
        # File-level parallelism: distribute available capacity across concurrent targets
        # This ensures full utilization even if building many targets (where target_jobs is capped at 4)
        file_jobs = max(2, total_concurrent // target_jobs)
    
    total_envs = len(environments)
    
    print(f'Found {total_envs} environments to build')
    
    # Determine parallelism based on number of targets
    use_parallel = total_envs > 1
    if use_parallel:
        print(f'Using {target_jobs} parallel targets, {file_jobs} parallel files per target')
    else:
        print(f'Building single target with {file_jobs} parallel files')
    print('------------------------------------------------------------')
    
    print(f'Building {total_envs} environment(s)...')
    
    # Build environments
    build_start_time = time.time()
    build_info_list = []
    
    if use_parallel:
        # Parallel build for multiple targets
        # Submit all build tasks
        futures = {GLOBAL_BUILD_POOL.submit(build_single_environment, env, file_jobs, clean): env 
                  for env in environments}
        
        # Collect results as they complete
        for future in as_completed(futures):
            env = futures[future]
            try:
                success, env_name, error, elapsed = future.result()
                build_info_list.append({
                    'env_name': env_name,
                    'success': success,
                    'compilation_time': elapsed
                })
                if success:
                    print(f'[OK] [{len(build_info_list)}/{total_envs}] {env_name}')
                else:
                    printError(f'[FAIL] [{len(build_info_list)}/{total_envs}] {env_name} FAILED')
                    if error:
                        print(f'  Error: {error[:200]}')  # Show first 200 chars of error
            except Exception as exc:
                build_info_list.append({
                    'env_name': env,
                    'success': False,
                    'compilation_time': 0
                })
                printError(f'[FAIL] {env} generated exception: {exc}')
    else:
        # Sequential build for single target
        for env in environments:
            success, env_name, error, elapsed = build_single_environment(env, file_jobs, clean)
            build_info_list.append({
                'env_name': env_name,
                'success': success,
                'compilation_time': elapsed
            })
            if success:
                print(f'[OK] {env_name}')
            else:
                printError(f'[FAIL] {env_name} FAILED')
                if error:
                    print(f'  Error: {error[:200]}')
    
    display_build_summary(build_info_list, build_start_time)
    
    # Return list of successfully built environments
    return [info['env_name'] for info in build_info_list if info['success']]


def flash_esp_target(env_name):
    """Flash ESP target using PlatformIO upload command at 921600 baud"""
    
    print('------------------------------------------------------------')
    print(f'Flashing environment: {env_name}')
    print(f'Upload speed: 921600 baud (from platformio.ini)')
    print()
    
    # Build PlatformIO upload command
    cmd = [
        PIO_CMD,
        'run',
        '--project-dir', MLRS_PROJECT_DIR,
        '-e', env_name,
        '--target', 'upload'
    ]
    
    print(f'Executing: {" ".join(cmd)}')
    print()
    
    start_time = time.time()
    result = subprocess.run(cmd, capture_output=False, text=True)
    elapsed = time.time() - start_time
    
    if result.returncode == 0:
        print()
        print(f'[OK] Flash completed successfully in {elapsed:.1f}s')
        return True
    else:
        printError(f'[FAIL] Flash failed with return code {result.returncode}')
        return False



#--------------------------------------------------
# application
#--------------------------------------------------

def mlrs_esp_copy_all_bin(environments):
    print('copying .bin files')
    firmwarepath = os.path.join(MLRS_ESP_BUILD_DIR,'firmware')
    create_clean_dir(firmwarepath)
    for subdir in environments:
        env_dir = os.path.join(MLRS_PIO_BUILD_DIR, subdir)
        if os.path.isdir(env_dir):
            print(subdir)
            file = os.path.join(env_dir, 'firmware.bin')
            if os.path.exists(file):
                dest_file = os.path.join(firmwarepath, subdir + '-' + VERSIONONLYSTR + BRANCHSTR + HASHSTR + '.bin')
                shutil.copy(file, dest_file)
            else:
                printWarning(f'Warning: firmware.bin not found for {subdir}')


#-- here we go
if __name__ == "__main__":
    cmdline_target = ''
    cmdline_D_list = []
    cmdline_nopause = False
    cmdline_version = ''
    cmdline_clean = True  # Clean by default
    cmdline_file_jobs = None
    cmdline_target_jobs = None
    cmdline_flash = False
    cmdline_list_targets = False

    cmd_pos = -1
    for cmd in sys.argv:
        cmd_pos += 1
        if cmd == '--target' or cmd == '-t' or cmd == '-T':
            if sys.argv[cmd_pos+1] != '':
                cmdline_target = sys.argv[cmd_pos+1]
        if cmd == '--define' or cmd == '-d' or cmd == '-D':
            if sys.argv[cmd_pos+1] != '':
                cmdline_D_list.append(sys.argv[cmd_pos+1])
        if cmd == '--nopause' or cmd == '-np':
                cmdline_nopause = True
        if cmd == '--version' or cmd == '-v' or cmd == '-V':
            if sys.argv[cmd_pos+1] != '':
                cmdline_version = sys.argv[cmd_pos+1]
        if cmd == '--no-clean' or cmd == '-nc':
                cmdline_clean = False
        if cmd == '--file-jobs' or cmd == '-fj':
            if cmd_pos + 1 < len(sys.argv) and sys.argv[cmd_pos+1].isdigit():
                cmdline_file_jobs = int(sys.argv[cmd_pos+1])
        if cmd == '--target-jobs' or cmd == '-tj':
            if cmd_pos + 1 < len(sys.argv) and sys.argv[cmd_pos+1].isdigit():
                cmdline_target_jobs = int(sys.argv[cmd_pos+1])
        if cmd == '--flash' or cmd == '-f' or cmd == '-F':
                cmdline_flash = True
        if cmd == '--list-targets' or cmd == '-lt' or cmd == '-LT':
                cmdline_list_targets = True

    # Validate arguments before doing anything expensive
    validate_arguments()

    # initialize global compilation thread pool
    GLOBAL_BUILD_POOL = ThreadPoolExecutor(max_workers=BUILD_POOL_SIZE)

    try:
        # Handle --list-targets early exit
        if cmdline_list_targets:
            print('------------------------------------------------------------')
            print('Available ESP32 targets:')
            print('------------------------------------------------------------')
            environments = get_platformio_environments()
            for env in sorted(environments):
                print(f'  {env}')
            print('------------------------------------------------------------')
            print(f'Total: {len(environments)} target(s)')
            sys.exit(0)
        
        if cmdline_version == '':
            mlrs_set_version()
            mlrs_set_branch_hash(VERSIONONLYSTR)
        else:
            VERSIONONLYSTR = cmdline_version

        # Validate flash requirement: must have exactly one target
        if cmdline_flash and cmdline_target == '':
            printError('Error: --flash requires --target to specify exactly one target')
            printError('Example: python run_make_esp_firmwares.py --target rx-generic-2400-d-pa --flash')
            sys.exit(1)
        
        built_environments = mlrs_esp_compile_all(
            clean=cmdline_clean, 
            file_jobs=cmdline_file_jobs, 
            target_jobs=cmdline_target_jobs,
            target_filter=cmdline_target
        )
        
        if built_environments:
            mlrs_esp_copy_all_bin(built_environments)
        
        # Flash via PlatformIO if requested
        if cmdline_flash and len(built_environments) == 1:
            flash_esp_target(built_environments[0])
        elif cmdline_flash and len(built_environments) != 1:
            printError(f'Error: --flash requires exactly one target. Built {len(built_environments)} targets.')
            printError('Use --target to specify a single environment.')

        if not cmdline_nopause:
            if os.name == 'posix':
                input("Press Enter to continue...")
            else:
                os.system("pause")
    finally:
        GLOBAL_BUILD_POOL.shutdown(wait=True)
