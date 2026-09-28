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
 -t <target> builds only the envs whose name contains target, e.g. -t elrs
 version 21.03.2026
********************************************************
'''
import os
import pathlib
import shutil
import re
import sys
import subprocess


#-- installation dependent

def find_pio():
    # PlatformIO CLI from the PATH, else from its default install location
    for name in ('pio', 'platformio'):
        pio = shutil.which(name)
        if pio:
            return pio
    core_dir = os.environ.get('PLATFORMIO_CORE_DIR', os.path.join(os.path.expanduser('~'),'.platformio'))
    scripts_dir = os.path.join(core_dir,'penv','Scripts' if os.name == 'nt' else 'bin')
    for name in ('pio', 'platformio'):
        pio = os.path.join(scripts_dir, name + ('.exe' if os.name == 'nt' else ''))
        if os.path.exists(pio):
            return pio
    print('ERROR: PlatformIO not found, install it or add it to the PATH')
    exit(1)



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

    v_patch = int(version_str.split('.')[2])

    git_branch = subprocess.getoutput("git branch --show-current")
    if not git_branch == 'main' and v_patch != 0: # is a branch, but not a main release
        BRANCHSTR = '-'+git_branch
    if BRANCHSTR != '':
        print('BRANCHSTR =', BRANCHSTR)

    git_hash = subprocess.getoutput("git rev-parse --short HEAD")
    if v_patch % 2 == 1: # odd firmware patch version, so is dev, so add git hash
        HASHSTR = '-@'+git_hash
    if HASHSTR != '':
        print('HASHSTR =', HASHSTR)


#-- helper

def remake_dir(path): # os dependent
    if os.name == 'posix':
        os.system('rm -r -f '+path)
    else:
        os.system('rmdir /s /q "'+path+'"')

def make_dir(path): # os dependent
    if os.name == 'posix':
        os.system('mkdir -p '+path)
    else:
        os.system('md "'+path+'"')


def create_dir(path):
    if not os.path.exists(path):
        make_dir(path)

def erase_dir(path):
    if os.path.exists(path):
        remake_dir(path)

def create_clean_dir(path):
    if os.path.exists(path):
        remake_dir(path)
    make_dir(path)


def printWarning(txt):
    print('\033[93m'+txt+'\033[0m') # light Yellow


def printError(txt):
    print('\033[91m'+txt+'\033[0m') # light Red



#--------------------------------------------------
# build system
#--------------------------------------------------

def mlrs_esp_update_elrs_targets():
    # regenerate the ELRS receiver envs, so they match the tools/elrs/targets submodule
    sys.path.insert(0, os.path.join(MLRS_TOOLS_DIR,'elrs'))
    import elrs_targets
    ini = elrs_targets.PIO_INI
    old = open(ini).read() if os.path.exists(ini) else ''
    elrs_targets.write_pio_ini(elrs_targets.load_products())
    if open(ini).read() != old:
        printWarning('platformio_elrs.ini has changed, needs to be committed')


def mlrs_esp_get_envs(target):
    # envs whose name contains target, all envs if target is empty
    envs = []
    for ini in ('platformio.ini', 'platformio_elrs.ini'):
        F = open(os.path.join(MLRS_PROJECT_DIR,ini), mode='r')
        envs += re.findall(r'^\[env:([^\]]+)\]', F.read(), re.MULTILINE)
        F.close()
    return [e for e in envs if target in e.lower()]


def mlrs_esp_compile_all(envs):
    # argument list, not a shell string, so paths with spaces work on all OSes
    pio_run = [find_pio(), 'run', '--project-dir', MLRS_PROJECT_DIR]
    for e in envs:
        pio_run += ['-e', e]

    print('Full Clean All')
    subprocess.call(pio_run + ['--target', 'fullclean'])
    print('Build All')
    subprocess.call(pio_run)



#--------------------------------------------------
# application
#--------------------------------------------------

def mlrs_esp_copy_all_bin(envs):
    print('copying .bin files')
    firmwarepath = os.path.join(MLRS_ESP_BUILD_DIR,'firmware')
    create_clean_dir(firmwarepath)
    for subdir in os.listdir(MLRS_PIO_BUILD_DIR):
        if envs and subdir not in envs: # only copy what was built
            continue
        if os.path.isdir(os.path.join(MLRS_PIO_BUILD_DIR,subdir)): # needs to use full path for the check to work
            print(subdir)
            file = os.path.join(MLRS_PIO_BUILD_DIR,subdir,'firmware.bin')
            shutil.copy(file, os.path.join(firmwarepath,subdir+'-'+VERSIONONLYSTR+BRANCHSTR+HASHSTR+'.bin'))


#-- here we go
if __name__ == "__main__":
    cmdline_target = ''
    cmdline_D_list = []
    cmdline_nopause = False
    cmdline_version = ''

    cmd_pos = -1
    for cmd in sys.argv:
        cmd_pos += 1
        if cmd == '--target' or cmd == '-t' or cmd == '-T':
            if sys.argv[cmd_pos+1] != '':
                cmdline_target = sys.argv[cmd_pos+1].lower() # target matching is case insensitive
        if cmd == '--define' or cmd == '-d' or cmd == '-D':
            if sys.argv[cmd_pos+1] != '':
                cmdline_D_list.append(sys.argv[cmd_pos+1])
        if cmd == '--nopause' or cmd == '-np':
                cmdline_nopause = True
        if cmd == '--version' or cmd == '-v' or cmd == '-V':
            if sys.argv[cmd_pos+1] != '':
                cmdline_version = sys.argv[cmd_pos+1]

    if cmdline_version == '':
        mlrs_set_version()
        mlrs_set_branch_hash(VERSIONONLYSTR)
    else:
        VERSIONONLYSTR = cmdline_version

    mlrs_esp_update_elrs_targets()

    envs = []
    if cmdline_target != '':
        envs = mlrs_esp_get_envs(cmdline_target)
        if not envs:
            printError('no env matches target '+cmdline_target)
            exit(1)
        print('building', len(envs), 'envs')

    mlrs_esp_compile_all(envs)
    mlrs_esp_copy_all_bin(envs)

    if not cmdline_nopause:
        os.system("pause")
