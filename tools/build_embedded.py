#!/usr/bin/env python3
"""Test and package the Pico power master and optional ESP32 panel slave. No flashing."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]

def run(args,**kw):subprocess.run([str(a) for a in args],cwd=ROOT,check=True,**kw)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sdk',default=os.environ.get('PICO_SDK_PATH'),help='Installed Pico SDK path')
    parser.add_argument('--toolchain',default=os.environ.get('PICO_TOOLCHAIN_PATH'))
    parser.add_argument('--picotool-dir',default=os.environ.get('picotool_DIR'))
    parser.add_argument('--with-esp32',action='store_true',help='Also build the unchanged ESP32 panel slave')
    args=parser.parse_args()
    build=ROOT/'artifacts/build/pico_power_master'
    out=ROOT/'artifacts/firmware/power-controller'
    run([sys.executable,'-m','unittest','discover','-s','embedded/tests','-v'])
    configure=['cmake','-S','embedded/pico_power_master','-B',build,'-DPICO_BOARD=pico']
    for key,value in [('PICO_SDK_PATH',args.sdk),('PICO_TOOLCHAIN_PATH',args.toolchain),('picotool_DIR',args.picotool_dir)]:
        if value:configure.append(f'-D{key}={Path(value).expanduser().resolve()}')
    run(configure)
    run(['cmake','--build',build,'--parallel','4'])
    files=[(build/'pico_power_master.uf2',None),(build/'pico_power_master.elf',None)]
    if args.with_esp32:
        pio=shutil.which('pio') or str(Path.home()/'.platformio/penv/bin/pio')
        core=ROOT/'artifacts/build/platformio-core'
        for kind in ('platforms','packages'):
            target=core/kind;target.mkdir(parents=True,exist_ok=True)
            installed=Path.home()/'.platformio'/kind
            if installed.is_dir():
                for source in installed.iterdir():
                    link=target/source.name
                    if source.is_dir() and not link.exists() and not link.is_symlink():link.symlink_to(source,target_is_directory=True)
        env=dict(os.environ,PLATFORMIO_CORE_DIR=str(core),PLATFORMIO_SETTING_ENABLE_TELEMETRY='no')
        run([pio,'run','-d','embedded/esp32_panel_slave'],env=env)
        esp=ROOT/'embedded/esp32_panel_slave/.pio/build/esp32_panel_slave'
        files.extend([(esp/'bootloader.bin','0x1000'),(esp/'partitions.bin','0x8000'),
            (core/'packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin','0xe000'),
            (esp/'firmware.bin','0x10000')])
    out.mkdir(parents=True,exist_ok=True)
    manifest={'power_protocol':1,'host_watchdog_ms':750,'remote_power_on':False,
              'native_tests':'passed','hardware_validation':'not established by this build',
              'images':[],'sources':{}}
    for source,offset in files:
        name='esp32_panel_slave.bin' if source.name=='firmware.bin' else source.name
        target=out/name;shutil.copyfile(source,target)
        manifest['images'].append({'file':name,'offset':offset,'bytes':target.stat().st_size,
            'sha256':hashlib.sha256(target.read_bytes()).hexdigest()})
    for folder in ('pico_power_master','esp32_panel_slave'):
        for path in sorted((ROOT/'embedded'/folder).rglob('*')):
            if path.is_file() and '.pio' not in path.parts:
                manifest['sources'][str(path.relative_to(ROOT))]=hashlib.sha256(path.read_bytes()).hexdigest()
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print('Packaged:',out)

if __name__=='__main__':main()
