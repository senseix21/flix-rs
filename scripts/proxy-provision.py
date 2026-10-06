#!/usr/bin/env python3
"""Create a private, offline proxy NVS image. Never flashes or enrolls a flight board."""
import argparse
import csv
import json
import os
from pathlib import Path
import stat
import subprocess
import sys
import tempfile

def validate(value):
    if set(value)!={'peer_mac','pmk','lmk','channel'}:raise ValueError('expected peer_mac, pmk, lmk, channel')
    peer=bytes.fromhex(value['peer_mac'].replace(':',''))
    pmk=bytes.fromhex(value['pmk']);lmk=bytes.fromhex(value['lmk'])
    if len(peer)!=6 or peer==b'\0'*6 or peer[0]&1:raise ValueError('peer must be a nonzero unicast MAC')
    if len(pmk)!=16 or len(lmk)!=16 or not any(pmk) or not any(lmk):raise ValueError('PMK and LMK must each contain 16 nonzero-key bytes')
    channel=value['channel']
    if type(channel) is not int or not 1<=channel<=14:raise ValueError('channel must be an integer in 1..14')
    return peer,pmk,lmk,channel

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--role',choices=['proxy','flight'],default='proxy');p.add_argument('--config',type=Path,required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    mode=args.config.stat().st_mode
    if not stat.S_ISREG(mode) or mode&0o077:p.error('configuration must be a regular file readable only by its owner (chmod 600)')
    try:peer,pmk,lmk,channel=validate(json.loads(args.config.read_text()))
    except (ValueError,TypeError,KeyError):p.error('invalid proxy configuration; check exact fields, key lengths, unicast MAC and channel')
    sdk=Path(os.environ.get('IDF_PATH',''))
    generator=sdk/'components/nvs_flash/nvs_partition_generator/nvs_partition_gen.py'
    if not generator.is_file():p.error('activate the pinned ESP-IDF v5.4.2 environment first')
    with tempfile.TemporaryDirectory() as directory:
        source=Path(directory)/'proxy.csv';image=Path(directory)/'proxy.bin'
        with source.open('w',newline='') as file:
            writer=csv.writer(file);writer.writerow(['key','type','encoding','value']);writer.writerow(['flixproxy' if args.role=='proxy' else 'flixespnow','namespace','',''])
            for key,data in [('peer',peer),('pmk',pmk),('lmk',lmk)]:writer.writerow([key,'data','hex2bin',data.hex()])
            writer.writerow(['channel','data','u8',channel])
        source.chmod(0o600)
        result=subprocess.run([sys.executable,str(generator),'generate',str(source),str(image),'0x6000'],stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        if result.returncode:p.error('NVS generator failed; confirm the SDK Python environment (secret output suppressed)')
        with os.fdopen(os.open(args.output,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600),'wb') as out:out.write(image.read_bytes())
    print('Private transport image created; no device was flashed.')
if __name__=='__main__':main()
