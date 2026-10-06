#!/usr/bin/env python3
"""Enroll a fresh per-boot key over direct trusted UART; never print the secret."""
import argparse
import json
import hashlib
import os
from pathlib import Path
import secrets
import time
from pyflix.client import Flix
from pyflix.serial import SerialTransport

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', required=True)
    parser.add_argument('--session-file', type=Path)
    parser.add_argument('--qgc-passphrase-file',type=Path,help='Save a fresh private QGC signing passphrase; key is its SHA-256 digest')
    parser.add_argument('--revoke',action='store_true')
    args=parser.parse_args()
    if args.revoke:
        with Flix(SerialTransport(args.device)) as client:
            client.disarm();client.console('maintenance')
            if client.status()!='maintenance':raise RuntimeError('maintenance unavailable')
            client.revoke_radio();client.console('exit')
        print('Radio session revoked; fresh enrollment required to enable radio.')
        return
    if args.session_file is None:parser.error('--session-file required for enrollment')
    phrase=secrets.token_hex(32) if args.qgc_passphrase_file else None
    key=hashlib.sha256(phrase.encode()).digest() if phrase else secrets.token_bytes(32)
    timestamp=int((time.time()-1420070400)*100000)
    # Create exclusively before enrollment: failure cannot overwrite a working session.
    fd=os.open(args.session_file,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
    enrolled=False
    phrase_created=False
    try:
        if phrase is not None:
            phrase_fd=os.open(args.qgc_passphrase_file,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
            phrase_created=True
            with os.fdopen(phrase_fd,'w') as stream:
                stream.write(phrase);stream.flush();os.fsync(stream.fileno())
        with Flix(SerialTransport(args.device)) as client:
            client.disarm(); client.console('maintenance')
            deadline=time.monotonic()+2
            while client.status()!='maintenance':
                if time.monotonic()>=deadline: raise RuntimeError('maintenance unavailable')
            client.enroll_radio(key,timestamp)
            try:
                data=json.dumps({'version':1,'key':key.hex(),'timestamp':timestamp,'scope':'per-boot-development'}).encode()
                while data:
                    count=os.write(fd,data)
                    if count<=0:raise OSError('session file write failed')
                    data=data[count:]
                os.fsync(fd)
            except BaseException:
                # Still in stopped maintenance. A lost cable can prevent revoke;
                # reboot restores the unprovisioned state in that case.
                try:client.revoke_radio()
                except Exception:pass
                raise
            client.console('exit')
            enrolled=True
        print('Radio session enrolled; private session file saved. Re-enroll after reboot.')
    finally:
        os.close(fd)
        if not enrolled:
            args.session_file.unlink(missing_ok=True)
            if phrase_created:args.qgc_passphrase_file.unlink(missing_ok=True)

if __name__=='__main__':main()
