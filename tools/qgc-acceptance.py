#!/usr/bin/env python3
"""Run the actual macOS QGC executable against a loopback signed Rust bench.

Uses a private XDG settings directory. Refuses an existing running QGC.
Logs only admitted message counts; enrollment and signing settings stay private.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import secrets
import struct
import socket
import threading
from collections import Counter
import subprocess
import time
from pyflix.client import PARAMETER_COUNT, pack

ROOT=Path(__file__).resolve().parents[1]

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable',type=Path,required=True)
    parser.add_argument('--trusted-local',action='store_true',help='Use explicit host-only unsigned BenchUsb policy for QGC Ethernet/USB links')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if subprocess.run(['pgrep','-x','QGroundControl'],capture_output=True).returncode==0:
        raise RuntimeError('close existing QGroundControl before isolated acceptance')
    args.output.mkdir(parents=True,exist_ok=False)
    phrase=secrets.token_hex(32);key=hashlib.sha256(phrase.encode()).digest()
    stamp=int((time.time()-1420070400)*100000)
    enrollment=args.output/'private.enrollment.frame'
    fd=os.open(enrollment,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
    with os.fdopen(fd,'wb') as stream:stream.write(pack(256,struct.pack('<QBB32s',stamp,1,1,key)))
    config=args.output.resolve()/'private-config'
    paths=[config/'qgroundcontrol.com/QGroundControl.ini']
    saved={p:(p.read_bytes(),p.stat().st_mode&0o777) if p.exists() else None for p in paths}
    process=bench=None
    observed=Counter();parameter_indices=set();stopping=threading.Event()
    relay=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);relay.bind(('127.0.0.1',14554));relay.settimeout(.1)
    def forward(endpoint):
        while not stopping.is_set():
            try:data,source=relay.recvfrom(4096)
            except socket.timeout:continue
            if source==endpoint:
                if len(data)>=35 and data[0]==253 and int.from_bytes(data[7:10],'little')==22:
                    parameter_indices.add(int.from_bytes(data[16:18],'little'))
                relay.sendto(data,('127.0.0.1',14553));continue
            if source!=('127.0.0.1',14553):continue
            if len(data)>=10 and data[0]==253:
                message=int.from_bytes(data[7:10],'little')
                valid=bool(data[2]&1) and len(data)>=25 and hashlib.sha256(key+data[:-6]).digest()[:6]==data[-6:]
                delta=int.from_bytes(data[-12:-6],'little')-stamp if data[2]&1 else 0
                observed[(data[5],data[6],message,bool(data[2]&1),valid,'future' if delta>=0 else 'before-enrollment',int.from_bytes(data[38:40],'little') if message==76 else None,struct.unpack_from('<f',data,10)[0] if message==76 else None)]+=1
            else:observed[('non-v2',data[0] if data else -1,data[5] if len(data)>5 else -1)]+=1
            relay.sendto(data,endpoint)
    try:
        for path in paths:
            path.parent.mkdir(parents=True,exist_ok=True)
            fd=os.open(path,os.O_WRONLY|os.O_CREAT|os.O_TRUNC,0o600)
            with os.fdopen(fd,'w') as stream:
                stream.write('[General]\nmavlink2SigningKey='+('' if args.trusted_local else phrase)+'\ntelemetrySave=false\n\n[AutoConnect]\nudpListenPort=14553\n')
        environment=os.environ.copy();environment.update(XDG_CONFIG_HOME=str(config),XDG_CACHE_HOME=str(config.parent/"private-cache"),XDG_DATA_HOME=str(config.parent/"private-data"),TZ="UTC",FLIX_BENCH_ENDPOINT='127.0.0.1:14554',FLIX_BENCH_TICKS='60000')
        with (args.output/'owner.log').open('w') as log:
            bench=subprocess.Popen([str(ROOT/'target/release/bench'),'0']+([] if args.trusted_local else [str(enrollment)]),stdout=subprocess.PIPE,stderr=log,text=True,env=environment)
            endpoint=bench.stdout.readline().strip()
            address,port=endpoint.rsplit(':',1)
            worker=threading.Thread(target=forward,args=((address,int(port)),),daemon=True);worker.start()
            # QGC startup includes arbitrary plugin logging. Keep it local/private.
            with (args.output/'qgc-private.log').open('w') as qlog:
                os.chmod(args.output/'qgc-private.log',0o600)
                process=subprocess.Popen([str(args.executable),'--logging:qgc.comms.mavlinkprotocol,qgc.comms.linkinterface,qgc.factsystem.parametermanager,qgc.vehicle.initialconnectstatemachine,qgc.vehicle.multivehiclemanager'],stdout=qlog,stderr=qlog,env=environment)
                bench.wait(timeout=150)
            result={'client':'QGroundControl v5.0.8','bench_endpoint':endpoint,
                    'executable_sha256':hashlib.sha256(args.executable.read_bytes()).hexdigest(),
                    'owner_returncode':bench.returncode,'qgc_early_returncode':process.poll(),
                    'rust_parameter_indices_sent':sorted(parameter_indices),
                    'observed_qgc_frames':[{'header':list(k),'count':v} for k,v in observed.items()],
                    'owner_summary':(args.output/'owner.log').read_text(),
                    'ingress_policy':'host-only BenchUsb' if args.trusted_local else 'signed per-boot session',
                    'physical_hardware':False,'manual_ui_commands_verified':False}
            qgc_log=(args.output/'qgc-private.log').read_text(errors='replace')
            failures=[]
            if 'Adding new vehicle link' not in qgc_log:failures.append('vehicle discovery unverified')
            if 'Initial load complete' not in qgc_log or 'Gave up on initial load' in qgc_log:
                failures.append('QGC internal parameter readiness unverified')
            if parameter_indices!=set(range(PARAMETER_COUNT)):failures.append('complete parameter transfer unverified')
            if not args.trusted_local and not any(len(k)>4 and k[2]==21 and k[3] and k[4] for k in observed):
                failures.append('signed QGC parameter request unverified')
            if bench.returncode!=0 or process.poll() is not None:failures.append('client or owner exited early')
            result.update(status='fail' if failures else 'pass',failures=failures)
            (args.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
            print(json.dumps(result,indent=2))
            return bool(failures)
    finally:
        stopping.set()
        if "worker" in locals():worker.join(timeout=1)
        relay.close()
        for child in (process,bench):
            if child is not None:
                if child.poll() is None:
                    child.terminate()
                    try:child.wait(timeout=5)
                    except subprocess.TimeoutExpired:child.kill();child.wait(timeout=5)
                if child.stdout is not None:child.stdout.close()
        for path,original in saved.items():
            if original is None:path.unlink(missing_ok=True)
            else:
                path.write_bytes(original[0]);path.chmod(original[1])
        enrollment.unlink(missing_ok=True)
if __name__=='__main__':raise SystemExit(main())
