#!/usr/bin/env python3
"""Actual Python client attaches after an isolated Gazebo owner's telemetry warmup."""
import argparse
import json
from pathlib import Path
import socket
import struct
import threading
import time
from gazebo_trace import decode
from pyflix.client import Flix, UdpTransport, pack

ROOT=Path(__file__).resolve().parents[1]
def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    fixture=json.loads((ROOT/'fixtures/simulator/acceptance.json').read_text());contract=fixture['late_client']
    trace=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);trace.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,4*1024*1024)
    trace.bind(('127.0.0.1',14560));trace.settimeout(.2)
    done=threading.Event();worker=None;errors=[];metrics=None
    def exercise():
        nonlocal metrics
        try:
            with Flix(UdpTransport(port=14550),simulation=True) as client:
                try:
                    client.disarm();client.set_mode(2);client.manual(thrust=0);client.arm()
                    until=time.monotonic()+contract['control_wall_seconds']
                    while time.monotonic()<until:
                        client.manual(thrust=contract['thrust']);time.sleep(.02)
                    client.disarm();client.console('maintenance')
                    if client.status()!='maintenance':raise RuntimeError('maintenance unavailable')
                    metrics=client.metrics();client.console('exit')
                finally:client.disarm()
        except Exception as error:errors.append(type(error).__name__+': '+str(error))
        finally:done.set()
    first=None;last=None;sequence=None;dropped=0;ticks=0;seen_output=False;stopped_at=None;failures=[]
    deadline=time.monotonic()+fixture['max_wall_seconds']
    try:
        with (args.output/'trace.jsonl').open('w') as output:
            while time.monotonic()<deadline:
                try:record=decode(trace.recv(512))
                except socket.timeout:continue
                except ValueError:failures.append('invalid trace');continue
                output.write(json.dumps(record,allow_nan=False,separators=(',',':'))+'\n')
                if record['kind']!='tick':continue
                last=record;ticks+=1;now=record['sim_us']
                if first is None:first=now
                if sequence is not None:dropped+=max(0,record['sequence']-sequence-1)
                sequence=record['sequence']
                if not record['numeric_valid']:failures.append('nonfinite observation')
                if record['state'] not in (2,3) and record['motors']!=[0.]*4:failures.append('stopped output nonzero')
                if record['state']==3 and any(v>0 for v in record['motors']):seen_output=True
                if worker is None and now-first>=fixture['warmup_us']:
                    worker=threading.Thread(target=exercise,daemon=True);worker.start()
                if done.is_set():
                    if record['state']==3 or record['motors']!=[0.]*4:failures.append('client left output active')
                    if stopped_at is None:stopped_at=now
                    if now-stopped_at>=contract['stopped_observation_us']:break
    finally:
        stop=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        stop.sendto(pack(76,struct.pack('<7fHBBB',0,0,0,0,0,0,0,400,1,1,0)),('127.0.0.1',14550));stop.close()
        trace.close()
        if worker is not None:worker.join(timeout=2)
    if not done.is_set():failures.append('client deadline expired')
    if dropped:failures.append('trace incomplete')
    if not seen_output:failures.append('armed control output not observed')
    if metrics is None or metrics['histograms']['sample_age']['count']==0:failures.append('metrics snapshot not received')
    failures.extend(errors)
    result=dict(version=1,status='fail' if failures else 'pass',failures=sorted(set(failures)),requirements=contract['requirements'],
                ticks=ticks,dropped_ticks=dropped,seen_armed_output=seen_output,metrics=metrics,final_tick=last,hardware_test=False,
                client='repository pyflix; unsigned loopback simulation',source='uncommitted; attach source manifest')
    (args.output/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    return bool(failures)
if __name__=='__main__':raise SystemExit(main())
