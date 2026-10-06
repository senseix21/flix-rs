#!/usr/bin/env python3
"""Trace two actual Gazebo models: distinct ports, commands and stop authority."""
import argparse
import json
import math
from pathlib import Path
import selectors
import socket
import struct
import time
from gazebo_trace import decode, tilt
from pyflix.client import pack

ROOT=Path(__file__).resolve().parents[1]
def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    fixture=json.loads((ROOT/'fixtures/simulator/acceptance.json').read_text());contract=fixture['isolation']
    selector=selectors.DefaultSelector();clients=[];traces=[];sequences=[0,0]
    for i in range(2):
        trace=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);trace.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,4*1024*1024)
        trace.bind(('127.0.0.1',14560+2*i));selector.register(trace,selectors.EVENT_READ,i);traces.append(trace)
        clients.append(socket.socket(socket.AF_INET,socket.SOCK_DGRAM))
    def send(i,message,payload):
        clients[i].sendto(pack(message,payload,sequences[i]),('127.0.0.1',14550+2*i));sequences[i]=(sequences[i]+1)&255
    def command(i,number,*parameters):send(i,76,struct.pack('<7fHBBB',*(list(parameters)+[0.]*(7-len(parameters))),number,1,1,0))
    def manual(i,thrust):send(i,69,struct.pack('<hhhhHB',0,0,int(thrust*1000),0,0,1))
    latest=[None,None];first=[None,None];last_sequence=[None,None];dropped=[0,0];ticks=[0,0]
    stop_receipt=[None,None];stop_latency=[None,None];last_control=[-1000000,-1000000]
    maximum_tilt=[0.,0.];maximum_lift=[0.,0.];failures=[];stage='warmup';phase_at=None
    b_generation=None;stopped_at=None;deadline=time.monotonic()+fixture['max_wall_seconds']
    try:
        with (args.output/'trace.jsonl').open('w',buffering=4*1024*1024) as output:
            while time.monotonic()<deadline and stage!='finished':
                for ready,_ in selector.select(.2):
                    i=ready.data
                    try:record=decode(ready.fileobj.recv(512))
                    except ValueError:failures.append('invalid trace');continue
                    output.write(json.dumps(dict(model=i,**record),allow_nan=False,separators=(',',':'))+'\n')
                    now=record['sim_us']
                    if record['kind']=='command':
                        frame=bytes.fromhex(record['frame_hex'])
                        if record['accepted'] and int.from_bytes(frame[7:10],'little')==76 and int.from_bytes(frame[38:40],'little')==400 and struct.unpack_from('<f',frame,10)[0]==0:stop_receipt[i]=now
                        continue
                    ticks[i]+=1;latest[i]=record
                    if first[i] is None:first[i]=now
                    if last_sequence[i] is not None:dropped[i]+=max(0,record['sequence']-last_sequence[i]-1)
                    last_sequence[i]=record['sequence']
                    if not record['numeric_valid'] or not all(v is not None and 0<=v<=1 for v in record['motors']):failures.append('invalid applied output')
                    if record['state'] not in (2,3) and record['motors']!=[0.]*4:failures.append('stopped model has output')
                    if record['state']==3:
                        maximum_tilt[i]=max(maximum_tilt[i],math.degrees(tilt(record['pose_wxyz'])))
                        maximum_lift[i]=max(maximum_lift[i],record['position_world_m'][2]-.0138)
                    if None in latest:continue
                    a,b=latest
                    if stage=='warmup' and all(latest[j]['sim_us']-first[j]>=fixture['warmup_us'] for j in range(2)):
                        for j in range(2):command(j,400,0)
                        stage='stop-both'
                    elif stage=='stop-both' and all(stop_receipt[j] is not None and latest[j]['sim_us']>=stop_receipt[j] and latest[j]['state']==1 for j in range(2)):
                        command(0,176,1,2);b_generation=b['generation'];stage='select-a'
                    elif stage=='select-a' and a['source']==0 and a['state']==1:
                        manual(0,0);command(0,400,1);stage='arm-a'
                    elif stage=='arm-a' and a['state']==3:phase_at=now;stage='a-only'
                    elif stage=='a-only':
                        if b['state']!=1 or b['source'] is not None or b['motors']!=[0.]*4 or b['generation']!=b_generation:failures.append('A commands changed B authority')
                        if now-phase_at>=contract['a_only_us']:command(1,176,1,2);stage='select-b'
                    elif stage=='select-b' and b['source']==0 and b['state']==1:
                        manual(1,0);command(1,400,1);stage='arm-b'
                    elif stage=='arm-b' and b['state']==3:phase_at=now;stage='both'
                    elif stage=='both' and now-phase_at>=contract['both_us']:
                        stop_receipt[0]=None;command(0,400,0);b_generation=b['generation'];stage='stop-a'
                    elif stage=='stop-a' and stop_receipt[0] is not None and a['sim_us']>=stop_receipt[0] and a['motors']==[0.]*4:
                        stop_latency[0]=a['sim_us']-stop_receipt[0];phase_at=now;stage='b-only'
                    elif stage=='b-only':
                        if a['state']==3 or a['motors']!=[0.]*4:failures.append('A resumed after stop')
                        if b['state']!=3 or b['generation']!=b_generation:failures.append('A stop changed B authority')
                        if now-phase_at>=contract['b_only_us']:
                            stop_receipt[1]=None;command(1,400,0);stage='stop-b'
                    elif stage=='stop-b' and stop_receipt[1] is not None and b['sim_us']>=stop_receipt[1] and b['motors']==[0.]*4:
                        stop_latency[1]=b['sim_us']-stop_receipt[1];stopped_at=now;stage='stopped'
                    elif stage=='stopped':
                        if any(r['state']==3 or r['motors']!=[0.]*4 for r in latest):failures.append('output resumed after final stop')
                        if now-stopped_at>=contract['stopped_observation_us']:stage='finished'
                    active_a=stage in ('a-only','select-b','arm-b','both')
                    active_b=stage in ('both','stop-a','b-only')
                    for j,active in enumerate((active_a,active_b)):
                        if active and latest[j]['sim_us']-last_control[j]>=fixture['control_interval_us']:
                            manual(j,contract['thrust']);last_control[j]=latest[j]['sim_us']
    finally:
        for i in range(2):
            command(i,400,0);clients[i].close();traces[i].close()
        selector.close()
    if stage!='finished':failures.append('deadline expired at '+stage)
    if any(dropped):failures.append('trace incomplete')
    if any(v>contract['maximum_tilt_degrees'] for v in maximum_tilt):failures.append('tilt envelope exceeded')
    if any(v<contract['minimum_lift_m'] for v in maximum_lift):failures.append('minimum lift not observed')
    if any(v is None or v>contract['stop_us'] for v in stop_latency):failures.append('stop bound unverified or exceeded')
    result=dict(version=1,status='fail' if failures else 'pass',failures=sorted(set(failures)),requirements=contract['requirements'],
                ticks=ticks,dropped_ticks=dropped,stop_latency_sim_us=stop_latency,max_tilt_degrees=maximum_tilt,max_lift_m=maximum_lift,
                final=latest,hardware_test=False,source='uncommitted; attach source manifest')
    (args.output/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    return bool(failures)
if __name__=='__main__':raise SystemExit(main())
