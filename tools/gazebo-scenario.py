#!/usr/bin/env python3
"""Run one finite control scenario against an isolated, trace-enabled Gazebo."""
import argparse
import json
import math
from pathlib import Path
import socket
import struct
import subprocess
import time
from gazebo_trace import decode, tilt
from pyflix.client import Decoder, pack

ROOT = Path(__file__).resolve().parents[1]

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--scenario',required=True)
    parser.add_argument('--native-reset',action='store_true')
    parser.add_argument('--reset-kind',choices=['all','time','model'],default='all')
    parser.add_argument('--queued-reset',action='store_true')
    parser.add_argument('--recover-reset',action='store_true')
    parser.add_argument('--engine',choices=['rust','legacy'],default='rust')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.queued_reset and (not args.native_reset or args.reset_kind!='model'):parser.error('queued-reset requires native model reset')
    contract=json.loads((ROOT/'fixtures/simulator/acceptance.json').read_text())
    scenario=next(s for s in contract['scenarios'] if s['name']==args.scenario)
    args.output.mkdir(parents=True,exist_ok=True)
    sequence=0
    def send(message,payload):
        nonlocal sequence
        frame=pack(message,payload,sequence)
        client.sendto(frame,('127.0.0.1',14550))
        if stage=='pausing':queued_frames.append(frame.hex())
        sequence=(sequence+1)&255
    def command(number,*parameters):
        send(76,struct.pack('<7fHBBB',*(list(parameters)+[0.]*(7-len(parameters))),number,1,1,0))
    def console(text):
        text=text.encode();send(126,struct.pack('<IHBBB70s',0,0,10,0,len(text),text))
    mode=scenario.get('mode',2)
    def manual(thrust,pitch=0,roll=0,yaw=0):
        if mode==3:
            send(82,struct.pack('<I8fBBB',0,1,0,0,0,0,0,0,thrust,1,1,0))
        else:
            send(69,struct.pack('<hhhhHB',int(pitch*1000),int(roll*1000),int(thrust*1000),int(yaw*1000),0,1))
    # Collectors own disk IO. The plugin only sends bounded nonblocking datagrams.
    trace=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    trace.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,4*1024*1024)
    trace.bind(('127.0.0.1',14560));trace.settimeout(.2)
    client=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);client.setblocking(False)
    replies=Decoder();saturation_notices=[]
    def drain_replies():
        # Owner replies go to the learned client endpoint; keep only STATUSTEXT warnings.
        while True:
            try: data=client.recv(512)
            except (BlockingIOError,OSError): return
            for message,payload in replies.feed(data):
                if message==253:
                    text=payload[1:51].split(b'\0',1)[0].decode('ascii','replace')
                    saturation_notices.append(dict(sim_us=last_tick['sim_us'] if last_tick else None,severity=payload[0],text=text))
    total=sum(s['duration_us'] for s in scenario['segments'])
    stage='warmup';start=None;armed_at=None;disarm_at=None;last_control=-1000000
    ticks=0;commands=0;dropped=0;last_sequence=None;invalid=0;max_tilt=0.;max_height=0.
    initial_height=None;failures=[];stop_ack_at=None;stopped_at=None;first_fault=None;post_stop_fault=None;first_inverted=None;last_tick=None;reset_seen=False;previous_now=None;reset_error=None
    last_setpoint_at=None; loss_zero_at=None; reconnect_started=False; reset_fault=None; recovery_started=None; recovered=False
    deadline=time.monotonic()+contract['max_wall_seconds']
    reset_process=None
    reset_client_log=None
    reset_generation=None
    previous_record=None
    queued_controls_before_reset=False
    queued_frames=[]
    mode_changed=False;mode_stopped_at=None;mode_rearmed=False;diagnostic_seen=False;saturated=False;recovered_output=False;gap_zero_ticks=0;limited_ticks=0;max_raw_acceleration=0.;contact_fault=None
    try:
        with (args.output/'trace.jsonl').open('w') as file:
            while time.monotonic()<deadline:
                if stage=='pausing' and reset_process.poll() is not None:
                    if reset_process.returncode!=0:
                        failures.append('pause not confirmed');break
                    command(176,1,mode);manual(0);command(400,1);manual(.55)
                    queued_controls_before_reset=True
                    reset_process=subprocess.Popen([str(ROOT/'target/gazebo-plugin/flix-world-reset'),'model-resume'],stdout=reset_client_log,stderr=subprocess.STDOUT)
                    stage='resetting'
                drain_replies()
                try: data,_=trace.recvfrom(512)
                except socket.timeout: continue
                try: record=decode(data)
                except ValueError: invalid+=1;continue
                file.write(json.dumps(record,allow_nan=False,separators=(',',':'))+'\n')
                now=record['sim_us']
                if record['kind']=='command':
                    commands+=1
                    frame=bytes.fromhex(record['frame_hex'])
                    if record['accepted'] and int.from_bytes(frame[7:10],'little') in (69,82): last_setpoint_at=now
                    if record['accepted'] and len(frame)>=40 and int.from_bytes(frame[7:10],'little')==76 and int.from_bytes(frame[38:40],'little')==400 and struct.unpack_from('<f',frame,10)[0]==0:
                        stop_ack_at=now
                    continue
                ticks+=1
                last_tick=record
                if record.get('sensor_limit_mask',0):limited_ticks+=1
                raw=record.get('raw_acceleration_body_m_s2',[])
                if raw and all(v is not None for v in raw):max_raw_acceleration=max(max_raw_acceleration,math.sqrt(sum(v*v for v in raw)))
                if 'landing_contact' in scenario and record['state']==5:contact_fault=record
                reset_observed=(previous_now is not None and now<previous_now) if args.reset_kind!='model' else (reset_generation is not None and record['generation']!=reset_generation)
                if stage=='resetting' and reset_observed:
                    reset_seen=True;stage='reset-settling';disarm_at=now
                    if record['state'] not in contract['native_reset']['allowed_stopped_states'] or record['source'] is not None or record['motors']!=[0.]*4:
                        failures.append('reset preserved authority')
                    if args.reset_kind!='time' and (math.hypot(*record['position_world_m'][:2])>contract['native_reset']['origin_tolerance_m'] or abs(record['position_world_m'][2]-.0138)>contract['native_reset']['origin_tolerance_m'] or math.degrees(tilt(record['pose_wxyz']))>contract['native_reset']['initial_tilt_degrees']):
                        failures.append('reset did not restore upright origin')
                    if args.reset_kind=='time' and previous_record is not None and math.dist(record['position_world_m'],previous_record['position_world_m'])>contract['native_reset']['time_only_position_jump_m']:
                        failures.append('time reset unexpectedly moved model')
                previous_now=now
                previous_record=record
                if all(v is not None for v in record['pose_wxyz']) and tilt(record['pose_wxyz'])>math.pi/2 and first_inverted is None:
                    first_inverted=record
                if stage=='settling' and record['state']==5 and post_stop_fault is None:
                    post_stop_fault=record
                if last_sequence is not None: dropped+=max(0,record['sequence']-last_sequence-1)
                last_sequence=record['sequence']
                if start is None: start=now;initial_height=record['position_world_m'][2]
                if 'timing_gap_at_us' in scenario and scenario['timing_gap_at_us']<=now<scenario['timing_gap_at_us']+scenario['timing_gap_us']:
                    gap_zero_ticks+=1
                    if record['motors']!=[0.]*4:failures.append('owner gap held nonzero force')
                expected_invalid=(scenario.get('sensor_fault')=='invalid' and now==scenario.get('sensor_fault_at_us') and record['acceleration_body_m_s2'][0] is None)
                if not record['numeric_valid'] and not expected_invalid: failures.append('nonfinite observation')
                if expected_invalid and any(v is None for key,values in record.items() if isinstance(values,list) and key!='acceleration_body_m_s2' for v in values):failures.append('nonfinite output during injected invalid sensor')
                if record['state'] not in (2,3) and record['motors']!=[0.]*4:
                    failures.append('stopped lifecycle has nonzero motors')
                if not all(v is not None and 0<=v<=1 for v in record['motors']):
                    failures.append('invalid applied motor output')
                if stage=='warmup' and now-start>=contract['warmup_us']:
                    command(400,0);stage='stopping'
                elif stage=='stopping' and stop_ack_at is not None and now>=stop_ack_at and record['state']==1:
                    if scenario.get('diagnostic'):
                        console('maintenance');stage='maintenance'
                    else:
                        command(176,1,mode);stage='select'
                elif stage=='maintenance' and record['state']==2:
                    command(209,1,0,10,.1,1);stage='diagnostic'
                elif stage=='diagnostic' and record['motors'][0]>0:
                    diagnostic_seen=True
                    if abs(record['motors'][0]-.1)>1e-6 or record['motors'][1:]!=[0.]*3:failures.append('diagnostic motor mapping invalid')
                    command(400,0);disarm_at=now;stop_ack_at=None;stage='settling'
                elif stage=='mode-stopping':
                    if record['state']==1 and record['motors']==[0.]*4:
                        mode_stopped_at=now;mode=scenario['change_mode'];command(176,1,mode);manual(0);stage='mode-hold'
                elif stage=='mode-hold':
                    if record['state']==3 or record['motors']!=[0.]*4:failures.append('mode selection implicitly armed')
                    if now-mode_stopped_at>=scenario['mode_hold_us']:
                        manual(0);command(400,1);stage='mode-rearming'
                elif stage=='mode-rearming' and record['state']==3:
                    mode_rearmed=True;armed_at=now;stage='flight'
                elif stage=='select' and record['source']==(1 if mode==3 else 0) and record['state']==1:
                    manual(0);command(400,1);stage='arming'
                elif stage=='arming' and record['state']==3:
                    armed_at=now;stage='flight'
                elif stage=='flight':
                    max_tilt=max(max_tilt,math.degrees(tilt(record['pose_wxyz'])))
                    max_height=max(max_height,record['position_world_m'][2]-initial_height)
                    losing='loss_after_us' in scenario and now-armed_at>=scenario['loss_after_us']
                    if losing and record['state']!=3 and record['motors']==[0.]*4 and loss_zero_at is None: loss_zero_at=now
                    if losing and now-armed_at>=scenario['loss_after_us']+scenario['lease_us']+5000:
                        reconnect_started=True
                        if record['state']==3 or record['motors']!=[0.]*4: failures.append('reconnect implicitly armed')
                    if record['state']==5 and first_fault is None and not losing:
                        first_fault=record
                    elapsed=now-armed_at
                    if scenario.get('saturation'):
                        if elapsed<520000 and max(record['motors'])>=.999:saturated=True
                        if elapsed>600000 and record['state']==3 and all(0<v<.95 for v in record['motors']):recovered_output=True
                    if 'change_mode' in scenario and not mode_changed and elapsed>=scenario['change_after_us']:
                        mode_changed=True;command(400,0);stage='mode-stopping';continue
                    if elapsed>=total:
                        if args.native_reset:
                            stage='pausing' if args.queued_reset else 'resetting'
                            reset_generation=record['generation']
                            try:
                                reset_client_log=(args.output/'reset-client.log').open('w')
                                reset_process=subprocess.Popen([str(ROOT/'target/gazebo-plugin/flix-world-reset'),'pause' if args.queued_reset else args.reset_kind],stdout=reset_client_log,stderr=subprocess.STDOUT)
                            except (OSError,subprocess.SubprocessError) as error:
                                reset_error=str(error);failures.append('native reset client failed')
                                command(400,0);disarm_at=now;stage='settling'
                        else:
                            command(400,0);disarm_at=now;stop_ack_at=None;stage='settling'
                    elif now-last_control>=contract['control_interval_us']:
                        boundary=0
                        for segment in scenario['segments']:
                            boundary+=segment['duration_us']
                            if elapsed<boundary:
                                if not losing: manual(segment['thrust'],segment['pitch'],segment.get('roll',0),segment.get('yaw',0))
                                elif reconnect_started: manual(0)
                                break
                        last_control=now
                elif stage=='reset-settling':
                    if record['state']==5 and reset_fault is None: reset_fault=record
                    if record['state']==3 or record['motors']!=[0.]*4:
                        failures.append('reset implicitly rearmed')
                    if now-disarm_at>=contract['settle_us']:
                        if args.recover_reset:
                            text=b'clear-fault';send(126,struct.pack('<IHBBB70s',0,0,10,0,len(text),text))
                            recovery_started=now;stage='recovering'
                        else:
                            stage='finished';break
                elif stage=='recovering':
                    if record['state']==3 or record['motors']!=[0.]*4: failures.append('recovery implicitly armed')
                    if now-recovery_started>=500000:
                        recovered=record['state']==1 and record['motors']==[0.]*4
                        if not recovered: failures.append('explicit reset recovery failed')
                        stage='finished';break
                elif stage=='settling':
                    if stop_ack_at is not None and now>=stop_ack_at and stopped_at is None and record['motors']==[0.]*4:
                        stopped_at=now
                    if stopped_at is not None and record['motors']!=[0.]*4:
                        failures.append('motor output resumed after stop')
                    if now-disarm_at>=contract['settle_us']:
                        stage='finished';break
            if stage!='finished': failures.append('scenario deadline expired at '+stage)
    finally:
        if reset_process is not None:
            try:
                if reset_process.wait(timeout=2)!=0:failures.append('native reset client failed')
            except subprocess.TimeoutExpired:
                reset_process.kill();reset_process.wait();failures.append('native reset client deadline expired')
        if reset_client_log is not None:reset_client_log.close()
        for _ in range(3):command(400,0)
        time.sleep(.05);drain_replies()
        client.close();trace.close()
    if dropped or invalid:failures.append('trace incomplete')
    if first_fault and not scenario['expect_fault_while_commanding']:failures.append('fault during commanded flight')
    if 'sensor_fault' in scenario:
        if first_fault is None or first_fault['reason']!=scenario['expected_fault_reason'] or first_fault['sim_us']-scenario['sensor_fault_at_us'] not in (0,1000) or first_fault['motors']!=[0.]*4:
            failures.append('injected sensor fault did not stop at declared boundary')
        if scenario['sensor_fault']=='stale' and first_fault is not None and first_fault['sim_us']-first_fault['sample_us']!=10001:failures.append('stale sample injection unverified')
    if scenario.get('diagnostic') and not diagnostic_seen:failures.append('active diagnostic not observed')
    if 'change_mode' in scenario and not mode_rearmed:failures.append('explicit mode rearm not observed')
    if scenario.get('saturation') and not (saturated and recovered_output):failures.append('saturation and recovery unverified')
    if 'timing_gap_at_us' in scenario:
        if gap_zero_ticks!=scenario['timing_gap_us']//contract['step_us']:failures.append('owner gap trace incomplete')
        expected=scenario['timing_gap_at_us']+scenario['timing_gap_us']
        if first_fault is None or first_fault['reason']!=6 or abs(first_fault['sim_us']-expected)>1000 or first_fault['motors']!=[0.]*4:failures.append('owner gap did not stop at declared boundary')
    if scenario.get('landing_contact') and (contact_fault is not None or limited_ticks==0):failures.append('landing contact fault or range observation missing')
    if args.engine=='rust' and limited_ticks and not any(n['text'].startswith('IMU saturated:') for n in saturation_notices):
        failures.append('range-limited samples not reported to operator')
    if max_tilt>scenario['max_armed_tilt_degrees']:failures.append('tilt envelope exceeded')
    if max_height<scenario['min_lift_m']:failures.append('minimum lift not observed')
    if args.native_reset and not reset_seen:failures.append('native reset not observed')
    if not args.native_reset and (stop_ack_at is None or stopped_at is None or stopped_at-stop_ack_at>scenario['disarm_stop_us']):
        failures.append('stop boundary unverified or exceeded')
    if 'loss_after_us' in scenario and (loss_zero_at is None or loss_zero_at-armed_at>scenario['loss_after_us']+scenario['lease_us']+1000): failures.append('lease stop boundary unverified or exceeded')
    result=dict(version=1,limited_ticks=limited_ticks,saturation_notices=saturation_notices,max_raw_acceleration_m_s2=max_raw_acceleration,contact_fault=contact_fault,gap_zero_ticks=gap_zero_ticks,mode_rearmed=mode_rearmed,diagnostic_seen=diagnostic_seen,saturated=saturated,recovered_output=recovered_output,queued_frames=queued_frames,queued_controls_before_reset=queued_controls_before_reset,reset_kind=args.reset_kind,explicit_reset_recovery=recovered,reset_fault=reset_fault,loss_zero_at_us=loss_zero_at,reconnect_without_arm_checked=reconnect_started,native_reset=args.native_reset,reset_seen=reset_seen,
                native_recovery_status=('latched-fault-needs-explicit-clear' if last_tick and last_tick['state']==5 else 'stopped') if args.native_reset else None,reset_error=reset_error,engine=args.engine,scenario=scenario['name'],profile=contract['profile'],requirements=scenario['requirements'],
                status='fail' if failures else 'pass',failures=sorted(set(failures)),ticks=ticks,
                command_events=commands,dropped_ticks=dropped,invalid_packets=invalid,
                max_tilt_degrees=max_tilt,max_lift_m=max_height,first_fault=first_fault,
                first_inverted=first_inverted,post_stop_fault=post_stop_fault,final_tick=last_tick,
                stop_latency_sim_us=None if stopped_at is None or stop_ack_at is None else stopped_at-stop_ack_at,
                hardware_test=False,source='uncommitted; attach source manifest')
    (args.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return 1 if failures else 0

if __name__=='__main__':raise SystemExit(main())
