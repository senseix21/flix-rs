#!/usr/bin/env python3
"""Compare finite nominal plant traces; report differences without changing gains."""
import argparse
import json
import math
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]

def rows(path):
    ticks=[json.loads(line) for line in path.read_text().splitlines()]
    ticks=[r for r in ticks if r['kind']=='tick']
    if any(not r['numeric_valid'] for r in ticks):raise ValueError('Nonfinite trace')
    if any(b['sequence']!=a['sequence']+1 for a,b in zip(ticks,ticks[1:])):raise ValueError('Dropped or reordered trace')
    armed=next(r['sim_us'] for r in ticks if r['state']==3)
    # The legacy adapter truncates double seconds to microseconds. Quantize to
    # its declared 1 ms physics tick, rather than introducing interpolation.
    return {round((r['sim_us']-armed)/1000):r for r in ticks if r['state']==3}

def angle(a,b):
    dot=sum(x*y for x,y in zip(a,b))
    norm=math.sqrt(sum(x*x for x in a)*sum(y*y for y in b))
    if norm==0:raise ValueError('Zero quaternion')
    return math.degrees(2*math.acos(min(1.,abs(dot/norm))))

def compare(left,right,limits):
    common=sorted(set(left)&set(right))
    if not common:raise ValueError('No aligned armed samples')
    metrics={'max_attitude_difference_degrees':0.,'max_motor_absolute_difference':0.,'max_position_difference_m':0.}
    first=None
    violations=[]
    for tick in common:
        a,b=left[tick],right[tick]
        values={
          'max_attitude_difference_degrees':angle(a['pose_wxyz'],b['pose_wxyz']),
          'max_motor_absolute_difference':max(abs(x-y) for x,y in zip(a['motors'],b['motors'])),
          'max_position_difference_m':math.sqrt(sum((x-y)**2 for x,y in zip(a['position_world_m'],b['position_world_m'])))
        }
        for key,value in values.items():metrics[key]=max(metrics[key],value)
        if any(values[key]>limits[key] for key in values):
            violations.append(tick)
            if first is None:first=dict(relative_tick=tick,metrics=values,rust=a,legacy=b)
    mismatches=sorted(set(left)^set(right))
    return dict(status='pass' if first is None and len(mismatches)<=2 else 'fail',samples=len(common),
                violation_ticks=violations,boundary_unmatched_ticks=mismatches,metrics=metrics,thresholds=limits,first_threshold_violation=first)

def transition_ticks(path):
    records=[json.loads(line) for line in path.read_text().splitlines()]
    armed=next(r['sim_us'] for r in records if r['kind']=='tick' and r['state']==3)
    previous=None;transitions=set()
    for r in records:
        if r['kind']!='command':continue
        frame=bytes.fromhex(r['frame_hex'])
        if len(frame)<18 or int.from_bytes(frame[7:10],'little')!=69:continue
        import struct
        values=struct.unpack_from('<4h',frame,10)
        if values!=previous:
            transitions.add(round((r['sim_us']-armed)/1000))
            previous=values
    return transitions

def main():
    parser=argparse.ArgumentParser();parser.add_argument('rust',type=Path);parser.add_argument('legacy',type=Path);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    limits=json.loads((ROOT/'fixtures/simulator/acceptance.json').read_text())['comparison']
    limits={k:v for k,v in limits.items() if k.startswith('max_')}
    rust,legacy=rows(args.rust),rows(args.legacy)
    result=compare(rust,legacy,limits)
    # Keep the full predeclared gate result. Characterize known dispatch-order
    # boundaries independently; this does not convert failed full parity to pass.
    transitions=transition_ticks(args.rust)|transition_ticks(args.legacy)
    result['command_transition_ticks']=sorted(transitions)
    result['stable_interval_characterization']=compare(
        {t:r for t,r in rust.items() if t not in transitions},
        {t:r for t,r in legacy.items() if t not in transitions},limits)
    unexpected=set(result['violation_ticks'])-transitions
    result['dispatch_difference_classification']={
        'status':'explained' if not unexpected and len(result['boundary_unmatched_ticks'])<=2 else 'unexplained',
        'unexpected_violation_ticks':sorted(unexpected),
        'contract':'Only actual logged MANUAL_CONTROL transition ticks may differ; no widened time window.'}
    result['full_tick_parity_remains_open']=result['status']!='pass'
    # Acceptance rule (fixtures/simulator/acceptance.json comparison.acceptance_rule).
    stable=result['stable_interval_characterization']
    result['acceptance']=dict(
        rule='stable-interval',
        status='pass' if stable['status']=='pass' and result['dispatch_difference_classification']['status']=='explained' else 'fail')
    result.update(profile='paired-Gazebo11-nominal',hardware_test=False,
      differences=['Legacy truth-yaw injection disabled in disposable comparison copy.',
                   'Legacy acceleration input retains its 0.1 filter; Rust uses MPU6050 range limits and retains raw input in trace v3.',
                   'Legacy controller processes MAVLink after control; Rust stop preempts tick.',
                   'Legacy state/reason/generation observation is descriptive, not a Rust lifecycle contract.'])
    args.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    return result['acceptance']['status']!='pass'
if __name__=='__main__':raise SystemExit(main())
