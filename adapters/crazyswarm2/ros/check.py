"""Evaluate one Crazyswarm2 S1 run from the FlixSIL traces and the coordinator's marks.

    python3 check.py --scenario formation --trace DIR

Writes DIR/results.json and exits nonzero when a check fails. Runs without ROS.
"""
import argparse
import itertools
import json
import math
import pathlib
import sys

GROUND = 0.0138
THRESHOLDS = {
    'hover_error_m': 0.05,
    'separation_observed_m': 0.25,
    'stop_latency_us': 2000,
}


def load(trace):
    drones = {}
    for path in sorted(trace.glob('cf*.jsonl')):
        drones[path.stem] = [json.loads(line) for line in path.read_text().splitlines()]
    marks = {}
    marks_path = trace / 'marks.jsonl'
    if marks_path.exists():
        for line in marks_path.read_text().splitlines():
            mark = json.loads(line)
            marks[mark['label']] = mark['sim_us']
    return drones, marks


def at(records, sim_us):
    """Last periodic record at or before sim_us."""
    return max((r for r in records if r['sim_us'] <= sim_us), key=lambda r: r['sim_us'])


def error(record):
    if record['target_m'] is None:
        return math.inf
    return math.dist(record['position_m'], record['target_m'])


def events(records, kind):
    return [e for r in records for e in r['events'] if e[1] == kind]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', required=True)
    parser.add_argument('--trace', type=pathlib.Path, required=True)
    args = parser.parse_args()
    drones, marks = load(args.trace)
    checks = []

    def check(name, passed, **detail):
        checks.append(dict(name=name, passed=bool(passed), **detail))

    check('three drones traced', sorted(drones) == ['cf1', 'cf2', 'cf3'], drones=sorted(drones))
    loops = {records[-1].get('position_loop', 'ground') for records in drones.values()}
    check('one position-loop mode', len(loops) == 1, position_loop=sorted(loops))
    ids = {name: records[0]['system_id'] for name, records in drones.items()}
    check('unique system IDs', len(set(ids.values())) == len(ids), system_ids=ids)
    flying = ['cf1', 'cf3'] if args.scenario == 'partial-arm' else sorted(drones)
    for name in flying:
        check(f'{name} armed', events(drones[name], 'armed'), events=events(drones[name], 'armed'))
    labels = {'formation': ('hover', 'triangle', 'line'), 'stop': ('hover',), 'partial-arm': ('hover',)}
    for label in labels[args.scenario]:
        if label not in marks:
            check(f'mark {label} recorded', False)
            continue
        for name in flying:
            e = error(at(drones[name], marks[label]))
            check(f'{name} {label} error', e < THRESHOLDS['hover_error_m'], error_m=round(e, 4))
    if args.scenario == 'formation':
        check('crossing plan rejected', 'crossing-rejected' in marks and 'crossing-accepted' not in marks)
        times = sorted(set.intersection(*(set(r['sim_us'] for r in records) for records in drones.values())))
        by_time = {name: {r['sim_us']: r for r in records} for name, records in drones.items()}
        closest = math.inf
        for t in times:
            airborne = [by_time[n][t]['position_m'] for n in drones if by_time[n][t]['position_m'][2] > GROUND + 0.05]
            for a, b in itertools.combinations(airborne, 2):
                closest = min(closest, math.dist(a, b))
        check('minimum pair distance', closest >= THRESHOLDS['separation_observed_m'], distance_m=round(closest, 4))
    if args.scenario in ('formation', 'partial-arm'):
        for name in flying:
            last = drones[name][-1]
            check(f'{name} landed and stopped',
                  last['position_m'][2] < GROUND + 0.02 and not any(last['motors']) and last['state'] != 3,
                  z_m=last['position_m'][2], state=last['state'])
    if args.scenario == 'stop':
        for name, records in drones.items():
            latency = records[-1]['stop_latency_us']
            check(f'{name} output stopped', latency is not None and latency <= THRESHOLDS['stop_latency_us'],
                  latency_us=latency, state=records[-1]['state'])
    if args.scenario == 'partial-arm':
        records = drones['cf2']
        check('cf2 never armed', not events(records, 'armed') and all(r['state'] != 3 for r in records))
        check('cf2 stayed on the ground', all(r['position_m'][2] < GROUND + 0.005 and not any(r['motors'])
                                             for r in records))

    passed = all(c['passed'] for c in checks)
    result = {'scenario': args.scenario, 'position_loop': sorted(loops)[0], 'passed': passed,
              'thresholds': THRESHOLDS, 'marks': marks, 'checks': checks}
    (args.trace / 'results.json').write_text(json.dumps(result, indent=1) + '\n')
    for c in checks:
        if not c['passed']:
            print('  FAIL', c)
    print(f"{'pass' if passed else 'FAIL'} crazyswarm2 {args.scenario}")
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
