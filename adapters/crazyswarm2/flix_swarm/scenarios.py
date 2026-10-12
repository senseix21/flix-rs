"""S1 swarm scenarios (swarm plan S1-09). Each writes results.json and trace.jsonl.

    PYTHONPATH=adapters/crazyswarm2 python3 -m flix_swarm.scenarios --scenario all --output DIR
    ... --onboard     # S2: the position loop runs in flight-core, fed by ATT_POS_MOCAP

Exit status is nonzero when any check fails. Thresholds are declared in THRESHOLDS and copied
into every result so evidence records what was tested.
"""
import argparse
import json
import math
import pathlib
import sys

from . import geometry as g
from .owner import ARMED, FAILSAFE, FAULT, REASONS, Capacity, Owner
from .swarm import GROUND, PlanRejected, Swarm
from .vehicle import Config

THRESHOLDS = {
    'hover_error_m': 0.05,
    'max_tilt_deg': 30.0,
    'separation_plan_m': 0.3,
    'separation_observed_m': 0.25,
    'stop_latency_us': 2000,
    'lease_margin_us': 25_000,
    'bystander_error_m': 0.1,
    # S2: flight-core position freshness limit (PositionTuning::DEFAULT.sample_us).
    'position_us': 200_000,
    'noisy_hover_error_m': 0.08,
}
LINE = [(0.0, 0.0, 0.0), (0.6, 0.0, 0.0), (1.2, 0.0, 0.0)]


class Checks:
    def __init__(self):
        self.items = []

    def __call__(self, name, passed, **detail):
        self.items.append(dict(name=name, passed=bool(passed), **detail))

    @property
    def passed(self):
        return all(item['passed'] for item in self.items)


def error(drone, now_us):
    return g.distance(drone.plant.position, drone.planner.current(now_us)[0])


def watch(swarm, seconds, drones, worst):
    """Run and record the worst tracking error and tilt of the given drones."""
    end = swarm.now_us + int(seconds * 1e6)
    while swarm.now_us < end:
        swarm.step()
        for d in drones:
            worst['error'] = max(worst.get('error', 0.0), error(d, swarm.now_us))
            worst['tilt'] = max(worst.get('tilt', 0.0), math.degrees(g.tilt(d.plant.attitude)))


def hover(swarm, check, height=0.5, error_limit=THRESHOLDS['hover_error_m']):
    swarm.provision()
    armed = swarm.arm()
    check('all drones armed', all(v is None for v in armed.values()), result=armed)
    swarm.takeoff(height, 2.0)
    swarm.run(4.0)
    for d in swarm.drones:
        check(f'{d.name} hover error', error(d, swarm.now_us) < error_limit,
              error_m=round(error(d, swarm.now_us), 4))


def landed(swarm, check):
    for d in swarm.drones:
        check(f'{d.name} landed and stopped',
              d.plant.position[2] < GROUND + 0.02 and not any(d.vehicle.motors) and d.vehicle.state != ARMED,
              z_m=round(d.plant.position[2], 4), state=d.vehicle.state)


def single(swarm, check):
    hover(swarm, check)
    swarm.go_to({'cf1': (0.5, 0.3, 0.6)}, 2.0)
    worst = {}
    watch(swarm, 3.0, swarm.drones, worst)
    d = swarm.drones[0]
    check('go_to reached', error(d, swarm.now_us) < THRESHOLDS['hover_error_m'], error_m=round(error(d, swarm.now_us), 4))
    check('tilt bounded', worst['tilt'] < THRESHOLDS['max_tilt_deg'], max_tilt_deg=round(worst['tilt'], 2))
    swarm.land(2.0)
    landed(swarm, check)


def formation(swarm, check):
    hover(swarm, check)
    goals = {d.name: d.planner.position for d in swarm.drones}
    try:
        swarm.go_to({'cf1': (1.2, 0.0, 0.5), 'cf3': (0.0, 0.0, 0.5)}, 2.0)
        check('crossing plan rejected', False)
    except PlanRejected as rejected:
        check('crossing plan rejected', True, reason=str(rejected))
    check('rejected plan moved nobody', all(d.planner.position == goals[d.name] for d in swarm.drones))
    triangle = {'cf1': (0.0, -0.3, 0.6), 'cf2': (0.6, 0.45, 0.6), 'cf3': (1.2, -0.3, 0.6)}
    swarm.go_to(triangle, 2.5)
    worst = {}
    watch(swarm, 3.5, swarm.drones, worst)
    swarm.go_to({'cf1': (0.0, 0.0, 0.8), 'cf2': (0.6, 0.0, 0.8), 'cf3': (1.2, 0.0, 0.8)}, 2.5)
    watch(swarm, 3.5, swarm.drones, worst)
    for d in swarm.drones:
        check(f'{d.name} formation error', error(d, swarm.now_us) < THRESHOLDS['hover_error_m'],
              error_m=round(error(d, swarm.now_us), 4))
    check('tilt bounded', worst['tilt'] < THRESHOLDS['max_tilt_deg'], max_tilt_deg=round(worst['tilt'], 2))
    swarm.land(2.5)
    landed(swarm, check)
    observed = swarm.min_separation[0]
    check('minimum pair distance', observed >= THRESHOLDS['separation_observed_m'], distance_m=round(observed, 4))


def swarm_stop(swarm, check):
    hover(swarm, check)
    swarm.stop_all()
    swarm.run(0.01)
    for d in swarm.drones:
        latency = None if d.vehicle.stopped_us is None else d.vehicle.stopped_us - d.vehicle.stop_requested_us
        check(f'{d.name} output stopped', latency is not None and latency <= THRESHOLDS['stop_latency_us'],
              latency_us=latency, state=d.vehicle.state)
    swarm.run(1.0)
    check('no drone re-armed', all(d.vehicle.state != ARMED and not any(d.vehicle.motors) for d in swarm.drones))


def bystanders(swarm, check, victim, seconds):
    others = [d for d in swarm.drones if d.name != victim]
    worst = {}
    watch(swarm, seconds, others, worst)
    check('bystanders kept station', worst['error'] < THRESHOLDS['bystander_error_m'],
          max_error_m=round(worst['error'], 4))
    check('bystanders still armed', all(d.vehicle.state == ARMED and d.vehicle.phase == 'armed' for d in others))


def stopped_at(drone, states):
    return next((e[0] for e in drone.vehicle.events if e[1] == 'state' and e[2] in states), None)


def link_loss(swarm, check):
    hover(swarm, check)
    victim = swarm.by_name('cf2')
    victim.vehicle.link.cut()
    cut_us = swarm.now_us
    bystanders(swarm, check, 'cf2', 2.0)
    # S1 stops on the command lease (Failsafe). S2 also loses position samples, whose 200 ms
    # limit expires first (Fault, position-expired). Either way: the first limit to expire.
    failed = stopped_at(victim, ('failsafe', 'fault'))
    lease = int(swarm.config.lease_s * 1e6)
    limit = (min(lease, THRESHOLDS['position_us']) if swarm.config.onboard else lease) + THRESHOLDS['lease_margin_us']
    check('cf2 stopped within its limit', failed is not None and failed - cut_us <= limit,
          after_cut_us=None if failed is None else failed - cut_us, limit_us=limit,
          reason=REASONS[victim.vehicle.reason])
    check('cf2 output zero', not any(victim.vehicle.motors) and victim.vehicle.state in (FAILSAFE, FAULT))
    check('cf2 reported lost', victim.vehicle.phase == 'lost',
          events=[e for e in victim.vehicle.events if e[1] == 'lost'])
    # Late traffic after the link returns must not revive authority.
    victim.vehicle.link.restore()
    for _ in range(25):
        victim.vehicle.link.send(victim.vehicle.messages.attitude(victim.vehicle.system_id, (1, 0, 0, 0), 0.6), swarm.now_us)
        swarm.run(0.02)
    check('late setpoints do not revive cf2',
          victim.vehicle.state in (FAILSAFE, FAULT) and not any(victim.vehicle.motors))


def sensor_fault(swarm, check):
    hover(swarm, check)
    victim = swarm.by_name('cf3')
    victim.vehicle.sensor_fault = ('nan', swarm.now_us + 1000)
    fault_us = swarm.now_us + 1000
    bystanders(swarm, check, 'cf3', 2.0)
    faulted = stopped_at(victim, ('fault',))
    check('cf3 faulted next tick', faulted is not None and faulted - fault_us <= 1000,
          after_us=None if faulted is None else faulted - fault_us, reason=victim.vehicle.reason)
    check('cf3 output zero', not any(victim.vehicle.motors) and victim.vehicle.state == FAULT)
    check('cf3 reported lost', victim.vehicle.phase == 'lost')


def arm_refusal(swarm, check):
    swarm.provision()
    swarm.by_name('cf2').vehicle.calibrated = False
    swarm.run(0.05)
    armed = swarm.arm()
    check('cf2 refused with reason', armed['cf2'] not in (None, 'arming'), refusal=armed['cf2'])
    check('others armed', armed['cf1'] is None and armed['cf3'] is None, result=armed)
    swarm.takeoff(0.5, 2.0)
    worst = {}
    victim = swarm.by_name('cf2')
    end = swarm.now_us + 4_000_000
    while swarm.now_us < end:
        swarm.step()
        worst['motor'] = max(worst.get('motor', 0.0), max(victim.vehicle.motors))
        worst['z'] = max(worst.get('z', 0.0), victim.plant.position[2])
    check('cf2 stayed on the ground', worst['motor'] == 0.0 and worst['z'] < GROUND + 0.005,
          max_motor=worst['motor'], max_z_m=round(worst['z'], 4))
    for name in ('cf1', 'cf3'):
        d = swarm.by_name(name)
        check(f'{name} hover error', error(d, swarm.now_us) < THRESHOLDS['hover_error_m'],
              error_m=round(error(d, swarm.now_us), 4))
    swarm.land(2.0)


def load(swarm, check):
    try:
        extra = Owner()
        extra.close()
        check('ninth owner refused', False)
    except Capacity:
        check('ninth owner refused', True)
    hover(swarm, check)
    count = len(swarm.drones)
    before = {d.name: d.vehicle.link.sent for d in swarm.drones}
    turn = {d.name: swarm.drones[(i + 1) % count].planner.position for i, d in enumerate(swarm.drones)}
    swarm.go_to(turn, 3.0)
    start = swarm.now_us
    swarm.run(4.0)
    seconds = (swarm.now_us - start) / 1e6
    rates = {d.name: round((d.vehicle.link.sent - before[d.name]) / seconds, 1) for d in swarm.drones}
    for d in swarm.drones:
        check(f'{d.name} rotated', error(d, swarm.now_us) < THRESHOLDS['hover_error_m'],
              error_m=round(error(d, swarm.now_us), 4))
    check('no frames dropped', all(d.vehicle.link.dropped == 0 for d in swarm.drones))
    check('setpoint rate recorded', True, frames_per_second=rates,
          total_frames_per_second=round(sum(rates.values()), 1),
          arm_latency_us={d.name: d.vehicle.arm_latency_us for d in swarm.drones})
    swarm.land(2.0)
    landed(swarm, check)


def position_dropout(swarm, check):
    """S2: the link stays up and targets keep coming, but cf2's position samples stop (SWM-07)."""
    hover(swarm, check)
    victim = swarm.by_name('cf2')
    victim.vehicle.positioning = False
    # The last sample may have been sent up to one period before the switch.
    cut_us = swarm.now_us - swarm.config.position_period_us
    bystanders(swarm, check, 'cf2', 2.0)
    faulted = stopped_at(victim, ('fault',))
    limit = THRESHOLDS['position_us'] + swarm.config.position_period_us + THRESHOLDS['lease_margin_us']
    check('cf2 stopped on stale position', faulted is not None and faulted - cut_us <= limit
          and REASONS[victim.vehicle.reason] == 'position-expired',
          after_us=None if faulted is None else faulted - cut_us, limit_us=limit,
          reason=REASONS[victim.vehicle.reason])
    check('cf2 output zero', not any(victim.vehicle.motors) and victim.vehicle.state == FAULT)
    check('cf2 reported lost', victim.vehicle.phase == 'lost')
    victim.vehicle.positioning = True
    swarm.run(0.5)
    check('returning samples do not revive cf2', victim.vehicle.state == FAULT and not any(victim.vehicle.motors))


def outside_volume(swarm, check):
    """S2: the drone itself rejects a target above the 2 m ceiling (SWM-08), even if the
    ground sends it; with no valid target the lease runs out and only that drone stops."""
    hover(swarm, check)
    victim = swarm.by_name('cf1')
    held = victim.plant.position[2]
    # Bypass the coordinator: the frames reach the drone, decode, and flight-core refuses them.
    victim.planner.go_to((0.0, 0.0, 2.5), 0.02, swarm.now_us)
    sent_us = swarm.now_us
    start = len(swarm.trace)
    bystanders(swarm, check, 'cf1', 1.0)
    peak = max(r['drones']['cf1']['position_m'][2] for r in swarm.trace[start:])
    check('cf1 did not follow the out-of-volume target', peak < held + 0.05,
          held_m=round(held, 4), peak_m=round(peak, 4))
    failed = stopped_at(victim, ('failsafe',))
    limit = int(swarm.config.lease_s * 1e6) + THRESHOLDS['lease_margin_us'] + 20_000
    check('cf1 stopped when its lease ran out', failed is not None and failed - sent_us <= limit,
          after_us=None if failed is None else failed - sent_us, limit_us=limit)


def noisy(swarm, check):
    """S2: formation with 2 cm position noise and 20 ms link latency."""
    hover(swarm, check, error_limit=THRESHOLDS['noisy_hover_error_m'])
    swarm.go_to({'cf1': (0.0, -0.3, 0.6), 'cf2': (0.6, 0.45, 0.6), 'cf3': (1.2, -0.3, 0.6)}, 2.5)
    worst = {}
    watch(swarm, 3.5, swarm.drones, worst)
    for d in swarm.drones:
        check(f'{d.name} formation error', error(d, swarm.now_us) < THRESHOLDS['noisy_hover_error_m'],
              error_m=round(error(d, swarm.now_us), 4))
    check('tilt bounded', worst['tilt'] < THRESHOLDS['max_tilt_deg'], max_tilt_deg=round(worst['tilt'], 2))
    swarm.land(2.5)
    landed(swarm, check)


def circle(count, radius=1.0):
    return [(radius * math.cos(2 * math.pi * i / count), radius * math.sin(2 * math.pi * i / count), 0.0)
            for i in range(count)]


SCENARIOS = {
    'single': ([(0.0, 0.0, 0.0)], single),
    'formation': (LINE, formation),
    'swarm-stop': (LINE, swarm_stop),
    'link-loss': (LINE, link_loss),
    'sensor-fault': (LINE, sensor_fault),
    'arm-refusal': (LINE, arm_refusal),
    'load': (circle(8), load),
}
# S2 only: they exercise the on-board position loop and its inputs.
ONBOARD = {
    'position-dropout': (LINE, position_dropout),
    'outside-volume': (LINE, outside_volume),
    'noisy': (LINE, noisy),
}
NOISY = dict(position_noise_m=0.02, link_latency_us=20_000)


def run(name, output, config=None, seed=1):
    positions, body = {**SCENARIOS, **ONBOARD}[name]
    check = Checks()
    config = config or Config()
    with Swarm(positions, seed=seed, config=config, separation=THRESHOLDS['separation_plan_m']) as swarm:
        try:
            body(swarm, check)
        except Exception as failure:  # record, then fail the scenario
            check('scenario completed', False, error=f'{type(failure).__name__}: {failure}')
        result = {
            'scenario': name,
            'passed': check.passed,
            'seed': seed,
            'config': vars(config),
            'thresholds': THRESHOLDS,
            'checks': check.items,
            'report': swarm.report(),
        }
        if output is not None:
            directory = pathlib.Path(output) / name
            directory.mkdir(parents=True, exist_ok=True)
            (directory / 'results.json').write_text(json.dumps(result, indent=1) + '\n')
            with (directory / 'trace.jsonl').open('w') as trace:
                for record in swarm.trace:
                    trace.write(json.dumps(record, separators=(',', ':')) + '\n')
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', default='all', choices=['all'] + sorted({**SCENARIOS, **ONBOARD}))
    parser.add_argument('--output', type=pathlib.Path)
    parser.add_argument('--seed', type=int, default=1)
    parser.add_argument('--latency-us', type=int, default=0)
    parser.add_argument('--onboard', action='store_true', help='S2: position loop in flight-core')
    args = parser.parse_args()
    available = {**SCENARIOS, **ONBOARD} if args.onboard else SCENARIOS
    names = sorted(available) if args.scenario == 'all' else [args.scenario]
    failed = []
    for name in names:
        options = dict(link_latency_us=args.latency_us, onboard=args.onboard, seed=args.seed)
        if name == 'noisy':
            options.update(NOISY)
        result = run(name, args.output, Config(**options), args.seed)
        for item in result['checks']:
            if not item['passed']:
                print(f'  FAIL {item}')
        print(f"{'pass' if result['passed'] else 'FAIL'} {name}")
        if not result['passed']:
            failed.append(name)
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
