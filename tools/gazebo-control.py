#!/usr/bin/env python3
"""Interactive keyboard control for the loopback Gazebo simulation only."""
import os
import select
import sys
import termios
import time
import tty
from pyflix.client import Flix, UdpTransport

class Controls:
    def __init__(self, client):
        self.client = client
        self.armed = False
        self.thrust = 0.0
        self.pulses = {}

    def stop(self):
        self.thrust = 0.0
        self.armed = False
        self.pulses.clear()
        self.client.disarm()

    def key(self, key, now):
        if key == 'a':
            if self.thrust != 0:
                raise ValueError('Disarm with Space before arming')
            self.client.manual(thrust=0)
            self.client.arm()
            self.armed = True
        elif key == ' ':
            self.stop()
        elif key == 'c':
            self.stop()
            self.client.console('clear-fault')
            self.client.set_mode(2)
        elif key == 'w' and self.armed:
            self.thrust = min(1.0, round(self.thrust + .02, 2))
        elif key == 's':
            self.thrust = max(0.0, round(self.thrust - .02, 2))
        elif key in 'ikjlqe' and self.armed:
            axis, value = {'i':('pitch',.2),'k':('pitch',-.2),'j':('roll',-.2),'l':('roll',.2),'q':('yaw',-.3),'e':('yaw',.3)}[key]
            self.pulses[axis] = (value, now + .15)

    def tick(self, now):
        axes = {axis:value for axis,(value,expiry) in self.pulses.items() if now < expiry}
        self.client.manual(thrust=self.thrust if self.armed else 0.0, **axes)

def main():
    if not sys.stdin.isatty():
        raise SystemExit('Run this control client in an interactive terminal.')
    print('GAZEBO SIMULATION ONLY · browser: http://127.0.0.1:8765')
    print('A arm | Space disarm | W/S thrust ±2% | I/K pitch | J/L roll | Q/E yaw')
    print('C clear latched fault while stopped | X exit and disarm')
    print('Start at 0% thrust. Hold W gradually; hover depends on the simulated plant.')
    print('Attitude/yaw keys return to centre after 150 ms unless repeated. Close other clients.')
    original = termios.tcgetattr(sys.stdin)
    with Flix(UdpTransport(), simulation=True) as client:
        controls = Controls(client)
        print('Connecting to simulation…', flush=True)
        deadline = time.monotonic() + 20
        while True:
            try:
                client.get_parameter(0)
                break
            except TimeoutError:
                if time.monotonic() >= deadline:
                    raise
        client.disarm()
        try:
            client.set_mode(2)
        except RuntimeError as error:
            print(f'Mode request denied: {error}. Press C for explicit stopped fault recovery.')
        client.manual(thrust=0)
        try:
            print('Ready · STAB · disarmed. Press A to arm.', flush=True)
            tty.setcbreak(sys.stdin.fileno())
            while True:
                ready, _, _ = select.select([sys.stdin], [], [], .02)
                if ready:
                    key = os.read(sys.stdin.fileno(), 1).decode('ascii', errors='ignore').lower()
                    if key == 'x' or not key:
                        break
                    try:
                        controls.key(key, time.monotonic())
                        print(f'\r{"Armed" if controls.armed else "Disarmed"} · thrust {controls.thrust:.0%}         ', end='', flush=True)
                    except (ValueError, RuntimeError, TimeoutError) as error:
                        print(f'\rRequest denied: {error}\n', end='', flush=True)
                controls.tick(time.monotonic())
        except KeyboardInterrupt:
            pass
        finally:
            try:
                controls.stop()
            finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original)
                print('\nStopped.')

if __name__ == '__main__':
    main()
