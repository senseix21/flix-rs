#!/usr/bin/env python3
"""Opt-in contact compliance for the Flix model (evidence/plant-parity).

Default Gazebo/ODE contact is near-rigid: the 65 g body chatters at the 1 ms step
and arrests landings within one step. `soft` gives the body collision a critically
damped 30 Hz contact (kp = m*wn^2, kd = 2*m*wn), 0.28 mm static sag. Applied to both
engines' copies of the model so paired runs share one plant. Default: unchanged."""
import sys
import xml.etree.ElementTree as ET

PROFILES = {'soft': dict(kp='2309', kd='24.5', max_vel='0.1', min_depth='0.0001')}

def main():
    path, profile = sys.argv[1], sys.argv[2]
    if profile in ('', 'rigid'):
        return
    values = PROFILES[profile]
    tree = ET.parse(path)
    collision = tree.find(".//link[@name='body']/collision")
    assert collision is not None, 'flix body collision not found'
    assert collision.find('surface') is None, 'model already defines a surface'
    ode = ET.SubElement(ET.SubElement(ET.SubElement(collision, 'surface'), 'contact'), 'ode')
    for key, value in values.items():
        ET.SubElement(ode, key).text = value
    tree.write(path)

if __name__ == '__main__':
    main()
