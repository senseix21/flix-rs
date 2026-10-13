#!/usr/bin/env python3
"""Generate a Gazebo world with N Flix drones for the swarm viewer (swarm plan S1, Gazebo view).

Each drone is a copy of the pinned legacy model with the Rust plugin configured for its own
loopback command port (14600 + i) and trace port (14700 + i), the soft contact profile, and the
model name flix1..flixN. Start positions match flix_swarm.gazebo.layout.

    python3 scripts/gazebo-swarm-world.py LEGACY_MODEL_DIR OUT_DIR COUNT
"""
import pathlib
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = pathlib.Path(__file__).resolve().parents[1]
SPACING = 0.6
COMMAND_BASE = 14600
TRACE_BASE = 14700


def main():
    source, out, count = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2]), int(sys.argv[3])
    if not 2 <= count <= 8:
        raise SystemExit('COUNT must be 2..8 (owner registry bound)')
    models = out / 'models'
    if models.exists():
        shutil.rmtree(models)
    world = ET.parse(ROOT / 'adapters/gazebo/bench.world')
    root = world.getroot().find('world')
    for include in root.findall('include'):
        root.remove(include)
    for i in range(count):
        name = f'flix_swarm_{i + 1}'
        target = models / name
        shutil.copytree(source, target)
        sdf = target / 'flix.sdf'
        tree = ET.parse(sdf)
        plugin = tree.find('.//plugin')
        if plugin is None or plugin.get('filename') != 'libflix.so':
            raise SystemExit('legacy model plugin not found')
        plugin.set('filename', 'libflix_rust.so')
        ET.SubElement(plugin, 'udp_port').text = str(COMMAND_BASE + i)
        ET.SubElement(plugin, 'trace_port').text = str(TRACE_BASE + i)
        tree.write(sdf)
        config = ET.parse(target / 'model.config')
        config.find('name').text = name
        config.write(target / 'model.config')
        subprocess.run([sys.executable, str(ROOT / 'scripts/gazebo-contact.py'), str(sdf), 'soft'], check=True)
        include = ET.SubElement(root, 'include')
        ET.SubElement(include, 'uri').text = f'model://{name}'
        ET.SubElement(include, 'name').text = f'flix{i + 1}'
        ET.SubElement(include, 'pose').text = f'{i * SPACING} 0 0.0138 0 0 0'
    world.write(out / 'swarm.world')


if __name__ == '__main__':
    main()
