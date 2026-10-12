import importlib.util
import json
import pathlib
import struct
import tempfile
import unittest
from gazebo_telemetry import Telemetry
from pyflix.client import pack

spec = importlib.util.spec_from_file_location('viewer', pathlib.Path(__file__).resolve().parents[1]/'gazebo-viewer.py')
viewer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(viewer)

class ViewerTests(unittest.TestCase):
    def test_pose_uses_model_world_frame_not_relative_link_frame(self):
        block='pose { name: "flix" position {x: 1 y: 2 z: 3} orientation {x: 0 y: 0 z: 0 w: 1}}'
        self.assertEqual(viewer.parse_pose(block)['position'], [1.,2.,3.])
        self.assertIsNone(viewer.parse_pose(block.replace('"flix"','"flix::body"')))

    def test_swarm_models_are_named_and_others_ignored(self):
        block='pose { name: "flix3" position {x: 1 y: 2 z: 3} orientation {x: 0 y: 0 z: 0 w: 1}}'
        self.assertEqual(viewer.parse_pose(block)['name'], 'flix3')
        for other in ('flix9', 'flix12', 'flix3::body', 'ground', 'xflix1'):
            self.assertIsNone(viewer.parse_pose(block.replace('"flix3"', f'"{other}"')))

    def test_malformed_pose_does_not_replace_live_state(self):
        for block in ['', 'pose {}', 'pose {name: "flix"}', 'pose {name: "flix" position {x: nan y: 0 z: 0} orientation {x: 0 y: 0 z: 0 w: 1}}', 'pose {name: "flix" position {x: 0 y: 0 z: 0} orientation {x: 0 y: 0 z: 0 w: 0}}']:
            self.assertIsNone(viewer.parse_pose(block))

    def test_observer_reads_valid_motor_frames_and_rejects_invalid_values(self):
        with tempfile.TemporaryDirectory() as folder:
            destination=pathlib.Path(folder)/'snapshot.json'
            observer=Telemetry(destination)
            observer.feed(pack(140,struct.pack('<Q8fB',0,.1,.2,.3,.4,0,0,0,0,0)))
            self.assertAlmostEqual(json.loads(destination.read_text())['motors'][2],.3)
            before=dict(observer.state)
            observer.feed(pack(140,struct.pack('<Q8fB',0,float('nan'),0,0,0,0,0,0,0,0)))
            self.assertEqual(observer.state,before)

    def test_observation_io_failure_does_not_interrupt_transport(self):
        with tempfile.TemporaryDirectory() as folder:
            path=pathlib.Path(folder)/'directory';path.mkdir()
            observer=Telemetry(path)
            observer.feed(pack(0,struct.pack('<IBBBBB',2,2,0,1,3,3)))
            self.assertFalse(observer.state['armed'])
            self.assertFalse(observer.state['fault'])

control_spec = importlib.util.spec_from_file_location('control', pathlib.Path(__file__).resolve().parents[1]/'gazebo-control.py')
control = importlib.util.module_from_spec(control_spec)
control_spec.loader.exec_module(control)

class ControlTests(unittest.TestCase):
    def test_zero_thrust_arm_expiring_stick_and_explicit_stop(self):
        class Client:
            def __init__(self):self.calls=[]
            def manual(self,**values):self.calls.append(('manual',values))
            def arm(self):self.calls.append(('arm',))
            def disarm(self):self.calls.append(('disarm',))
        client=Client();keys=control.Controls(client)
        keys.key('w',0);self.assertEqual(keys.thrust,0)
        keys.key('a',0);self.assertEqual(client.calls[0],('manual',{'thrust':0}))
        keys.key('w',0);keys.key('i',1);keys.tick(1.1)
        self.assertEqual(client.calls[-1],('manual',{'thrust':.02,'pitch':.2}))
        keys.tick(1.2);self.assertEqual(client.calls[-1],('manual',{'thrust':.02}))
        keys.key(' ',1.3);keys.tick(1.4)
        self.assertFalse(keys.armed);self.assertEqual(keys.pulses,{})
        self.assertEqual(client.calls[-1],('manual',{'thrust':0}))

from gazebo_browser_control import BrowserControl
import threading
import queue
import time

class BrowserControlTests(unittest.TestCase):
    def instance(self):
        control=BrowserControl.__new__(BrowserControl)
        control.lock=threading.Lock();control.armed=True;control.ready=True;control.error=None
        control.inputs=dict(roll=0.,pitch=0.,yaw=0.,thrust=0.)
        control.stop_requested=threading.Event();control.commands=queue.Queue(8);control.disarmed_at=0.
        return control

    def test_input_validation_is_atomic_and_rejects_nonfinite_or_out_of_bounds(self):
        control=self.instance();values=dict(roll=.2,pitch=0.,yaw=0.,thrust=.3)
        control.input(values);self.assertEqual(control.inputs,values)
        for invalid in [dict(values,thrust=-.1),dict(values,roll=float('nan')),dict(values,yaw=2),dict(values,thrust=True),{'thrust':.2}]:
            with self.assertRaises(ValueError):control.input(invalid)
            self.assertEqual(control.inputs,values)
        self.assertGreater(control.expires,time.monotonic());self.assertLessEqual(control.expires-time.monotonic(),.5)

    def test_reserved_disarm_revokes_input_even_when_command_queue_full(self):
        control=self.instance()
        for _ in range(8):control.commands.put(('arm',0,queue.Queue(1)))
        control.input(dict(roll=.2,pitch=0,yaw=0,thrust=.3))
        control.request('disarm')
        self.assertFalse(control.armed);self.assertTrue(control.stop_requested.is_set())
        self.assertEqual(control.inputs,dict(roll=0,pitch=0,yaw=0,thrust=0))
        with self.assertRaises(ValueError):control.input(dict(roll=0,pitch=0,yaw=0,thrust=.1))

    def test_reset_disarms_cancels_queued_arm_and_bounds_restart(self):
        from unittest.mock import patch, MagicMock
        import subprocess
        for failure in (None, subprocess.TimeoutExpired('docker', 12)):
            control=self.instance();control.closed=threading.Event();control.container='test-simulation'
            control.expires=0
            reset_reply,arm_reply=queue.Queue(1),queue.Queue(1)
            control.commands.put(('reset',time.monotonic(),reset_reply))
            control.commands.put(('arm',time.monotonic(),arm_reply))
            client=MagicMock();client.__enter__.return_value=client
            with patch('gazebo_browser_control.Flix',return_value=client), patch('gazebo_browser_control.UdpTransport'), patch('gazebo_browser_control.subprocess.run',side_effect=failure) as restart:
                control.run_session()
            self.assertFalse(control.armed);self.assertFalse(control.ready)
            self.assertEqual(control.inputs,dict(roll=0,pitch=0,yaw=0,thrust=0))
            client.arm.assert_not_called();self.assertGreaterEqual(client.disarm.call_count,2)
            self.assertIn('cancelled',arm_reply.get_nowait())
            self.assertEqual(bool(reset_reply.get_nowait()),failure is not None)
            restart.assert_called_once_with(['docker','restart','test-simulation'],check=True,timeout=12,capture_output=True)
