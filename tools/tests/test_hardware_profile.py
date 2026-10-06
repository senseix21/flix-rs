import importlib.util
import json
import pathlib
import unittest
ROOT=pathlib.Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('hardware_profile',ROOT/'scripts/hardware-profile.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
class HardwareProfileTests(unittest.TestCase):
    def test_uninspected_supplier_template_is_rejected(self):
        p=json.loads((ROOT/'fixtures/hardware/esp32s3-mpu6050.json').read_text())
        errors=module.validate(p)
        self.assertTrue(any('missing' in e for e in errors))
        self.assertTrue(any('enable' in e for e in errors))
    def test_conflicts_partition_and_discharge_fail_the_gate(self):
        p=json.loads((ROOT/'fixtures/hardware/esp32s3-mpu6050.json').read_text())
        p.update(reserved_pins=[26],flash_mb=8,partition_bytes=100,image_bytes=101)
        p['pins']={name:4 for name in p['pins']};p['power'].update(battery_discharge_a=.8,loaded_motor_a=.5)
        errors=module.validate(p)
        self.assertTrue(any('overlaps' in e for e in errors))
        self.assertTrue(any('fit flash' in e for e in errors))
        self.assertTrue(any('margin' in e for e in errors))
