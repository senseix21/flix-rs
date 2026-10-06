import importlib.util
from pathlib import Path
import unittest
spec=importlib.util.spec_from_file_location('compare',Path(__file__).resolve().parents[1]/'gazebo-compare.py')
compare=importlib.util.module_from_spec(spec);spec.loader.exec_module(compare)

class ComparisonTests(unittest.TestCase):
    def sample(self, motors=.4):
        return dict(pose_wxyz=[1,0,0,0],motors=[motors]*4,position_world_m=[0,0,1])
    def test_opposite_quaternion_sign_same_orientation_and_threshold_failure_kept(self):
        limits=dict(max_attitude_difference_degrees=5,max_motor_absolute_difference=.1,max_position_difference_m=.5)
        left={0:self.sample(),1:self.sample()};right={0:self.sample(),1:self.sample(.6)}
        right[0]['pose_wxyz']=[-1,0,0,0]
        result=compare.compare(left,right,limits)
        self.assertEqual(result['status'],'fail')
        self.assertEqual(result['first_threshold_violation']['relative_tick'],1)
        self.assertEqual(result['metrics']['max_attitude_difference_degrees'],0)
        with self.assertRaises(ValueError):compare.compare({},right,limits)
