"""Test the pure motion guard without ROS or vehicle hardware."""
import ast
import math
from collections import deque
from pathlib import Path
import unittest

source = Path(__file__).resolve().parents[3] / 'src/robot_base_driver/scripts/global_odom_bridge.py'
with source.open() as stream:
    tree = ast.parse(stream.read())
definition = next(node for node in tree.body
                  if isinstance(node, ast.ClassDef) and node.name == 'LocalMotionGuard')
namespace = {'math': math, 'deque': deque}
exec(compile(ast.Module(body=[definition], type_ignores=[]), str(source), 'exec'), namespace)
LocalMotionGuard = namespace['LocalMotionGuard']


class MotionGuardTests(unittest.TestCase):
    def guard(self):
        return LocalMotionGuard(2.0, 0.8, math.radians(90))

    def test_normal_motion(self):
        guard = self.guard()
        results = [guard.observe(10 + i * .02, (.8 * i * .02, 0., 0.)) for i in range(51)]
        self.assertTrue(all(r['accepted'] and not r['fault'] for r in results))
        self.assertAlmostEqual(results[-1]['window_speed_mps'], .8)

    def test_short_interval_small_correction_does_not_stop(self):
        guard = self.guard()
        for i in range(16):
            guard.observe(10 + i * .02, (.8 * i * .02, 0., 0.))
        result = guard.observe(10.301, (.8 * .3 + .045, 0., 0.))
        self.assertGreater(result['message_speed_mps'], 40)
        self.assertIsNone(result['fault'])
        for i in range(16, 31):
            result = guard.observe(10 + i * .02, (.8 * i * .02 + .045, 0., 0.))
            self.assertIsNone(result['fault'])

    def test_position_jump_is_immediate(self):
        guard = self.guard()
        guard.observe(10., (0., 0., 0.))
        self.assertTrue(guard.observe(10.001, (.5, 0., 0.))['fault'].startswith('position_jump'))

    def test_yaw_jump_is_immediate(self):
        guard = self.guard()
        guard.observe(10., (0., 0., 0.))
        self.assertTrue(guard.observe(10.02, (0., 0., math.radians(30)))['fault'].startswith('yaw_jump'))

    def test_persistent_speed_stops_within_window_plus_confirmation(self):
        guard = self.guard()
        faults = []
        for i in range(26):
            result = guard.observe(10 + i * .02, (2.5 * i * .02, 0., 0.))
            if result['fault']:
                faults.append((i * .02, result['fault']))
        self.assertTrue(faults)
        self.assertLessEqual(faults[0][0], .32)
        self.assertTrue(faults[0][1].startswith('window_speed'))

    def test_persistent_lateral_motion_stops(self):
        guard = self.guard()
        results = [guard.observe(10 + i * .02, (0., 1.2 * i * .02, 0.)) for i in range(26)]
        self.assertTrue(any(r['fault'] and 'lateral' in r['fault'] for r in results))

    def test_persistent_yaw_rate_stops(self):
        guard = self.guard()
        results = [guard.observe(10 + i * .02, (0., 0., 2.0 * i * .02)) for i in range(26)]
        self.assertTrue(any(r['fault'] and 'yaw_rate' in r['fault'] for r in results))

    def test_duplicate_and_out_of_order_do_not_replace_history(self):
        guard = self.guard()
        guard.observe(10., (0., 0., 0.))
        self.assertFalse(guard.observe(10., (20., 0., 0.))['accepted'])
        self.assertFalse(guard.observe(9.9, (20., 0., 0.))['accepted'])
        self.assertEqual(guard.history[-1], (10., (0., 0., 0.)))

    def test_gap_stops_and_restarts_window(self):
        guard = self.guard()
        guard.observe(10., (0., 0., 0.))
        self.assertTrue(guard.observe(10.6, (.1, 0., 0.))['fault'].startswith('message_gap'))
        self.assertEqual(len(guard.history), 1)

    def test_invalid_pose_or_stamp_rejected(self):
        guard = self.guard()
        for stamp, pose in [(0., (0., 0., 0.)), (10., (math.nan, 0., 0.))]:
            self.assertFalse(guard.observe(stamp, pose)['accepted'])

    def test_angle_wrap_is_not_a_jump(self):
        guard = self.guard()
        guard.observe(10., (0., 0., math.radians(179)))
        self.assertIsNone(guard.observe(10.1, (0., 0., math.radians(-179)))['fault'])


if __name__ == '__main__':
    unittest.main()
