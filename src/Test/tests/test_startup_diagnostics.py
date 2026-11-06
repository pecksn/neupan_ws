"""Real control-loop entry reporting with fake I/O; never publishes to ROS."""
import json
from pathlib import Path
import runpy
import unittest
import test_followup_safety as fixture

ROOT = Path(__file__).resolve().parents[3]


class StartupDiagnosticsTests(unittest.TestCase):
    def test_missing_tf_is_reported_in_ordinary_and_camera_modes(self):
        probe = runpy.run_path(str(ROOT/'tracker_logs/review_20261106/startup_diagnosis/replay_entry_order.py'))['probe']
        for camera in (False, True):
            result = probe(camera)
            self.assertEqual(result['lookups'], 1)
            self.assertEqual(result['stops'], ['vehicle TF unavailable'])

    def test_restored_pose_waiting_for_path_replaces_old_tf_runtime(self):
        def setup(node, clock, namespace):
            node.neupan_planner.initial_path = None
            node.neupan_planner.waypoints = []
            node.diagnostic_stage = 'vehicle_tf'
        node, outputs = fixture.CommandBoundaryTests().run_loop(setup_hook=setup)
        state = json.loads(node.runtime_pub.publish.call_args.args[0])
        self.assertEqual(state['stage'], 'waiting_for_initial_path')
        self.assertEqual(state['command_source'], 'waiting_for_initial_path')
        self.assertNotIn('vehicle TF unavailable', json.dumps(state))
        self.assertTrue(all(value == 0 for value in outputs))

    def test_tf_recovery_does_not_erase_shared_input_recovery_gate(self):
        def setup(node, clock, namespace):
            node.neupan_planner.initial_path = None
            node.neupan_planner.waypoints = []
            original = node.listener.getLatestCommonTime
            calls = [0]
            def lookup(*args):
                calls[0] += 1
                if calls[0] == 1:
                    raise LookupError('missing map')
                return original(*args)
            node.listener.getLatestCommonTime = lookup
        node, outputs = fixture.CommandBoundaryTests().run_loop(setup_hook=setup)
        state = json.loads(node.runtime_pub.publish.call_args.args[0])
        self.assertEqual(state['stage'], 'waiting_for_initial_path')
        self.assertIn('input', state['stop_reasons'])
        self.assertEqual(state['stop_reasons']['input']['stage'], 'input_recovery_pending')
        self.assertNotIn('vehicle TF unavailable', json.dumps(state))
        self.assertTrue(all(value == 0 for value in outputs))


if __name__ == '__main__':
    unittest.main()
