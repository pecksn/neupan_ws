"""Offline log/bag acceptance: mocked ROS lifecycle, real temporary JSONL/bag indexes.

Never opens a serial port, starts a ROS node, or publishes onto the vehicle.
"""
import json
import math
import signal
import subprocess
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import MagicMock, patch

import rospy
import rosbag
import yaml
from geometry_msgs.msg import Twist
from std_msgs.msg import String, UInt8

WORKSPACE = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(WORKSPACE / 'src/robot_base_driver/scripts'))
sys.path.insert(0, str(WORKSPACE / 'src/vehicle_bag_recorder/scripts'))
from system_run_logger import SystemRunLogger
from bag_recorder import BagRecorder


class LoggingRecordsTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        self.params = {'~log_dir': str(self.directory)}
        for target in ('init_node', 'Subscriber', 'Timer', 'on_shutdown',
                       'loginfo', 'logwarn', 'logerr', 'logwarn_throttle'):
            stub = patch.object(rospy, target)
            stub.start()
            self.addCleanup(stub.stop)
        for stub in (
            patch.object(rospy, 'get_param', lambda key, default=None: self.params.get(key, default)),
            patch.object(rospy.Time, 'now', lambda: rospy.Time(100)),
        ):
            stub.start()
            self.addCleanup(stub.stop)

    def logger(self):
        logger = SystemRunLogger()
        self.addCleanup(logger.on_shutdown)
        return logger

    @staticmethod
    def records(logger):
        def reject_constant(token):
            raise ValueError('non-standard JSON constant: ' + token)
        return [json.loads(line, parse_constant=reject_constant)
                for line in Path(logger.log_path).read_text().splitlines()]

    def test_all_command_sources_and_nested_status_use_strict_json(self):
        logger = self.logger()
        for value in (math.nan, math.inf, -math.inf):
            msg = Twist()
            msg.linear.x, msg.angular.z = value, value
            logger.auto_command_cb(msg)
            logger.command_cb(msg)
            logger.remote_command_cb(msg)
            logger.command_audit_cb(String(data=json.dumps({'request': {'speed': value}})))
            logger.neupan_runtime_cb(String(data=json.dumps({'nested': [value, {'valid': 1.25}]})))
            logger.timer_cb(None)
            rows = self.records(logger)
            sample = rows[-1]['data']
            for source in ('auto_command', 'command', 'remote_command'):
                self.assertEqual(sample[source]['linear_x'], repr(value))
            self.assertEqual(sample['command_audit']['request']['speed'], repr(value))
            self.assertEqual(sample['neupan_runtime']['nested'], [repr(value), {'valid': 1.25}])
        events = [row for row in self.records(logger) if row['record_type'] == 'auto_command_event']
        self.assertEqual(len(events), 3)
        self.assertEqual([row['data']['linear_x'] for row in events], ['nan', 'inf', '-inf'])

    def test_recursive_normalization_preserves_normal_values_and_input(self):
        logger = self.logger()
        original = {'record_type': 'test', 'nested': [math.nan, (math.inf, -math.inf)],
                    'finite': [0, 2.5, True, None, 'NaN is evidence']}
        self.assertTrue(logger.write_record(original))
        row = self.records(logger)[-1]
        self.assertEqual(row['nested'], ['nan', ['inf', '-inf']])
        self.assertEqual(row['finite'], original['finite'])
        self.assertTrue(math.isnan(original['nested'][0]))
        self.assertIsInstance(original['nested'][1], tuple)

    def test_unserializable_record_is_contained_and_next_record_still_saved(self):
        logger = self.logger()
        self.assertFalse(logger.write_record({'bad': object()}))
        self.assertTrue(logger.write_record({'record_type': 'test', 'ok': 3.5}))
        self.assertEqual(self.records(logger)[-1]['ok'], 3.5)

    def test_new_subscriptions_and_topic_metadata(self):
        logger = self.logger()
        subscriptions = {call.args[0]: call.args[1] for call in rospy.Subscriber.call_args_list}
        self.assertIs(subscriptions['/cmd_vel_remote'], Twist)
        self.assertIs(subscriptions['/vehicle/control_mode'], UInt8)
        self.assertIs(subscriptions['/vehicle/command_audit'], String)
        topics = self.records(logger)[0]['topics']
        self.assertEqual(topics['remote_command'], '/cmd_vel_remote')
        self.assertEqual(topics['mode_request'], '/vehicle/control_mode')
        self.assertEqual(topics['command_audit'], '/vehicle/command_audit')

    def test_every_remote_and_mode_request_including_invalid_is_recorded(self):
        logger = self.logger()
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = .8, .25, .2
        logger.remote_command_cb(msg)
        msg.linear.x = math.nan
        logger.remote_command_cb(msg)
        for value in (1, 0, 99):
            logger.mode_request_cb(UInt8(data=value))
        events = [row for row in self.records(logger) if row['record_type'].endswith('_event')]
        self.assertEqual(len(events), 5)
        self.assertEqual(events[0]['data']['linear_y'], .25)
        self.assertEqual(events[1]['data']['linear_x'], 'nan')
        self.assertEqual(events[-1]['data']['requested_name'], 'INVALID')
        for row in events:
            self.assertEqual(row['stamp'], 100.)
            self.assertIn('wall_stamp', row)
            json.dumps(row, allow_nan=False)

    def test_audit_events_survive_bursts_and_periodic_snapshot_keeps_latest(self):
        logger = self.logger()
        for number in range(3):
            logger.command_audit_cb(String(data=json.dumps({'stamp': number, 'write_status': 'written'})))
        logger.timer_cb(None)
        rows = self.records(logger)
        events = [row for row in rows if row['record_type'] == 'command_audit_event']
        self.assertEqual([row['data']['stamp'] for row in events], [0, 1, 2])
        self.assertEqual(rows[-1]['record_type'], 'sample')
        self.assertEqual(rows[-1]['data']['command_audit']['stamp'], 2)
        self.assertEqual(rows[-1]['data']['command_audit_age_sec'], 0.)

    def recorder(self, required):
        path = self.directory / 'config.yaml'
        path.write_text(yaml.safe_dump({
            'output_directory': str(self.directory), 'session_subdirectory': False,
            'min_free_space_gb': 0, 'topics': required, 'required_topics': required,
        }))
        recorder = BagRecorder(str(path))
        recorder.process = MagicMock()
        recorder.process.poll.return_value = 0
        return recorder

    def test_real_split_bag_indexes_aggregated_and_manifest_preserved(self):
        recorder = self.recorder(['/vehicle/command_audit'])
        for number in range(2):
            with rosbag.Bag(recorder.output_prefix + '_%d.bag' % number, 'w') as bag:
                bag.write('/vehicle/command_audit', String(data='{}'), rospy.Time(100+number))
        recorder._record_bag_inventory()
        manifest = json.loads(Path(recorder.output_prefix + '.json').read_text())
        self.assertEqual(manifest['config_file'], recorder.config_path)
        result = manifest['recording_result']
        self.assertTrue(result['index_verification_complete'])
        self.assertTrue(result['all_required_topics_recorded'])
        self.assertEqual(result['topic_message_counts'], {'/vehicle/command_audit': 2})
        self.assertEqual(len(result['files']), 2)

    def test_publishers_do_not_prove_messages_recorded(self):
        recorder = self.recorder(['/gps/fix'])
        recorder.seen_required_topics.add('/gps/fix')
        with rosbag.Bag(recorder.output_prefix + '.bag', 'w') as bag:
            bag.write('/other', String(data='x'), rospy.Time(100))
        recorder._record_bag_inventory()
        result = recorder.manifest['recording_result']
        self.assertTrue(result['index_verification_complete'])
        self.assertFalse(result['all_required_topics_recorded'])
        self.assertEqual(result['required_topics_without_messages'], ['/gps/fix'])

    def test_unfinalized_or_absent_bag_never_claims_verified(self):
        recorder = self.recorder(['/gps/fix'])
        recorder._record_bag_inventory()
        self.assertFalse(recorder.manifest['recording_result']['index_verification_complete'])
        Path(recorder.output_prefix + '.bag.active').touch()
        recorder._record_bag_inventory()
        result = recorder.manifest['recording_result']
        self.assertFalse(result['all_required_topics_recorded'])
        self.assertEqual(len(result['unfinalized_files']), 1)

    def test_corrupt_bag_records_error_instead_of_claiming_complete(self):
        recorder = self.recorder(['/gps/fix'])
        Path(recorder.output_prefix + '.bag').write_bytes(b'not a rosbag')
        recorder._record_bag_inventory()
        result = recorder.manifest['recording_result']
        self.assertFalse(result['index_verification_complete'])
        self.assertTrue(result['inspection_errors'])
        self.assertFalse(result['all_required_topics_recorded'])

    def test_normal_stop_updates_manifest_without_starting_rosbag_or_signalling_vehicle(self):
        recorder = self.recorder(['/vehicle/control_mode'])
        with rosbag.Bag(recorder.output_prefix + '.bag', 'w') as bag:
            bag.write('/vehicle/control_mode', UInt8(data=1), rospy.Time(100))
        with patch('bag_recorder.os.killpg') as kill:
            recorder.stop()
            recorder.stop()
            kill.assert_not_called()
        result = json.loads(Path(recorder.output_prefix + '.json').read_text())['recording_result']
        self.assertEqual(result['topic_message_counts']['/vehicle/control_mode'], 1)
        self.assertTrue(result['all_required_topics_recorded'])

    def finalized_bag(self, recorder):
        path = recorder.output_prefix + '.bag'
        with rosbag.Bag(path, 'w') as bag:
            bag.write('/vehicle/control_mode', UInt8(data=1), rospy.Time(100))
        return Path(path)

    def test_double_timeout_saves_incomplete_manifest_and_never_sends_sigkill(self):
        recorder = self.recorder(['/vehicle/control_mode'])
        bag_path = self.finalized_bag(recorder)
        original = bag_path.read_bytes()
        recorder.process.poll.return_value = None
        recorder.process.wait.side_effect = subprocess.TimeoutExpired('mock rosbag', 20)
        with patch('bag_recorder.os.getpgid', return_value=12345), patch('bag_recorder.os.killpg') as kill:
            recorder.stop()
            recorder.stop()
        self.assertEqual([call.args[1] for call in kill.call_args_list], [signal.SIGINT, signal.SIGTERM])
        result = json.loads(Path(recorder.output_prefix + '.json').read_text())['recording_result']
        self.assertEqual(result['shutdown']['outcome'], 'stop_timeout')
        self.assertEqual(len(result['shutdown']['errors']), 2)
        self.assertTrue(result['process_running'])
        self.assertIsNone(result['return_code'])
        self.assertFalse(result['index_verification_complete'])
        self.assertFalse(result['all_required_topics_recorded'])
        self.assertEqual(result['topic_message_counts']['/vehicle/control_mode'], 1)
        self.assertEqual(bag_path.read_bytes(), original)

    def test_sigterm_after_timeout_records_success_and_prior_error(self):
        recorder = self.recorder(['/vehicle/control_mode'])
        self.finalized_bag(recorder)
        recorder.process.poll.side_effect = [None, 0]
        recorder.process.wait.side_effect = [subprocess.TimeoutExpired('mock', 20), 0]
        with patch('bag_recorder.os.getpgid', return_value=12345), patch('bag_recorder.os.killpg'):
            recorder.stop()
        result = recorder.manifest['recording_result']
        self.assertEqual(result['shutdown']['outcome'], 'sigterm_exit')
        self.assertEqual(result['shutdown']['errors'][0]['stage'], 'sigint_wait')
        self.assertEqual(result['return_code'], 0)
        self.assertFalse(result['process_running'])
        self.assertTrue(result['index_verification_complete'])

    def test_process_exit_race_during_getpgid_or_signal_still_saves(self):
        for target in ('getpgid', 'killpg'):
            with self.subTest(stage=target):
                recorder = self.recorder(['/vehicle/control_mode'])
                self.finalized_bag(recorder)
                recorder.process.poll.side_effect = [None, 0]
                with patch('bag_recorder.os.getpgid', return_value=12345) as getpgid, \
                        patch('bag_recorder.os.killpg') as kill:
                    (getpgid if target == 'getpgid' else kill).side_effect = ProcessLookupError('mock process exited')
                    recorder.stop()
                result = recorder.manifest['recording_result']
                self.assertEqual(result['shutdown']['errors'][0]['error_type'], 'ProcessLookupError')
                self.assertFalse(result['process_running'])
                self.assertEqual(result['return_code'], 0)
                self.assertTrue(result['index_verification_complete'])

    def test_manifest_failure_preserves_original_then_repeat_stop_retries_without_signals(self):
        recorder = self.recorder(['/vehicle/control_mode'])
        bag_path = self.finalized_bag(recorder)
        manifest_path = Path(recorder.output_prefix + '.json')
        original_manifest, original_bag = manifest_path.read_bytes(), bag_path.read_bytes()
        with patch('bag_recorder.os.replace', side_effect=OSError('mock write denied')):
            recorder.stop()
        self.assertFalse(recorder.inventory_saved)
        self.assertEqual(manifest_path.read_bytes(), original_manifest)
        with patch('bag_recorder.os.killpg') as kill:
            recorder.stop()
            recorder.stop()
            kill.assert_not_called()
        result = json.loads(manifest_path.read_text())['recording_result']
        self.assertTrue(recorder.inventory_saved)
        self.assertEqual(result['shutdown']['errors'][0]['stage'], 'inventory_save')
        self.assertEqual(bag_path.read_bytes(), original_bag)

    def test_poll_error_saves_unknown_process_state_and_incomplete_result(self):
        recorder = self.recorder(['/vehicle/control_mode'])
        self.finalized_bag(recorder)
        recorder.process.poll.side_effect = OSError('mock poll failure')
        recorder.stop()
        result = json.loads(Path(recorder.output_prefix + '.json').read_text())['recording_result']
        self.assertIsNone(result['process_running'])
        self.assertFalse(result['index_verification_complete'])
        self.assertEqual(result['shutdown']['outcome'], 'stop_error')
        self.assertTrue(result['inspection_errors'])

    def test_all_profiles_include_small_volume_evidence_without_duplicates(self):
        required = {'/vehicle/control_mode', '/vehicle/control_mode/status',
                    '/vehicle/command_audit', '/cmd_vel_remote', '/gnss/solution_status',
                    '/gps_goal_ll', '/move_base_simple/goal', '/goal_arrived', '/osm_global_path_ll'}
        for name in ('vehicle_data', 'hybridnets_data', 'camera_right_test'):
            with self.subTest(profile=name):
                config = yaml.safe_load((WORKSPACE / ('src/vehicle_bag_recorder/config/' + name + '.yaml')).read_text())
                topics = config['topics']
                self.assertEqual(len(topics), len(set(topics)))
                self.assertTrue(required <= set(topics))
                self.assertTrue(set(config['required_topics']) <= set(topics))


if __name__ == '__main__':
    unittest.main()
