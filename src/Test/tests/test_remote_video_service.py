"""Raw-video service tests: fake subprocesses only, no ROS/vehicle startup."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]
SOURCE = ROOT / 'src/hybridnets_receiver/scripts/remote_video_service.py'
spec = importlib.util.spec_from_file_location('remote_video_service_test', SOURCE)
service = importlib.util.module_from_spec(spec)
spec.loader.exec_module(service)


class OnePass:
    def __init__(self, rounds=1): self.rounds = rounds
    def is_set(self): return self.rounds <= 0
    def wait(self, _):
        self.rounds -= 1
        return self.is_set()


class RemoteVideoServiceTests(unittest.TestCase):
    def test_existing_master_is_not_replaced_or_stopped(self):
        with patch.object(service, 'master_pid', return_value=123), \
                patch.object(service.subprocess, 'Popen') as launch, \
                patch.object(service, 'stop_child') as stop:
            service.supervise('master', OnePass())
        launch.assert_not_called()
        stop.assert_called_once_with(None)

    def test_master_mode_starts_only_roscore(self):
        child = NS(poll=lambda:None)
        with patch.object(service, 'master_pid', return_value=None), \
                patch.object(service.subprocess, 'Popen', return_value=child) as launch, \
                patch.object(service, 'stop_child'):
            service.supervise('master', OnePass())
        self.assertEqual(launch.call_args[0][0],
            ['/usr/bin/python3','/opt/ros/noetic/bin/roscore','-p','11311'])
        self.assertTrue(launch.call_args[1]['start_new_session'])

    def test_video_waits_for_master(self):
        with patch.object(service, 'master_pid', return_value=None), \
                patch.object(service.subprocess, 'Popen') as launch:
            service.supervise('video', OnePass())
        launch.assert_not_called()

    def test_video_does_not_replace_existing_port_or_node_owner(self):
        with patch.object(service, 'master_pid', return_value=123), \
                patch.object(service.Path, 'exists', return_value=True), \
                patch.object(service, 'video_busy', return_value=True), \
                patch.object(service.subprocess, 'Popen') as launch:
            service.supervise('video', OnePass())
        launch.assert_not_called()

    def test_video_restarts_only_own_child_when_master_changes(self):
        child = NS(poll=lambda:None)
        with patch.object(service, 'master_pid', side_effect=[123,456]), \
                patch.object(service.Path, 'exists', return_value=True), \
                patch.object(service, 'video_busy', return_value=False), \
                patch.object(service.subprocess, 'Popen', return_value=child) as launch, \
                patch.object(service, 'stop_child') as stop:
            service.supervise('video', OnePass(2))
        self.assertEqual(launch.call_count, 2)
        self.assertEqual(stop.call_count, 2)
        self.assertEqual(launch.call_args[0][0][-2:], ['hybridnets_receiver','video_only.launch'])
        self.assertIn('--wait', launch.call_args[0][0])

    def test_stop_does_not_signal_an_unrelated_reused_pid(self):
        with patch.object(service.Path,'read_text',return_value='123'), \
                patch.object(service.Path,'read_bytes',return_value=b'/usr/bin/python3\0unrelated.py\0master\0'), \
                patch.object(service.os,'kill') as kill:
            service.stop_owned('master')
        kill.assert_not_called()

    def test_stop_requires_exact_service_identity_and_mode(self):
        for command, expected in (([service.SCRIPT,'master'], True), ([service.SCRIPT,'video'], False)):
            with patch.object(service.Path,'read_text',return_value='123'), \
                    patch.object(service.Path,'read_bytes',return_value='\0'.join(command).encode()), \
                    patch.object(service,'wait_for_release') as wait, \
                    patch.object(service.os,'kill') as kill:
                service.stop_owned('master')
            self.assertEqual(kill.called, expected)
            self.assertEqual(wait.called, expected)

    def test_stop_waits_for_container_lock_to_be_released(self):
        with patch.object(service.Path,'open'), \
                patch.object(service.fcntl,'flock',side_effect=[BlockingIOError(),None]) as lock, \
                patch.object(service.time,'sleep') as sleep:
            service.wait_for_release('video')
        self.assertEqual(lock.call_count,2)
        sleep.assert_called_once_with(.1)

    def test_stop_wait_is_bounded(self):
        with patch.object(service.Path,'open'), \
                patch.object(service.fcntl,'flock',side_effect=BlockingIOError()), \
                patch.object(service.time,'monotonic',side_effect=[0.,31.]):
            with self.assertRaises(TimeoutError):service.wait_for_release('video')

    def test_launch_split_keeps_json_in_neupan_not_in_video_service(self):
        package = ROOT / 'src/hybridnets_receiver/launch'
        video = ET.parse(str(package/'video_only.launch')).getroot()
        self.assertEqual([v.get('type') for v in video.findall('node')], ['rtp_camera_receiver.py'])
        receiver = ET.parse(str(package/'receiver.launch')).getroot()
        self.assertEqual(receiver.find("arg[@name='start_video']").get('default'), 'false')
        self.assertIsNotNone(receiver.find("node[@type='hybridnets_json_receiver.py']"))
        full = ET.parse(str(ROOT/'src/neupan_ros/example/lxw_car/launch/full_system_run.launch')).getroot()
        self.assertEqual(full.find("arg[@name='enable_hybridnets_video']").get('default'), 'false')
        self.assertEqual(full.find("arg[@name='enable_hybridnets_receiver']").get('default'), 'true')


if __name__ == '__main__':
    unittest.main()
