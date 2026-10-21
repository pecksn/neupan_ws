"""Command mux safety-heartbeat regression; fake messages, no vehicle nodes."""
import json
import math
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'src/robot_base_driver/scripts'))
import rospy
from control_mode_mux import ControlModeMux
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String, UInt8


class SafetyHeartbeatTests(unittest.TestCase):
    def setUp(self):
        self.now = rospy.Time(100)
        self.monotonic = 100.
        self.params = {'~enable_auto_localization_safety_stop': True}
        for name in ('init_node', 'Publisher', 'Subscriber', 'Timer', 'logwarn'):
            stub = patch.object(rospy, name)
            mock = stub.start()
            if name == 'Publisher':
                mock.side_effect = lambda *a, **kw: MagicMock()
            self.addCleanup(stub.stop)
        for stub in (
                patch.object(rospy, 'get_param', lambda k, d=None: self.params.get(k, d)),
                patch.object(rospy.Time, 'now', lambda: self.now),
                patch('time.monotonic', lambda: self.monotonic)):
            stub.start()
            self.addCleanup(stub.stop)

    def auto(self, mux, speed=.8):
        command = Twist()
        command.linear.x = speed
        mux.auto_callback(command)
        return command

    def test_no_signal_never_allows_auto(self):
        mux = ControlModeMux()
        self.auto(mux)
        command, reason = mux.selected_command(self.now)
        self.assertEqual(command.linear.x, 0.)
        self.assertEqual(reason, 'localization_safety_missing')

    def test_fresh_safe_signal_allows_auto(self):
        mux = ControlModeMux()
        original = self.auto(mux)
        mux.localization_safety_callback(Bool(data=True))
        command, reason = mux.selected_command(self.now)
        self.assertIs(command, original)
        self.assertEqual(reason, 'auto_active')

    def test_timeout_stops_despite_fresh_planner_commands(self):
        mux = ControlModeMux()
        mux.localization_safety_callback(Bool(data=True))
        self.monotonic += 1.01
        self.auto(mux)
        command, reason = mux.selected_command(self.now)
        self.assertEqual(command.linear.x, 0.)
        self.assertEqual(reason, 'localization_safety_timeout')

    def test_exact_timeout_boundary_is_closed(self):
        mux = ControlModeMux()
        self.auto(mux)
        mux.localization_safety_callback(Bool(data=True))
        self.monotonic += 1.
        self.assertEqual(mux.selected_command(self.now)[1], 'localization_safety_timeout')

    def test_new_heartbeats_extend_deadline_without_fixed_requirement(self):
        mux = ControlModeMux()
        for _ in range(30):
            self.monotonic += .2
            self.auto(mux)
            mux.localization_safety_callback(Bool(data=True))
            self.assertEqual(mux.selected_command(self.now)[1], 'auto_active')

    def test_fresh_false_signal_stops_immediately(self):
        mux = ControlModeMux()
        self.auto(mux)
        mux.localization_safety_callback(Bool(data=True))
        mux.output_pub.publish.reset_mock()
        mux.localization_safety_callback(Bool(data=False))
        self.assertEqual(mux.output_pub.publish.call_args.args[0].linear.x, 0.)
        self.assertEqual(mux.selected_command(self.now)[1], 'localization_unsafe')

    def test_false_heartbeats_cannot_restore_auto(self):
        mux = ControlModeMux()
        for _ in range(10):
            self.monotonic += .2
            self.auto(mux)
            mux.localization_safety_callback(Bool(data=False))
            self.assertEqual(mux.selected_command(self.now)[1], 'localization_unsafe')

    def test_repeated_unsafe_heartbeats_do_not_flood_status_logs(self):
        mux = ControlModeMux()
        mux.localization_safety_callback(Bool(data=False))
        mux.status_pub.publish.reset_mock()
        rospy.logwarn.reset_mock()
        for _ in range(30):
            mux.localization_safety_callback(Bool(data=False))
        mux.status_pub.publish.assert_not_called()
        rospy.logwarn.assert_not_called()

    def test_valid_signal_restores_but_not_stale_auto_commands(self):
        mux = ControlModeMux()
        self.auto(mux)
        mux.localization_safety_callback(Bool(data=True))
        self.monotonic += 2.
        self.now += rospy.Duration(2.)
        self.assertEqual(mux.selected_command(self.now)[1], 'localization_safety_timeout')
        mux.localization_safety_callback(Bool(data=True))
        self.assertEqual(mux.selected_command(self.now)[1], 'auto_timeout')
        self.auto(mux)
        self.assertEqual(mux.selected_command(self.now)[1], 'auto_active')

    def test_manual_takeover_remains_available_after_timeout(self):
        mux = ControlModeMux()
        mux.localization_safety_callback(Bool(data=True))
        self.monotonic += 2.
        command = Twist()
        command.linear.x = -.2
        mux.remote_callback(command)
        mux.mode_callback(UInt8(data=mux.MANUAL))
        self.assertIs(mux.selected_command(self.now)[0], command)
        self.assertEqual(mux.selected_command(self.now)[1], 'remote_active')
        mux.mode_callback(UInt8(data=mux.AUTO))
        self.assertEqual(mux.output_pub.publish.call_args.args[0].linear.x, 0.)
        self.assertEqual(mux.selected_command(self.now)[1], 'localization_safety_timeout')

    def test_manual_remote_timeout_is_unchanged(self):
        mux = ControlModeMux()
        mux.mode = mux.MANUAL
        mux.remote_callback(Twist())
        self.now += rospy.Duration(2.)
        self.assertEqual(mux.selected_command(self.now)[1], 'remote_timeout')

    def test_disabled_guard_preserves_indoor_behavior(self):
        self.params['~enable_auto_localization_safety_stop'] = False
        mux = ControlModeMux()
        original = self.auto(mux)
        self.monotonic += 100.
        self.assertIs(mux.selected_command(self.now)[0], original)

    def test_ros_time_freeze_or_jump_cannot_extend_safety_lease(self):
        for ros_second in (100, 101, 99):
            with self.subTest(ros_second=ros_second):
                self.monotonic = 100.
                mux = ControlModeMux()
                mux.localization_safety_callback(Bool(data=True))
                self.monotonic += 2.
                self.now = rospy.Time(ros_second)
                self.auto(mux)
                self.assertEqual(mux.selected_command(self.now)[1], 'localization_safety_timeout')

    def test_configurable_deadline(self):
        self.params['~localization_safety_timeout'] = 3.
        mux = ControlModeMux()
        mux.localization_safety_callback(Bool(data=True))
        self.monotonic += 2.
        self.auto(mux)
        self.assertEqual(mux.selected_command(self.now)[1], 'auto_active')
        self.monotonic += 1.1
        self.assertEqual(mux.selected_command(self.now)[1], 'localization_safety_timeout')

    def test_invalid_deadline_is_rejected(self):
        for value in (0., -1., math.nan, math.inf):
            with self.subTest(value=value):
                self.params['~localization_safety_timeout'] = value
                with self.assertRaises(ValueError):
                    ControlModeMux()

    def test_timer_stops_and_reports_effective_not_cached_safety(self):
        mux = ControlModeMux()
        mux.localization_safety_callback(Bool(data=True))
        self.monotonic += 1.1
        self.auto(mux)
        mux.timer_callback(None)
        self.assertEqual(mux.output_pub.publish.call_args.args[0].linear.x, 0.)
        state = json.loads(mux.status_pub.publish.call_args.args[0].data)
        self.assertFalse(state['localization_safe'])
        self.assertTrue(state['localization_safe_reported'])
        self.assertEqual(state['reason'], 'localization_safety_timeout')
        self.assertGreater(state['localization_safety_age_sec'], 1.)
        self.assertEqual(state['localization_safety_timeout_sec'], 1.)
        rospy.logwarn.assert_called()

    def test_periodic_status_updates_heartbeat_age_without_warning_spam(self):
        mux = ControlModeMux()
        mux.localization_safety_callback(Bool(data=True))
        self.auto(mux)
        mux.timer_callback(None)
        mux.status_pub.publish.reset_mock()
        self.monotonic += .6
        mux.timer_callback(None)
        mux.status_pub.publish.assert_called_once()
        self.assertAlmostEqual(json.loads(mux.status_pub.publish.call_args.args[0].data)
                               ['localization_safety_age_sec'], .6)


def ros_smoke_worker():
    """Run only in a child interpreter, with an isolated master and topic tree."""
    import signal
    import socket
    import subprocess
    import tempfile
    import time

    with tempfile.TemporaryDirectory(prefix='mux_safety_ros_') as log_dir:
        with socket.socket() as reservation:
            reservation.bind(('127.0.0.1', 0))
            port = reservation.getsockname()[1]
        env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:'+str(port),
                   ROS_HOSTNAME='127.0.0.1', ROS_LOG_DIR=log_dir)
        for name in ('ROS_MASTER_URI', 'ROS_HOSTNAME', 'ROS_LOG_DIR'):
            os.environ[name] = env[name]
        master = subprocess.Popen(['roscore', '-p', str(port)], env=env,
                                  stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        process = None
        try:
            rospy.init_node('mux_safety_test', anonymous=True, disable_signals=True)
            prefix = '/mux_safety_test/'
            parameters = {'enable_auto_localization_safety_stop':'true',
                          'localization_safety_timeout':'1.0', 'publish_rate':'40.',
                          'mode_topic':prefix+'mode', 'auto_topic':prefix+'auto',
                          'remote_topic':prefix+'remote', 'output_topic':prefix+'selected',
                          'status_topic':prefix+'status', 'safety_topic':prefix+'safe'}
            process = subprocess.Popen(['rosrun', 'robot_base_driver', 'control_mode_mux.py',
                '__name:=mux_safety_test_node']+['_'+k+':='+v for k,v in parameters.items()],
                env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            outputs, states = [], []
            subscribers = [
                rospy.Subscriber(prefix+'selected', Twist,
                                 lambda msg:outputs.append((time.monotonic(),msg.linear.x))),
                rospy.Subscriber(prefix+'status', String,
                                 lambda msg:states.append(json.loads(msg.data)))]
            publishers = {name:rospy.Publisher(prefix+name, kind, queue_size=1)
                          for name,kind in (('mode',UInt8), ('auto',Twist),
                                            ('remote',Twist), ('safe',Bool))}
            deadline = time.monotonic()+10.
            while not all(p.get_num_connections() for p in publishers.values()):
                assert process.poll() is None, 'mux exited before publishers connected'
                assert time.monotonic()<deadline, 'ROS topic connection timeout'
                time.sleep(.02)

            def wait_for(reason, speed, safety=None, manual=False):
                deadline = time.monotonic()+3.
                started = time.monotonic()
                while time.monotonic()<deadline:
                    assert process.poll() is None, 'mux exited during safety test'
                    command = Twist()
                    command.linear.x = .8
                    publishers['auto'].publish(command)  # Planner never stops.
                    remote = Twist()
                    remote.linear.x = -.2
                    publishers['remote'].publish(remote)
                    publishers['mode'].publish(UInt8(data=1 if manual else 0))
                    if safety is not None:
                        publishers['safe'].publish(Bool(data=safety))
                    if (states and states[-1]['reason']==reason and outputs
                            and outputs[-1][0]>started and abs(outputs[-1][1]-speed)<1.e-6):
                        return time.monotonic(), states[-1]
                    time.sleep(.02)
                raise AssertionError('Expected '+reason+', latest='+str(states[-1:]))

            wait_for('localization_safety_missing', 0.)
            last_safe_sent, _ = wait_for('auto_active', .8, safety=True)
            stopped_at, state = wait_for('localization_safety_timeout', 0.)
            elapsed = stopped_at-last_safe_sent
            assert .85<elapsed<1.5, 'unsafe heartbeat expiry delay: '+str(elapsed)
            assert state['localization_safe'] is False
            assert state['localization_safe_reported'] is True
            assert state['localization_safety_age_sec']>=1.
            wait_for('remote_active', -.2, manual=True)
            wait_for('localization_safety_timeout', 0.)
            wait_for('localization_unsafe', 0., safety=False)
            wait_for('auto_active', .8, safety=True)
            print('ROS mux smoke OK: missing -> active -> silence/zero after %.3fs '
                  '-> manual takeover -> AUTO blocked -> false blocked -> fresh true restored' % elapsed)
        finally:
            rospy.signal_shutdown('isolated safety heartbeat test complete')
            for child in (process, master):
                if child is not None and child.poll() is None:
                    child.send_signal(signal.SIGINT)
                    try:
                        child.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        child.kill()
                        child.wait(timeout=5)


@unittest.skipUnless(os.environ.get('MUX_SAFETY_ROS_SMOKE') == '1', 'optional isolated ROS mux test')
class SafetyHeartbeatRosTest(unittest.TestCase):
    def test_real_ros_heartbeat_loss_and_manual_takeover(self):
        import subprocess
        # Keep rospy.init_node()/shutdown in a separate interpreter, so this
        # test can coexist with the GPS bridge's independent ROS smoke test.
        result = subprocess.run([sys.executable, __file__, '--ros-smoke-worker'],
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, timeout=45)
        self.assertEqual(result.returncode, 0, result.stdout)
        print(result.stdout.strip())


if __name__ == '__main__':
    if '--ros-smoke-worker' in sys.argv:
        ros_smoke_worker()
    else:
        unittest.main()
