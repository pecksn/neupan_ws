"""Afternoon diagnostics only: actual driver/bridge, no hardware or master."""
import json
import ast
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock
import test_gnss_cadence as cadence


@unittest.skipUnless(cadence.pipeline.ROS,'ROS1 messages required')
class LocalizationDiagnosticTests(unittest.TestCase):
    setUp=cadence.CadencePipelineTests.setUp
    fix=cadence.CadencePipelineTests.fix
    drive=cadence.CadencePipelineTests.drive

    def status(self):
        before=(self.b.mode,self.b.reason,self.b.transform_initialized,
                self.b.gnss_timebase.previous_clock,self.b.gnss_timebase.fault)
        self.b.publish_status(self.now)
        self.assertEqual(before,(self.b.mode,self.b.reason,self.b.transform_initialized,
                self.b.gnss_timebase.previous_clock,self.b.gnss_timebase.fault))
        return json.loads(self.b.status_pub.publish.call_args.args[0].data)['diagnostic_summary']

    def test_float_before_initialization_explicitly_waits_first_heading(self):
        self.drive(float_after=0.)
        summary=self.status()
        self.assertEqual(summary['phase'],'waiting_first_map_heading')
        self.assertFalse(self.b.transform_initialized)
        self.assertEqual(summary['heading_type'],'NARROW_FLOAT')

    def test_fresh_float_stream_is_not_blame_for_old_fixed_expiry(self):
        self.drive(end=10.,float_after=7.)
        old=max(stamp for family,stamp in self.b.gnss_timebase.metadata
                if family=='PVTSLNA' and stamp<self.epoch+7*cadence.NS)
        self.b.observation_buffer_diagnostics['selected_heading_ns']=old
        summary=self.status()
        self.assertEqual(summary['phase'],'local_position_continuation')
        self.assertTrue(summary['latest_pvt']['valid'])
        self.assertFalse(summary['selected_heading']['valid'])
        self.assertEqual(summary['reason'],'latest_heading_solution_unfixed')
        self.assertTrue(self.b.transform_initialized)
        # Transition text must also avoid calling the historical sample a
        # current-stream outage; the internal reason/transition stay native.
        self.b.mode=self.b.TRACKING
        self.b.set_mode(self.b.DEAD_RECKONING,'measurement_receive_stale_PVTSLNA')
        args=cadence.pipeline.rospy.logwarn.call_args.args
        self.assertIn('latest_heading_solution_unfixed',args[3])
        self.assertIn('selected_heading=measurement_receive_stale_PVTSLNA',args[3])
        self.assertEqual(self.b.reason,'measurement_receive_stale_PVTSLNA')

    def test_real_stream_loss_and_late_transport_remain_distinct(self):
        self.drive(end=10.,stop_pvt_after=7.)
        self.assertEqual(self.status()['latest_pvt']['reason'],'measurement_receive_stale_PVTSLNA')
        latest=self.b.gnss_timebase.latest['PVTSLNA']
        stamp=latest['measurement_stamp_ns']
        self.b.gnss_timebase.metadata['PVTSLNA',stamp]=dict(latest,host_read_ros_ns=stamp+700000000)
        self.assertEqual(self.status()['latest_pvt']['reason'],'measurement_transport_late_PVTSLNA')

    def test_gga_logger_preserves_difference_fields_with_limit_note(self):
        from system_run_logger import SystemRunLogger
        from nmea_msgs.msg import Gpgga
        logger=SystemRunLogger.__new__(SystemRunLogger);logger.update=Mock()
        message=Gpgga();message.diff_age=7;message.station_id='0123'
        logger.gga_cb(message)
        value=logger.update.call_args.args[1]
        self.assertEqual(value['diff_age'],7)
        self.assertEqual(value['station_id'],'0123')
        self.assertIn('zero',value['diff_age_note'])


class NeuPANTerminalDiagnosticTests(unittest.TestCase):
    def test_fresh_first_heading_hint_does_not_change_stop_or_claim_running(self):
        path=Path(__file__).resolve().parents[3]/'src/neupan_ros/src/neupan_core.py'
        tree=ast.parse(path.read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='neupan_core')
        methods=[n for n in cls.body if isinstance(n,ast.FunctionDef)
                 and n.name in ('localization_startup_cb','temporary_input_stop')]
        ros=NS(logwarn_throttle=Mock(),Time=NS(now=lambda:NS(to_sec=lambda:100.)))
        namespace=dict(json=json,time=NS(monotonic=lambda:100.),rospy=ros,
                       Twist=lambda:'ZERO',yellow_terminal_text=lambda text:text)
        exec(compile(ast.Module(body=methods,type_ignores=[]),str(path),'exec'),namespace)
        for phase,age,stamp,hint in [('waiting_first_map_heading',0.,100.,True),
                                     ('local_position_continuation',0.,100.,False),
                                     ('waiting_first_map_heading',3.,100.,False),
                                     ('waiting_first_map_heading',0.,90.,False)]:
            ros.logwarn_throttle.reset_mock()
            registry=NS(reasons={},snapshot=lambda:{})
            registry.set=lambda key,*args:registry.reasons.update({key:{}})
            node=NS(vel_pub=Mock(),runtime_pub=Mock(),stop_registry=registry,cycle_metrics=lambda:{})
            message=NS(data=json.dumps(dict(stamp=stamp,initialized=phase!='waiting_first_map_heading',
                diagnostic_summary=dict(phase=phase,reason='latest_heading_solution_unfixed',heading_type='NARROW_FLOAT'),
                latest_heading_health=dict(valid=False),
                gnss_solution=dict(heading_type='NARROW_INT',heading_valid=True))))
            namespace['localization_startup_cb'](node,message)
            node.localization_startup_status=(100.-age,node.localization_startup_status[1])
            namespace['temporary_input_stop'](node,'vehicle TF unavailable')
            node.vel_pub.publish.assert_called_once_with('ZERO')
            args=ros.logwarn_throttle.call_args.args
            self.assertEqual('waiting for first map heading' in args[2],hint)
            self.assertIn('receiver_heading=NARROW_FLOAT, usable=False',args[3])
            self.assertEqual(registry.reasons['input']['detail'],'vehicle TF unavailable')
