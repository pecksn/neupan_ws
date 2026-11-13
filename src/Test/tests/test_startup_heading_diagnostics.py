"""Problem 1 only: explain startup without changing quality/clock/control gates."""
import ast
import copy
import json
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock
import test_gnss_cadence as cadence


@unittest.skipUnless(cadence.pipeline.ROS,'ROS1 fake callbacks required')
class HeadingStartupDiagnostics(unittest.TestCase):
    setUp=cadence.CadencePipelineTests.setUp
    drive=cadence.CadencePipelineTests.drive
    fix=cadence.CadencePipelineTests.fix

    def summary(self):
        before=copy.deepcopy(self.b.gnss_timebase.__dict__)
        state=(self.b.mode,self.b.reason,self.b.transform_initialized,self.b.stable_since)
        report=self.b.localization_diagnostic_summary(self.now,False)
        self.assertEqual(before,self.b.gnss_timebase.__dict__)
        self.assertEqual(state,(self.b.mode,self.b.reason,self.b.transform_initialized,self.b.stable_since))
        return report

    def test_recorded_float_quality_has_baseline_position_and_time_explanation(self):
        self.drive(float_after=0.)
        path=Path(__file__).resolve().parents[3]/'tracker_logs/review_20261112/field_diagnosis/heading_start_103741.json'
        row=next(r for r in json.loads(path.read_text())['solutions'] if r['heading']=='NARROW_FLOAT')
        self.b.latest_solution_health['payload']['baseline_m']=row['baseline']
        report=self.summary()
        self.assertFalse(self.b.transform_initialized)
        self.assertEqual(report['baseline_m'],row['baseline'])
        self.assertEqual(report['position_type'],'NARROW_INT')
        self.assertEqual(report['heading_satellites'],12)
        self.assertIn('Position Fixed',report['startup_message'])
        self.assertIn('heading not fixed (Float)',report['startup_message'])
        self.assertIn('waiting for initial map heading',report['startup_message'])
        self.assertTrue(report['clock_ready'])
        self.assertIsNotNone(report['latest_pvt']['receive_age_sec'])
        self.assertIsNotNone(report['latest_pvt']['source_age_sec'])

    def test_clock_wait_is_not_mislabeled_as_only_heading_wait(self):
        self.drive(end=1.,float_after=0.)
        report=self.summary()
        self.assertEqual(report['phase'],'waiting_measurement_clock')
        self.assertIn('time not confirmed',report['startup_message'])
        self.assertFalse(report['clock_ready'])

    def test_real_driver_none_and_bad_fixed_geometry_do_not_gain_heading_permission(self):
        from libnmea_navsat_driver.unicore import crc32
        from test_gnss_observation_time import pvts
        self.drive(float_after=0.)
        for index,heading,baseline,phase in ((7,'NONE','0.0','waiting_first_map_heading'),
            (8,'L1_INT','6.18','waiting_heading_geometry')):
            raw=pvts(self.epoch+index*cadence.NS,heading=heading,position='NARROW_INT')
            header,body=raw.split('*')[0].split(';')
            fields=body.split(',');fields[21]=baseline
            text=header+';'+','.join(fields)
            raw=text+'*%08x'%crc32(text[1:])
            self.now=cadence.pipeline.rospy.Time.from_sec(self.epoch/cadence.NS+index+.24)
            self.mono=cadence.NS+index*cadence.NS+240000000
            self.driver.add_sentence(raw,'gps',host_read_ros_ns=self.now.to_nsec(),host_read_mono_ns=self.mono)
            report=self.summary()
            self.assertEqual(report['heading_type'],heading)
            self.assertEqual(report['baseline_m'],float(baseline))
            self.assertEqual(report['phase'],phase,report)
            self.assertFalse(self.b.transform_initialized)
            self.assertFalse(self.driver.heading_pub.publish.called)

    def test_fixed_with_lost_local_odom_reports_local_wait_without_new_gate(self):
        self.drive(end=3.4)
        self.assertTrue(self.b.gnss_timebase.clock_ready)
        self.assertFalse(self.b.transform_initialized)
        # This is a diagnostic input state, not permission to initialize.
        self.b.last_local_recv=None
        report=self.summary()
        self.assertEqual(report['phase'],'waiting_local_odometry',report)
        self.assertFalse(report['local_odom_ready'])


class TerminalHeadingDiagnostics(unittest.TestCase):
    def test_terminal_preserves_zero_and_throttle_while_showing_new_summary(self):
        path=Path(__file__).resolve().parents[3]/'src/neupan_ros/src/neupan_core.py'
        cls=next(v for v in ast.parse(path.read_text()).body if isinstance(v,ast.ClassDef) and v.name=='neupan_core')
        functions=[v for v in cls.body if isinstance(v,ast.FunctionDef) and v.name in ('localization_startup_cb','temporary_input_stop')]
        clock=[100.]
        ros=NS(logwarn_throttle=Mock(),Time=NS(now=lambda:NS(to_sec=lambda:100.)))
        namespace=dict(json=json,time=NS(monotonic=lambda:clock[0]),rospy=ros,
                       Twist=lambda:'ZERO',yellow_terminal_text=lambda v:v)
        exec(compile(ast.Module(body=functions,type_ignores=[]),str(path),'exec'),namespace)
        registry=NS(reasons={},snapshot=lambda:{})
        registry.set=lambda key,*args:registry.reasons.update({key:{}})
        node=NS(vel_pub=Mock(),runtime_pub=Mock(),stop_registry=registry,cycle_metrics=lambda:{})
        summary=dict(phase='waiting_first_map_heading',startup_message='Position Fixed; heading unavailable (no solution); waiting for initial map heading',
            heading_type='NONE',receiver_position_fixed=True,receiver_heading_fixed=False,baseline_m=0.,
            heading_satellites=0,clock_ready=True,clock_wait_reason='clock_verified',
            latest_pvt=dict(source_age_sec=3.2,receive_age_sec=.2,transport_age_sec=3.))
        payload=dict(stamp=100.,initialized=False,diagnostic_summary=summary,gnss_solution={})
        namespace['localization_startup_cb'](node,NS(data=json.dumps(payload)))
        namespace['temporary_input_stop'](node,'vehicle TF unavailable')
        args=ros.logwarn_throttle.call_args.args
        self.assertIn('Position Fixed',args[2]);self.assertIn('no solution',args[2])
        self.assertIn('baseline=0.00 m',args[3])
        self.assertIn('measured 3.20 s ago',args[3]);self.assertIn('received 0.20 s ago',args[3])
        self.assertIn('clock=ready',args[3])
        self.assertNotIn('vehicle TF unavailable',args[2])
        self.assertNotIn('receiver_position_Fixed',args[3])
        clock[0]=101.;namespace['temporary_input_stop'](node,'vehicle TF unavailable')
        self.assertEqual(ros.logwarn_throttle.call_count,1)
        self.assertEqual(node.vel_pub.publish.call_count,2)
        clock[0]=108.;namespace['temporary_input_stop'](node,'vehicle TF unavailable')
        self.assertEqual(ros.logwarn_throttle.call_count,2)
        self.assertEqual(registry.reasons['input']['detail'],'vehicle TF unavailable')


if __name__=='__main__':unittest.main()
