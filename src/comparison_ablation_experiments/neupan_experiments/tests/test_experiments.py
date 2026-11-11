"""Offline composition and fake-message tests. Never start ROS nodes/devices."""
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock,patch

ROOT=Path(__file__).resolve().parents[4]
SCRIPTS=Path(__file__).resolve().parents[1]/'scripts'


def load(name):
    spec=importlib.util.spec_from_file_location(name,SCRIPTS/(name+'.py'))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


runner=load('run_experiment');gps=load('gps_always_projection');gate=load('teb_command_gate')


class ExperimentTests(unittest.TestCase):
    def test_compose_changes_only_in_memory_and_selects_exactly_one_planner(self):
        source=ROOT/'src/neupan_ros/example/lxw_car/launch/full_system_run.launch'
        before=source.read_bytes()
        for mode in ('neupan','gps_always','teb'):
            root,nav=runner.compose(mode,ROOT,Path('/tmp/experiment_navigation.launch'))
            names=[v.get('name') for v in root.iter('node')]
            self.assertEqual('move_base' in names,mode=='teb')
            self.assertEqual('neupan_control' in names,mode!='teb')
            self.assertEqual(nav is not None,mode=='gps_always')
            mux=next(v for v in root.iter('node') if v.get('name')=='control_mode_mux')
            self.assertEqual(next(v for v in mux.findall('param') if v.get('name')=='auto_timeout').get('value'),'$(arg auto_command_timeout)')
            if nav is not None:
                p=next(v for v in nav.iter('node') if v.get('name')=='gps_fix_to_map_odom')
                self.assertEqual(p.get('pkg'),'neupan_experiments')
        self.assertEqual(source.read_bytes(),before)

    def test_teb_keeps_radians_and_current_vehicle_width(self):
        root,_=runner.compose('teb',ROOT,Path('/tmp/unused'))
        move=next(v for v in root.iter('node') if v.get('name')=='move_base')
        self.assertEqual(next(v for v in move.findall('param') if v.get('name')=='TebLocalPlannerROS/cmd_angle_instead_rotvel').get('value'),'true')
        footprint=next(v.text for v in move.findall('rosparam') if v.get('param')=='local_costmap/footprint')
        self.assertIn('0.425',footprint)
        self.assertIn('0.81',footprint)

    def test_low_quality_numeric_position_still_publishes_and_preserves_stamp(self):
        node=gps.GpsAlwaysProjection.__new__(gps.GpsAlwaysProjection)
        node.origin=(23.,113.);node.latest_source=None;node.count=0
        node.output=Mock();node.rospy=Mock()
        def odom():
            return NS(header=NS(),pose=NS(pose=NS(position=NS(),orientation=NS()),covariance=[0.]*36))
        node.Odometry=odom
        stamp=NS(to_nsec=lambda:100000000000)
        for seq,status in enumerate((-1,0,1,2)):
            node.latest_source=None
            message=NS(latitude=23.0001,longitude=113.0001,position_covariance=[100.,0,0,0,100.,0,0,0,0],
                header=NS(stamp=stamp),status=NS(status=status))
            node.fix(message)
            self.assertEqual(node.output.publish.call_count,seq+1)
            self.assertIs(node.output.publish.call_args.args[0].header.stamp,stamp)
        node.fix(message)
        self.assertEqual(node.output.publish.call_count,4) # no duplicate lease/measurement

    def test_invalid_numeric_observations_are_not_fabricated(self):
        for lat,cov in ((math.nan,[1.]*9),(91.,[1.]*9),(23.,[math.nan]*9)):
            with self.assertRaises(ValueError):gps.project_fix(lat,113.,cov,23.,113.)

    def test_gps_age_does_not_stop_but_dead_local_stream_does(self):
        node=gps.GpsAlwaysProjection.__new__(gps.GpsAlwaysProjection)
        node.samples={'local':(100.,100.,True),'global':(100.,100.,True)}
        node.lio=(100.,True);node.count=1;node.heading_count=1;node.last_fix=(-100.,-1)
        node.rospy=NS(Time=NS(now=lambda:NS(to_sec=lambda:100.)))
        node.Bool=lambda **kw:NS(**kw);node.String=lambda **kw:NS(**kw)
        node.safety=Mock();node.status=Mock()
        with patch.object(gps.time,'monotonic',return_value=100.):node.tick(None)
        self.assertTrue(node.safety.publish.call_args.args[0].data)
        node.samples['local']=(98.,98.,True)
        with patch.object(gps.time,'monotonic',return_value=100.):node.tick(None)
        self.assertFalse(node.safety.publish.call_args.args[0].data)

    def test_float_heading_is_used_with_own_stamp_but_invalid_epoch_is_not(self):
        import json
        node=gps.GpsAlwaysProjection.__new__(gps.GpsAlwaysProjection)
        node.heading_source=None;node.heading_count=0;node.rotation=90.;node.north_cw=True
        node.heading_output=Mock();node.rospy=NS(Time=lambda sec,ns:(sec,ns))
        node.Odometry=lambda:NS(header=NS(),pose=NS(pose=NS(orientation=NS()),covariance=[0.]*36))
        packet=dict(heading_deg=30.,stamp_ns=100000000001,heading_fixed=False,heading_valid=False)
        node.heading(NS(data=json.dumps(packet)))
        self.assertEqual(node.heading_count,1)
        result=node.heading_output.publish.call_args.args[0]
        self.assertEqual(result.header.stamp,(100,1))
        self.assertAlmostEqual(result.pose.pose.orientation.z,math.sin(math.radians(-30.)/2))
        packet['stamp_ns']=0;node.heading(NS(data=json.dumps(packet)))
        self.assertEqual(node.heading_count,1)

    def test_teb_watchdog_does_not_replay_old_commands_or_accept_nonfinite_controls(self):
        self.assertTrue(gate.fresh(100.,100.,100.1,100.1))
        self.assertFalse(gate.fresh(99.,100.,100.1,100.1))
        self.assertFalse(gate.fresh(100.,98.,100.1,100.1))
        node=gate.TebCommandGate.__new__(gate.TebCommandGate)
        node.command=(0.,None)
        node.command_cb(NS(linear=NS(x=math.nan),angular=NS(z=0.)))
        self.assertIsNone(node.command)

    def test_recorder_config_and_invalid_scan_cannot_silently_pass(self):
        import yaml
        recorder=yaml.safe_load((SCRIPTS.parent/'config/experiment_recording.yaml').read_text())
        self.assertIn('/experiments/gnss_heading',recorder['topics'])
        self.assertIn('/experiments/teb_cmd_raw',recorder['topics'])
        node=gate.TebCommandGate.__new__(gate.TebCommandGate)
        message=NS(header=NS(stamp=NS(to_sec=lambda:100.)),range_min=.4,range_max=30.,ranges=[math.nan]*20)
        node.scan_cb(message);self.assertFalse(node.scan_valid)
        message.ranges=[1.,2.,3.];node.scan_cb(message);self.assertTrue(node.scan_valid)


if __name__=='__main__':unittest.main()
