"""G90 late-observation contract; fake clocks, no hardware or ROS master."""
from pathlib import Path
import sys
import unittest
import math
from unittest.mock import patch,Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'src/robot_base_driver/scripts'))
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'src/robot_base_driver/src'))
from test_gnss_observation_time import ObservationTiming,NS,pvts,stamp,nmea
from gnss_timebase import GnssTimebase
import test_gnss_measurement_contract as pipeline
from test_gnss_auto_start import gga
from libnmea_navsat_driver.unicore import crc32,parse_pvtslna


class LateTimingTests(unittest.TestCase):
    def test_one_positive_late_packet_does_not_poison_clock_identity(self):
        obs=ObservationTiming('measurement','auto',lambda:True,pvt_period_sec=1.,gga_period_sec=.05)
        t=stamp()
        for i in range(81):
            source=t+i*50000000;now=source+150000000;mono=NS+i*50000000
            if i%20==0:obs.observe(pvts(source),now,mono)
            obs.observe(nmea('GNGGA,0000%02d.%03d,2302.4,N,11323.4,E,4,12,1,5,M,0,M,,0000'%
                ((i*50)//1000,(i*50)%1000)),now,mono)
        self.assertTrue(obs.clock_verified)
        event=obs.observe(pvts(t+5*NS),t+5*NS+507000000,NS+5*NS+357000000)
        self.assertFalse(obs.auto_failed,'ordinary positive delay permanently latched clock')
        gate=GnssTimebase('measurement','auto')
        gate.receive(event,event['host_read_ros_ns'],event['host_read_mono_ns'])
        self.assertIsNone(gate.fault)
        for i in range(121,181):
            source=t+i*50000000;now=source+150000000;mono=NS+i*50000000
            if i%20==0:
                event=obs.observe(pvts(source),now,mono);gate.receive(event,now,mono)
            event=obs.observe(gga(source),now,mono);gate.receive(event,now,mono)
        self.assertTrue(obs.clock_verified)
        self.assertTrue(gate.clock_ready)
        self.assertIsNone(gate.fault)

    def test_candidate_requires_independent_utc_assertion(self):
        with self.assertRaises(ValueError):
            ObservationTiming('measurement','auto',lambda:True,transport_limit_sec=4.)

    def test_four_second_candidate_accepts_source_time_not_now(self):
        obs=ObservationTiming('measurement',True,transport_limit_sec=4.,common_utc_verified=True)
        t=stamp();event=obs.observe(pvts(t),t+3*NS,NS)
        self.assertEqual(event['epoch_order'],'new')
        self.assertEqual(event['measurement_stamp_ns'],t)
        gate=GnssTimebase('measurement',True);gate.receive(event,t+3*NS,NS)
        self.assertTrue(gate.check('PVTSLNA',t,t+3*NS,NS,4.,1.,.3)[0])
        self.assertFalse(gate.check('PVTSLNA',t,t+5*NS,3*NS,4.,1.,.3)[0])
        late=obs.observe(pvts(t+NS),t+5*NS+100000000,3*NS+100000000)
        self.assertEqual(late['epoch_order'],'rejected_age')
        gate=GnssTimebase('measurement',True)
        gate.receive(late,late['host_read_ros_ns'],late['host_read_mono_ns'])
        self.assertEqual(gate.check('PVTSLNA',t+NS,late['host_read_ros_ns'],
            late['host_read_mono_ns'],4.,1.,.3)[1],'measurement_transport_late_PVTSLNA')

    def test_known_fixed_types_do_not_include_float_or_unknown(self):
        for kind in ('NARROW_INT','L1_INT','NARROW_FLOAT','L1_FLOAT','NONE','WIDE_INT','UNKNOWN'):
            result=parse_pvtslna(pvts(stamp(),heading=kind))
            self.assertEqual(result['heading_fixed'],kind in ('NARROW_INT','L1_INT'))
            self.assertEqual(result['heading_quality_class'],kind)

    def test_duplicate_never_renews_receive_and_session_jump_stays_latched(self):
        obs=ObservationTiming('measurement',True,transport_limit_sec=4.,common_utc_verified=True)
        t=stamp();event=obs.observe(pvts(t),t+3*NS,NS)
        gate=GnssTimebase('measurement','auto');gate.receive(event,t+3*NS,NS)
        gate.receive(dict(event,clock_verified=False,clock_waiting=True),t+3*NS+100000000,NS+100000000)
        gate.receive(dict(event,session_id='other'),t+3*NS+200000000,NS+200000000)
        self.assertEqual(gate.fault,'time_session_changed_restart_required')
        gate=GnssTimebase('measurement',True);gate.receive(event,t+3*NS,NS)
        gate.receive(event,t+4*NS,2*NS)
        self.assertFalse(gate.check('PVTSLNA',t,t+4*NS+400000000,2*NS+400000000,4.,1.,.3)[0])
        gate.tick(t+5*NS,2*NS+500000000)
        self.assertEqual(gate.fault,'host_clock_jump_restart_required')


@unittest.skipUnless(pipeline.ROS,'ROS1 fake messages required')
class DelayedMovingPipelineTests(unittest.TestCase):
    fix=pipeline.MeasurementPipelineTests.fix
    local=pipeline.MeasurementPipelineTests.local

    def setUp(self):
        pipeline.MeasurementPipelineTests.setUp(self)
        self.params.update({'~pvt_period_sec':1.,'~gga_period_sec':.05,'~low_precision_startup':True,
            '~gps_time_clock_verified':'auto','~gnss_transport_limit_sec':4.,'~gnss_common_utc_verified':True})
        import test_runtime_position as fixture
        with patch.object(pipeline.rospy,'Publisher',side_effect=lambda *a,**k:Mock()):
            self.b=fixture.RuntimePositionTests.bridge(self,initialized=False)
            self.projection=pipeline.MeasurementPipelineTests.projection(self)
        self.driver.observation_timing=ObservationTiming('measurement','auto',lambda:True,
            pvt_period_sec=1.,gga_period_sec=.05,transport_limit_sec=4.,common_utc_verified=True)
        def fix(msg):self.projection.fix_cb(msg)
        def metadata(msg):
            self.b.time_metadata_cb(msg);self.projection.time_metadata_cb(msg)
        self.driver.fix_pub.publish.side_effect=lambda msg:(self.b.fix_cb(msg),fix(msg))
        self.projection.pub.publish.side_effect=lambda msg:self.fix(self.driver.fix_pub.publish.call_args.args[0])
        self.driver.time_diagnostics_pub.publish.side_effect=metadata
        self.driver.gpgga_pub.publish.side_effect=self.b.gga_cb
        self.driver.heading_pub.publish.side_effect=self.b.heading_cb
        self.driver.solution_pub.publish.side_effect=self.b.solution_cb

    def test_three_second_gps_aligns_history_then_outputs_current_moving_pose(self):
        for tick in range(701):
            t=tick*.02;self.now=pipeline.rospy.Time.from_sec(self.epoch/NS+t);self.mono=NS+round(t*NS)
            self.local(t)
            if tick>=162 and (tick-162)%50==0:
                source=self.epoch+(tick-162)*20000000
                body=pvts(source,position='NARROW_INT').split('*')[0]
                head,payload=body.split(';');fields=payload.split(',')
                fields[22]=str((90.-math.degrees(.3+(source-self.epoch)/NS))%360.)
                body=head+';'+','.join(fields)
                self.driver.add_sentence(body+'*{:08x}'.format(crc32(body[1:])),'gps',
                    host_read_ros_ns=self.now.to_nsec(),host_read_mono_ns=self.mono)
            if tick>=165 and tick%5==0:
                source=self.epoch+tick*20000000-3150000000
                utc=gga(source).split('*')[0][1:]
                self.driver.add_sentence(nmea(utc+',2302.4,N,11323.4,E,4,12,1,5,M,0,M,,0000'),'gps',
                    host_read_ros_ns=self.now.to_nsec(),host_read_mono_ns=self.mono)
            self.projection.timer_cb(None);self.b.update_guard(self.now);self.b.publish_global_odom(self.now)
        self.assertTrue(self.b.transform_initialized,self.b.reason)
        self.assertAlmostEqual(self.b.transform_yaw,.3,places=3)
        self.assertAlmostEqual(self.b.transform_x,100.,places=3)
        self.assertAlmostEqual(self.b.transform_y,20.,places=3)
        current=self.b.pub.publish.call_args.args[0].pose.pose.position
        expected=100.+math.cos(.3)*math.sin(t)-math.sin(.3)*(1-math.cos(t))
        self.assertAlmostEqual(current.x,expected,places=3)
        self.assertLess(len(self.b.local_pose_history),10000)
        self.assertIsNone(self.b.local_pose_at(self.epoch)[0])
        self.b.local_pose_history.clear()
        self.assertIsNone(self.b.local_pose_at(self.epoch+11*NS)[0],
                          'a reset/gap must not invent historical LIO')
        self.now+=pipeline.rospy.Duration(1.1);self.mono+=1100000000
        self.assertFalse(self.b.localization_is_safe(self.now),'current LIO must still expire in 1s')

    def test_l1_installation_sanity_and_latest_float_cannot_borrow_fixed(self):
        self.driver.observation_timing=ObservationTiming('measurement',True)
        self.now=pipeline.rospy.Time.from_sec(self.epoch/NS+.3);self.mono=NS+300000000
        for i,(kind,baseline,pitch,count) in enumerate([('L1_INT',.67,0.,1),('L1_INT',4.,0.,1),
                ('L1_INT',.67,75.,1),('NARROW_INT',.67,0.,2),('NARROW_FLOAT',.67,0.,2)]):
            source=self.epoch+100000000+i*20000000
            body=pvts(source,heading=kind).split('*')[0]
            head,payload=body.split(';');fields=payload.split(',');fields[21]=str(baseline);fields[23]=str(pitch)
            body=head+';'+','.join(fields)
            self.driver.add_sentence(body+'*{:08x}'.format(crc32(body[1:])),'gps',
                host_read_ros_ns=self.now.to_nsec(),host_read_mono_ns=self.mono)
            self.assertEqual(self.driver.heading_pub.publish.call_count,count)
            if kind=='L1_INT' and (baseline>1.2 or pitch>45):
                self.assertEqual(self.b.latest_heading_health(self.now)['reason'],'latest_heading_geometry_invalid')
        self.assertFalse(self.b.latest_heading_health(self.now)['valid'])

    def test_high_rate_history_is_time_bounded_and_has_a_hard_memory_cap(self):
        for i in range(4001):
            self.b.local_pose_history.append((self.epoch+i*2500000,(i*.0025,0.,0.)))
        pose,_=self.b.local_pose_at(self.epoch+3*NS)
        self.assertAlmostEqual(pose[0],3.)
        for i in range(2200):
            t=self.epoch+i*1000000
            self.b.remember_observation('solution',dict(stamp_ns=t,stamp=t/NS),
                pipeline.rospy.Time(t//NS,t%NS))
        self.assertEqual(len(self.b.observation_history['solution']),2048)
        t=self.epoch+20*NS
        self.b.remember_observation('solution',dict(stamp_ns=t,stamp=t/NS),pipeline.rospy.Time(t//NS))
        self.assertEqual(len(self.b.observation_history['solution']),1)

    def test_full_launch_candidate_is_shared_without_changing_realtime_timeouts(self):
        import roslaunch
        path=Path(__file__).resolve().parents[3]/'src/neupan_ros/example/lxw_car/launch/full_system_run.launch'
        params=[]
        for camera in ('false','true'):
            config=roslaunch.config.load_config_default([(str(path),['outdoor_mode:=true',
                'enable_camera_right_control:='+camera,'gps_transport_limit_sec:=4.0',
                'gps_common_utc_verified:=true'])],None,verbose=False)
            subset={key:value.value for key,value in config.params.items() if key.startswith(
                ('/global_odom_bridge/','/gps_fix_to_map_odom/','/nmea_serial_driver_node/'))}
            for node in ('global_odom_bridge','gps_fix_to_map_odom','nmea_serial_driver_node'):
                self.assertEqual(subset['/'+node+'/gnss_transport_limit_sec'],4.)
                self.assertTrue(subset['/'+node+'/gnss_common_utc_verified'])
            self.assertEqual(subset.get('/global_odom_bridge/data_timeout_sec',self.b.data_timeout_sec),1.)
            self.assertEqual(subset['/nmea_serial_driver_node/baud'],115200)
            params.append(subset)
        self.assertEqual(params[0],params[1])
