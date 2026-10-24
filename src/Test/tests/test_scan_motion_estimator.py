"""Velocity estimation must work without simulator IDs, poses or actor data."""
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch
import threading
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'neupan_ros/example/lxw_car_simulation/scripts'))
from scan_motion_estimator import ScanMotionEstimator,PredictiveYieldGuard

class MotionTest(unittest.TestCase):
    @staticmethod
    def points(t):
        angle=np.linspace(-1.,1.,12)
        static=np.array([4+.3*np.cos(angle),-2+.3*np.sin(angle)])
        moving=np.array([10+.4*np.cos(angle),3-.55*t+.4*np.sin(angle)])
        return np.hstack((static,moving))
    def test_world_static_cluster_stays_static_and_moving_cluster_gets_velocity(self):
        tracker=ScanMotionEstimator()
        for i in range(8):vel,moving=tracker.update(self.points(i*.1),i*.1)
        np.testing.assert_allclose(vel[:,:12],0,atol=1e-9)
        np.testing.assert_allclose(vel[:,12:],np.tile([[0.],[-.55]],(1,12)),atol=1e-9)
        self.assertEqual(len(moving),1)
    def test_insufficient_history_does_not_invent_motion(self):
        tracker=ScanMotionEstimator()
        for i in range(3):vel,_=tracker.update(self.points(i*.1),i*.1)
        np.testing.assert_array_equal(vel,0)
    def test_gap_discards_old_velocity(self):
        tracker=ScanMotionEstimator()
        for i in range(8):tracker.update(self.points(i*.1),i*.1)
        vel,_=tracker.update(self.points(2),2)
        np.testing.assert_array_equal(vel,0)
    def test_unrealistic_jump_not_associated(self):
        tracker=ScanMotionEstimator()
        tracker.update(self.points(0),0)
        vel,_=tracker.update(self.points(.1)+5,.1)
        np.testing.assert_array_equal(vel,0)

    def test_sliding_visible_window_on_static_wall_is_not_object_motion(self):
        tracker=ScanMotionEstimator()
        for i in range(8):
            points=np.array([np.full(12,4.),np.linspace(0,1,12)+.05*i])
            vel,_=tracker.update(points,.1*i)
        np.testing.assert_array_equal(vel,0)

    def test_planar_object_moving_normal_to_its_surface_is_retained(self):
        tracker=ScanMotionEstimator()
        for i in range(8):
            points=np.array([np.full(12,4.+.055*i),np.linspace(-.4,.4,12)])
            vel,_=tracker.update(points,.1*i)
        np.testing.assert_allclose(vel,np.tile([[.55],[0]],(1,12)),atol=1e-9)

class YieldGuardTest(unittest.TestCase):
    def test_predicted_crossing_stops_before_current_overlap(self):
        guard=PredictiveYieldGuard()
        self.assertTrue(guard.update(np.zeros((3,1)),[{'centre':[4,2],'velocity':[0,-.6],'radius_m':.65}],0))
        self.assertEqual(guard.status['reason'],'predicted_conflict')
    def test_moving_away_does_not_stop(self):
        guard=PredictiveYieldGuard()
        self.assertFalse(guard.update(np.zeros((3,1)),[{'centre':[4,2],'velocity':[0,.6],'radius_m':.65}],0))
    def test_release_requires_stable_clear_interval(self):
        guard=PredictiveYieldGuard();state=np.zeros((3,1))
        guard.update(state,[{'centre':[4,2],'velocity':[0,-.6]}],0)
        self.assertTrue(guard.update(state,[],.1))
        self.assertTrue(guard.update(state,[],.59))
        self.assertFalse(guard.update(state,[],.61))
    def test_rotated_world_has_same_yield_decision(self):
        guard=PredictiveYieldGuard()
        self.assertTrue(guard.update(np.array([1,2,np.pi/2]),[{'centre':[-1,6],'velocity':[.6,0]}],0))

class CaptureTimingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import outdoor_motion_controller as controller
        cls.module=controller

    def setUp(self):
        self.node=object.__new__(self.module.OutdoorMotionController)
        self.node.motion_lock=threading.RLock();self.node.estimator=ScanMotionEstimator()
        self.node.motion_enabled=True;self.node.yield_guard=None
        self.node.robot_state=np.zeros((3,1));self.node.map_frame='map'
        self.node.listener=Mock();self.node.listener.lookupTransform.return_value=([0,0,0],[0,0,0,1])
        self.node.flip_angle=False;self.node.scan_range=[.1,20];self.node.scan_angle_range=[-3.14,3.14]
        self.node.scan_downsample=1;self.node.scan_self_filter=False
        self.node.mark_scan_valid=Mock();self.node.mark_scan_fault=Mock()
        self.node.quat_to_yaw_list=lambda q:0.
        class Stamp:
            def __init__(self,t):self.t=t
            def to_sec(self):return self.t
            def __sub__(self,other):return Stamp(self.t-other.t)
        self.Stamp=Stamp
        self.msg=SimpleNamespace(header=SimpleNamespace(stamp=Stamp(10.004),frame_id='base_link'),
            ranges=[2.,2.,2.,2.,2.],angle_min=0.,angle_increment=.1,range_min=.1,range_max=30.)
        self.node.motion_pending=self.msg

    def test_scan_ahead_of_clock_waits_then_uses_original_timestamp(self):
        with patch.object(self.module.rospy.Time,'now',return_value=self.Stamp(10.)):
            self.node.process_motion_scan(None)
        self.node.listener.lookupTransform.assert_not_called()
        self.node.mark_scan_fault.assert_not_called()
        self.assertIs(self.node.motion_pending,self.msg)
        with patch.object(self.module.rospy.Time,'now',return_value=self.Stamp(10.02)):
            self.node.process_motion_scan(None)
        self.assertIsNone(self.node.motion_pending)
        self.assertEqual(self.node.motion_snapshot[0],10.004)
        self.node.mark_scan_valid.assert_called_once()

    def test_tf_delay_is_retried_without_resetting_fresh_scan_health(self):
        self.node.listener.lookupTransform.side_effect=self.module.tf.LookupException('not yet')
        with patch.object(self.module.rospy.Time,'now',return_value=self.Stamp(10.02)):
            self.node.process_motion_scan(None)
        self.assertIs(self.node.motion_pending,self.msg)
        self.node.mark_scan_fault.assert_not_called()
        self.node.listener.lookupTransform.side_effect=None
        with patch.object(self.module.rospy.Time,'now',return_value=self.Stamp(10.04)):
            self.node.process_motion_scan(None)
        self.node.mark_scan_valid.assert_called_once()

    def test_both_motion_features_off_does_not_execute_estimator(self):
        self.node.motion_enabled=False;self.node.estimator=Mock()
        with patch.object(self.module.rospy.Time,'now',return_value=self.Stamp(10.02)):
            self.node.process_motion_scan(None)
        self.node.estimator.update.assert_not_called()
        np.testing.assert_array_equal(self.node.motion_snapshot[2],0)
        self.assertEqual(self.node.motion_snapshot[3],[])
        self.node.listener.lookupTransform.assert_called_once_with('map','base_link',self.msg.header.stamp)

    def test_old_capture_still_faults(self):
        with patch.object(self.module.rospy.Time,'now',return_value=self.Stamp(10.6)):
            self.node.process_motion_scan(None)
        self.node.mark_scan_fault.assert_called_once_with('capture_stamp_expired')
        self.node.listener.lookupTransform.assert_not_called()

    def test_switch_read_after_base_initializes_ros_private_namespace(self):
        import tempfile
        with tempfile.TemporaryDirectory() as folder:
            initialized=[]
            def base_init(node):
                initialized.append(True)
                node.dune_checkpoint='test-model'
                node.planner_config_file=str(Path(folder)/'config.yaml')
                Path(node.planner_config_file).write_text('pan: {iter_num: 5, iter_threshold: 0.01}')
                scalar=Mock();scalar.detach.return_value=1.
                node.neupan_planner=SimpleNamespace(ref_speed=.8,ipath=SimpleNamespace(dt=.1,T=50),pan=SimpleNamespace(iter_num=2,iter_threshold=.1,dune_max_num=100,nrmp_max_num=10,nrmp_layer=SimpleNamespace(**{k:scalar for k in ('q_s','p_u','eta','d_max','d_min')})))
            def parameter(name,default=None):
                self.assertTrue(initialized,'ROS private parameters require node initialization')
                return {'~diagnostic_log_dir':folder,'~simulation_motion_estimation':False,'~simulation_predictive_yield':True}.get(name,default)
            with patch.object(self.module.neupan_core,'__init__',base_init),patch.object(self.module.rospy,'get_param',side_effect=parameter),patch.object(self.module.rospy,'Timer'),patch.object(self.module.rospy,'on_shutdown'):
                node=self.module.OutdoorMotionController()
                try:
                    self.assertFalse(node.motion_enabled)
                    self.assertIsNotNone(node.yield_guard)
                    self.assertEqual(node.neupan_planner.pan.iter_num,5)
                    self.assertEqual(node.neupan_planner.pan.iter_threshold,.01)
                finally:node.motion_log.close()

    def test_native_velocity_input_is_capture_aligned_and_age_compensated(self):
        points=np.array([[13.],[1.]])
        velocities=np.array([[0.],[-.55]])
        owner=SimpleNamespace(motion_enabled=True,motion_lock=threading.RLock(),motion_log=None,yield_guard=None,
            motion_snapshot=(10.,points,velocities,[]))
        native=Mock(return_value=(np.zeros((2,1)),{'stop':False}))
        proxy=self.module.MotionPlanner(native,owner)
        with patch.object(self.module.rospy,'get_time',return_value=10.2):
            proxy(np.zeros((3,1)),points)
        np.testing.assert_allclose(native.call_args[0][1],[[13.],[.89]])
        np.testing.assert_array_equal(native.call_args[0][2],velocities)

    def test_disabled_velocity_input_retains_points_and_independent_guard(self):
        points=np.array([[13.],[1.]])
        guard=Mock();guard.update.return_value=True;guard.status={'holding':True}
        owner=SimpleNamespace(motion_enabled=False,motion_lock=threading.RLock(),motion_log=None,yield_guard=guard,
            motion_snapshot=(10.,points,np.array([[0.],[-.55]]),[{'centre':[13,1]}]))
        native=Mock(return_value=(np.ones((2,1)),{'stop':False}))
        with patch.object(self.module.rospy,'get_time',return_value=10.2):
            action,info=self.module.MotionPlanner(native,owner)(np.zeros((3,1)),points)
        np.testing.assert_array_equal(native.call_args[0][1],points)
        self.assertIsNone(native.call_args[0][2])
        guard.update.assert_called_once()
        np.testing.assert_array_equal(action,0)
        self.assertTrue(info['stop'])

    def test_restart_winner_restores_warm_state_and_does_not_advance_progress_twice(self):
        class Planner:
            def __init__(self):
                self.ipath=SimpleNamespace(point_index=12,curve_index=0)
                self.cur_vel_array=np.zeros((2,3));self.ref_speed=.8;self.info={};self.calls=0
                self.pan=SimpleNamespace(current_nom_values=[],dune_layer=SimpleNamespace(min_distance=0,obstacle_points=None))
            def __call__(self,*args):
                self.calls+=1;self.ipath.point_index+=1
                self.cur_vel_array[:]=self.calls
                self.pan.current_nom_values=[self.calls]
                self.pan.dune_layer.min_distance=self.calls
                self.pan.dune_layer.obstacle_points=self.calls
                self.info={'stop':False,'candidate':self.calls}
                return np.full((2,1),self.calls),self.info
        planner=Planner()
        with patch.object(self.module,'trajectory_score',side_effect=[(False,10),(False,100)]):
            action,info,audit=self.module.solve_with_restarts(planner,np.zeros((3,1)),None)
        self.assertEqual(audit['selected'],'warm')
        self.assertEqual(planner.ipath.point_index,13)
        np.testing.assert_array_equal(action,1)
        np.testing.assert_array_equal(planner.cur_vel_array,1)
        self.assertEqual(planner.pan.current_nom_values,[1])
        self.assertEqual(planner.pan.dune_layer.min_distance,1)
        self.assertEqual(planner.info['candidate'],1)

    def test_restart_scoring_rejects_low_tracking_cost_plan_that_penetrates_scan(self):
        scalar=Mock();scalar.detach.return_value=1.
        tensor=Mock();tensor.detach.return_value.cpu.return_value.numpy.return_value=np.zeros(1)
        planner=SimpleNamespace(cur_vel_array=np.zeros((2,1)),ipath=SimpleNamespace(dt=.1),
            robot=SimpleNamespace(vertices=np.array([[-.14,-.14,.81,.81],[-.425,.425,-.425,.425]])),
            pan=SimpleNamespace(nrmp_layer=SimpleNamespace(**{key:scalar for key in ('q_s','p_u','eta','d_min','d_max')})))
        points=np.array([[1.],[0.]])
        def info(y):
            states=[np.zeros((3,1)),np.array([[1.],[y],[0.]])]
            return {'stop':False,'opt_state_list':states,'ref_state_list':[np.zeros((3,1)),np.array([[1.],[0.],[0.]])],'ref_speed_tensor':tensor}
        unsafe=self.module.trajectory_score(planner,info(0),points)
        safe=self.module.trajectory_score(planner,info(1.5),points)
        self.assertTrue(unsafe[0]);self.assertFalse(safe[0])
        self.assertLess(safe,unsafe)

    def test_stale_velocity_snapshot_is_not_fed_to_solver(self):
        owner=SimpleNamespace(motion_enabled=True,motion_lock=threading.RLock(),motion_log=None,yield_guard=None,
            motion_snapshot=(10.,np.zeros((2,1)),np.zeros((2,1)),[]))
        native=Mock()
        with patch.object(self.module.rospy,'get_time',return_value=10.6):
            with self.assertRaises(RuntimeError):
                self.module.MotionPlanner(native,owner)(np.zeros((3,1)),None)
        native.assert_not_called()

if __name__=='__main__':unittest.main()
