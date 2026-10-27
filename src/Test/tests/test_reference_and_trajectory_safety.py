"""Reference/command safety regressions, no ROS master or vehicle outputs."""
import ast
import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src/neupan_ros/src'))
from neupan_safety import validate_solution, trajectory_clearance, reference_offsets


def reference_class():
    from neupan.blocks.initial_path import InitialPath
    return InitialPath


class ReferenceSafetyTests(unittest.TestCase):
    def make_path(self, points, gear=1, kinematics='acker'):
        cls=reference_class()
        robot=NS(kinematics=kinematics,L=.67,max_speed=np.array([1.5,.35]))
        path=cls(50,.1,.8,robot)
        path.set_initial_path([np.array([x,y,yaw,gear]).reshape(4,1) for x,y,yaw in points])
        return path

    def test_irregular_sampling_duplicates_and_end_hold(self):
        for offset in (0.,500.,1.e6):
            path=self.make_path([(offset+x,0.,0.) for x in (0.,0.,.01,.5,2.,2.,6.)])
            before=np.hstack(path.cur_curve).copy()
            _,_,ref,speed=path.generate_nom_ref_state(np.zeros((3,1)),np.zeros((2,50)),.8)
            np.testing.assert_allclose(ref[0],offset+np.arange(51)*.08,atol=1.e-9)
            np.testing.assert_array_equal(np.hstack(path.cur_curve),before)
            self.assertTrue(np.all(speed==.8))
        path=self.make_path([(0,0,0),(.12,0,0)])
        _,_,ref,speed=path.generate_nom_ref_state(np.zeros((3,1)),np.zeros((2,50)),.8)
        self.assertAlmostEqual(ref[0,-1],.12)
        self.assertEqual(speed[-1],0)

    def test_reverse_diff_and_smooth_curve(self):
        path=self.make_path([(x,0,math.pi) for x in np.arange(0,8,.2)],gear=-1)
        _,_,ref,speed=path.generate_nom_ref_state(np.array([[0.],[0.],[math.pi]]),np.zeros((2,50)),.8)
        self.assertTrue(np.all(speed<0));self.assertGreater(ref[0,-1],3.)
        path=self.make_path([(x,0,0) for x in (0,.1,8)],kinematics='diff')
        path.generate_nom_ref_state(np.zeros((3,1)),np.zeros((2,50)),.8)
        a=np.arange(0,1,.02)
        path=self.make_path([(5*math.sin(t),5*(1-math.cos(t)),t) for t in a])
        path.generate_nom_ref_state(np.zeros((3,1)),np.zeros((2,50)),.8)

    def test_endpoint_roundoff_and_missing_intersection(self):
        path=self.make_path([(0,0,0),(10,0,0)])
        for epsilon in (-1.e-15,0.,1.e-15):
            result=path.range_cir_seg(np.array([.42,0]),.08,[np.array([0.,0]),np.array([.5+epsilon,0])])
            self.assertIsNotNone(result)
        with self.assertRaisesRegex(ValueError,'distant endpoint'):
            path.find_interaction_point(np.array([[0.],[10.],[0.]]),0,.08)

    def test_discontinuous_yaw_rejected_before_optimizer(self):
        path=self.make_path([(0,0,0),(.08,0,math.pi),(8,0,math.pi)])
        with self.assertRaisesRegex(ValueError,'heading'):
            path.generate_nom_ref_state(np.zeros((3,1)),np.zeros((2,50)),.8)

    def test_two_recorded_failure_inputs_have_no_reference_jump(self):
        import json
        for filename in sorted((ROOT/'tracker_logs').glob('neupan_failure_20261026_*.npz')):
            p=np.load(filename)
            metadata=json.loads(filename.with_suffix('.json').read_text())
            route=p['initial_path']; path=self.make_path(route[:3].T)
            path.curve_index=metadata['path_progress']['curve_index']
            path.point_index=metadata['path_progress']['point_index']
            _,_,ref,_=path.generate_nom_ref_state(p['solver_robot_state'],p['cur_vel_array'],.8)
            self.assertLessEqual(float(np.linalg.norm(np.diff(ref[:2],axis=1),axis=0).max()),.080001)


class TrajectorySafetyTests(unittest.TestCase):
    def test_reference_metrics_use_polyline_not_sparse_vertices(self):
        points=np.array([[.25,.75],[0.,0.],[0.,0.]])
        self.assertEqual(reference_offsets(points,[[0,0],[1,0]])['reference_modified_points'],0)
        points[1]=.5
        self.assertAlmostEqual(reference_offsets(points,[[0,0],[1,0]])['reference_max_offset_m'],.5)
    def setUp(self):
        self.body=np.array([[-.14,.81,.81,-.14],[-.425,-.425,.425,.425]])
        self.robot=NS(max_speed=np.array([1.5,.35]),max_acce=np.array([.8,.08]),kinematics='acker',L=.67)

    def test_full_scan_and_between_pose_collision(self):
        states=np.array([[0.,2.],[0.,0.],[0.,0.]])
        with self.assertRaisesRegex(ValueError,'unsafe'):
            trajectory_clearance(states,np.array([[1.],[0.]]),self.body)
        report=trajectory_clearance(states,np.array([[1.],[1.]]),self.body)
        self.assertGreater(report['min_clearance_m'],.05)
        self.assertGreater(report['checked_poses'],20)
        with self.assertRaises(ValueError):
            trajectory_clearance(states,np.array([[math.nan],[1.]]),self.body)

    def test_action_limits_and_nonfinite(self):
        for action in (np.array([[1.6],[0.]]),np.array([[0.],[-.615]]),np.array([[math.nan],[0.]])):
            with self.assertRaises(ValueError): validate_solution(action,{},self.robot,.1)
        states=[np.array([[x],[0.],[0.]]) for x in (0,.08,.16)]
        info=dict(opt_state_list=states,vel_tensor=np.array([[.8,.8],[0.,0.]]))
        validate_solution(np.array([[.8],[0.]]),info,self.robot,.1)
        with self.assertRaisesRegex(ValueError,'differs'):
            validate_solution(np.array([[1.4],[0.]]),info,self.robot,.1)
        states[-1][0,0]=220.
        with self.assertRaisesRegex(ValueError,'residual'):
            validate_solution(np.array([[.8],[0.]]),info,self.robot,.1)

    def test_failure_revokes_action_even_when_disk_write_fails(self):
        import neupan_diagnostics as d
        node=NS(vel_pub=Mock())
        try: raise ValueError('solver failure')
        except ValueError:
            exc=sys.exc_info()
        with patch('rospy.get_param',return_value='/tmp'), patch.object(d,'save_failure',side_effect=OSError('disk full')) as saving:
            d.report_failure(node,exc)
        saving.assert_called_once()
        self.assertEqual(node.vel_pub.publish.call_count,1)
        self.assertEqual(node.vel_pub.publish.call_args[0][0].linear.x,0.)


if __name__=='__main__': unittest.main()
