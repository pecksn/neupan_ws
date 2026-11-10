"""Separate map error from actual vehicle entry; real geometry functions."""
import unittest
import json
from types import SimpleNamespace as NS
import numpy as np
from test_camera_right_refinement import namespace as n


class RoadAssociationTests(unittest.TestCase):
    def test_new_camera_source_cancels_pending_confirmation_not_accepted_map_reference(self):
        import test_stable_reference_policy as stable
        node,_,clock,points,_=stable.StableReferenceTests().repair_node()
        accepted=points-[0.,.6];node.frozen_reference_map=accepted.copy();node.reference_version=1
        node.reference_evidence_event={'frames':['old1','old2']};node.reference_outside_count=2
        node.json_cb(NS(data=json.dumps(dict(frame_seq=1,receiver_session_id='new',camera_time_epoch='new'))))
        self.assertIsNone(node.reference_evidence_event)
        self.assertEqual(node.reference_outside_count,0)
        np.testing.assert_array_equal(node.frozen_reference_map,accepted)
        self.assertEqual(node.reference_version,1)
    def test_parallel_road_ambiguity_does_not_select_another_branch(self):
        points=np.array([[0.,4.59],[10.,4.59]])
        selected=[[0,4.59,10,4.59,3,0]]
        self.assertFalse(n['competing_road_branch'](points,selected,[0.,0.],0.,(2,8),6.))
        self.assertTrue(n['competing_road_branch'](points,selected+[[0,-2,10,-2,3,0]],[0.,0.],0.,(2,8),6.))

    def test_later_right_edge_keeps_full_measured_support(self):
        right=np.array([[4.70,-.9],[6.,-.9],[7.60,-.9]])
        x,_,_=n['boundary_samples'](None,right,1.,advisory=True)
        self.assertGreater(x[-1]-x[0],2.)

    def test_bad_osm_does_not_require_large_vehicle_translation(self):
        points=np.column_stack((np.arange(0.,7.251,.25),np.full(30,4.59)))
        geometry={i:dict(points_odom=np.array([[points[i,0],.1],[np.nan,np.nan],[points[i,0],-.9]]),
            normal_odom=[0.,1.],mode='right_only') for i in range(8,30)}
        candidate,report=n['stable_segment_reference'](points,points,geometry,([0.,0.],0.),geometry,
            [0.,0.],dict(wheelbase=.67,max_steer=.35,max_steer_rate=.08,speed=1.5),
            max_shift=6.,heading=0.)
        self.assertTrue(report['changed'],report)
        self.assertTrue(report['shape']['ok'])
        np.testing.assert_allclose(candidate[report['reference_connection']['start_index']],[0.,0.])
        self.assertLess(abs(candidate[-1,1]),.3)
        self.assertGreater(abs(candidate[-1,1]-points[-1,1]),4.)
        from test_outdoor_start_connection import AdapterTests
        adapter,apply=AdapterTests().node()
        adapter.robot_state=np.zeros((3,1));adapter.arrival_path_key=('map',123)
        adapter.camera_guard_selected_state=dict(source='camera',reference_role='advisory',reference_version=1,
            reference_connection=report['reference_connection'],matched_path_digest='bound')
        adapter.pending_route_update=('path',[np.array([[x],[y],[0.],[1.]]) for x,y in candidate],('map',123))
        apply(adapter)
        loaded=adapter.neupan_planner.set_initial_path.call_args.args[0]
        np.testing.assert_allclose(loaded[0][:2,0],[0.,0.])
        np.testing.assert_allclose(loaded[-1][:2,0],candidate[-1])
        self.assertEqual(adapter.reference_receipt['selected_reference_version'],1)

    def test_normal_bend_uses_local_normals_and_in_band_reference_is_held(self):
        x=np.arange(0.,40.01,.5);points=np.column_stack((x,x*x/200.))
        tangent=np.vstack((np.diff(points,axis=0),points[-1]-points[-2]))
        normals=np.column_stack((-tangent[:,1],tangent[:,0]))/np.linalg.norm(tangent,axis=1)[:,None]
        geometry={i:dict(points_odom=np.array([points[i],points[i]+normals[i]*4,points[i]-normals[i]*3.6]),
                         normal_odom=normals[i].tolist()) for i in range(4,17)}
        candidate,report=n['stable_segment_reference'](points,points,geometry,([0.,0.],0.),geometry,
            [0.,0.],dict(wheelbase=.67,max_steer=.35,max_steer_rate=.08,speed=1.5),max_shift=6.,heading=0.)
        self.assertTrue(report['changed'],report)
        self.assertTrue(report['shape']['ok'])
        self.assertGreater(abs(candidate[-1,0]-points[-1,0]),.01)
        held,report=n['stable_segment_reference'](points,candidate,geometry,([0.,0.],0.),geometry,
            [0.,0.],max_shift=6.,heading=0.)
        # An entry still being traversed is separately retained by stable_result;
        # once the forward reference is in band it must not pursue exact 2m.
        in_band=points-normals*1.6
        held,report=n['stable_segment_reference'](points,in_band,geometry,([0.,0.],0.),geometry,
            [0.,0.],max_shift=6.,heading=0.)
        self.assertFalse(report['changed']);np.testing.assert_array_equal(held,in_band)
