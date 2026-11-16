"""Read-only capture-frame road diagnostics; no ROS master or vehicle."""
import copy
import ast
import json
import math
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import numpy as np
from test_camera_right_refinement import namespace as n
import test_stable_reference_policy as stable


class ClearanceDiagnosticsTests(unittest.TestCase):
    def inputs(self):
        x=np.arange(1.,8.01,.25)
        road=np.column_stack((x,np.where(x<=4.,-4.,-2.)))
        observation=dict(capture_timestamp_ns=10**12,frame_seq=7,receiver_session_id='session',
            camera_time_epoch='epoch',capture_pose_map=[0.,0.,0.],
            capture_map_anchor=([0.,0.],0.),right_base=road)
        packet={key:observation[key] for key in ('capture_timestamp_ns','frame_seq','receiver_session_id','camera_time_epoch')}
        reference=np.column_stack((np.arange(0.,10.,.25),np.zeros(40)))
        return reference,observation,packet

    def call(self,reference,observation,packet,**kw):
        self.assertTrue('capture_reference_gap_summary' in n,'missing same-frame near/far diagnostics')
        return n['capture_reference_gap_summary'](reference,observation,packet,10**12+10**8,**kw)

    def test_near_and_far_are_separate_and_never_relabelled_vehicle_clearance(self):
        reference,observation,packet=self.inputs()
        report=self.call(reference,observation,packet,camera_y=.2)
        self.assertAlmostEqual(report['near']['median_m'],4.2)
        self.assertAlmostEqual(report['far']['median_m'],2.2)
        self.assertEqual(report['near']['forward_range_m'],[2.,4.])
        self.assertGreater(report['near']['point_count'],2)
        self.assertGreater(report['near']['support_length_m'],.5)
        self.assertFalse(report['used_cached_observations'])
        self.assertFalse(report['used_for_control'])
        self.assertEqual(report['distance_reference'],'camera_center')

    def test_capture_pose_not_latest_tf_is_used_for_curved_reference(self):
        reference,observation,packet=self.inputs()
        x=np.arange(1.,8.01,.25);road=np.column_stack((x,-2.+.02*x*x))
        reference=np.column_stack((x,.02*x*x));observation['right_base']=road
        anchor=([42.,-17.],.7)
        observation['capture_pose_map']=[42.,-17.,.7]
        observation['capture_map_anchor']=anchor
        reference=n['planar_transform'](reference,anchor)
        report=self.call(reference,observation,packet)
        self.assertEqual(report['status'],'ok')
        self.assertLess(report['near']['median_m'],2.) # Local curve normals, not global Y.
        self.assertGreater(report['near']['median_m'],1.9)
        self.assertEqual(report['time_basis'],'capture_time_tf')

    def test_stale_other_frame_session_or_missing_capture_tf_is_unknown(self):
        for change,reason in ((dict(capture_timestamp_ns=1),'capture_identity_mismatch'),
                (dict(receiver_session_id='retired'),'capture_identity_mismatch'),
                (dict(camera_time_epoch='old'),'capture_identity_mismatch'),
                (dict(frame_seq=6),'capture_identity_mismatch'),
                (dict(capture_pose_map=None),'capture_tf_unavailable')):
            with self.subTest(change=change):
                reference,observation,packet=self.inputs();observation.update(change)
                result=self.call(reference,observation,packet)
                self.assertEqual(result['status'],'unknown');self.assertEqual(result['reason'],reason)
                self.assertIsNone(result['near']['median_m'])
        reference,observation,packet=self.inputs()
        observation['capture_timestamp_ns']=packet['capture_timestamp_ns']=10**12-10**9
        self.assertEqual(self.call(reference,observation,packet)['reason'],'capture_stale_or_future')

    def test_partial_coverage_and_holes_are_not_extrapolated_or_joined(self):
        reference,observation,packet=self.inputs()
        observation['right_base']=np.array([[5.,-2.],[5.25,-2.],[5.5,-2.],[7.,-2.],[7.25,-2.],[7.5,-2.]])
        result=self.call(reference,observation,packet)
        self.assertIsNone(result['near']['median_m'])
        self.assertEqual(result['near']['reason'],'no_continuous_support_in_window')
        self.assertEqual(result['nearest_continuous']['forward_range_m'],[5.,5.5])
        self.assertAlmostEqual(result['far']['support_length_m'],.5)
        self.assertTrue(result['nearest_continuous']['outside_preferred_window'])

    def test_crossing_route_is_ambiguous_not_a_chosen_future_branch(self):
        reference,observation,packet=self.inputs()
        reference=np.array([[0.,0.],[8.,0.],[0.,1.],[8.,1.]])
        result=self.call(reference,observation,packet)
        self.assertEqual(result['reason'],'reference_branch_ambiguous_or_unsupported')
        self.assertIsNone(result['near']['median_m'])

    def test_diagnostic_is_read_only(self):
        reference,observation,packet=self.inputs();before=copy.deepcopy((reference,observation,packet))
        self.call(reference,observation,packet)
        np.testing.assert_array_equal(reference,before[0]);np.testing.assert_array_equal(observation.pop('right_base'),before[1].pop('right_base'))
        self.assertEqual(observation,before[1]);self.assertEqual(packet,before[2])

    def test_previous_localization_generation_cannot_supply_current_road_diagnostic(self):
        reference,observation,packet=self.inputs();observation['localization_frame_key']=('bridge',0)
        result=self.call(reference,observation,packet,localization_frame_key=('bridge',1))
        self.assertEqual(result['status'],'unknown')
        self.assertEqual(result['reason'],'localization_generation_mismatch')

    def test_publication_keeps_null_car_side_gap_and_adds_reason_without_control_change(self):
        node,guard,clock,points,_,_=stable.StableReferenceTests().adopted_reference()
        reference,observation,packet=self.inputs()
        node.raw_road_observation=observation;node.packet=dict(node.packet or {},**packet)
        node.current_vehicle_position=np.array([0.,0.]);node.cache_geometry={};node.final_path_diagnostics={}
        with stable.StableReferenceTests().repair_context(clock):
            node.publish_result(copy.deepcopy(node.path),clock[0],'occluded',False,0)
        report=guard.status['inference_diagnostics']
        self.assertIn('capture_reference_clearance',report)
        self.assertIsNone(report['current_camera_clearance_estimate_m'])
        self.assertEqual(report['current_camera_clearance_missing_reason'],'no_cached_boundary_geometry')
        self.assertEqual(node.reference_version,1)

    def test_bad_or_changed_diagnostic_cannot_change_path_or_motion_authority(self):
        node,guard,clock,points,_,_=stable.StableReferenceTests().adopted_reference()
        _,observation,packet=self.inputs()
        node.raw_road_observation=observation;node.packet=dict(node.packet or {},**packet)
        node.current_vehicle_position=np.array([0.,0.]);node.final_path_diagnostics={}
        state=copy.deepcopy((node.reference_version,node.cache.committed,node.cache.pending,node.reference_accepted_stamp))
        with stable.StableReferenceTests().repair_context(clock):
            node.publish_result(copy.deepcopy(node.path),clock[0],'hold',False,0)
            before=copy.deepcopy(guard.status)
            with patch.dict(n,capture_reference_gap_summary=lambda *a,**kw:(_ for _ in ()).throw(ValueError('diagnostic only'))):
                node.publish_result(copy.deepcopy(node.path),clock[0],'hold',False,0)
            after=copy.deepcopy(guard.status)
        self.assertEqual(after['inference_diagnostics']['capture_reference_clearance']['status'],'unknown')
        for value in (before,after):
            value['inference_diagnostics'].pop('capture_reference_clearance')
            value.pop('processing_health',None)
        self.assertEqual(before,after)
        self.assertEqual((node.reference_version,node.cache.committed,node.cache.pending,node.reference_accepted_stamp),state)

    def test_existing_logger_retains_new_fields_without_a_new_subscription(self):
        from pathlib import Path
        source=Path(__file__).resolve().parents[3]/'src/robot_base_driver/scripts/system_run_logger.py'
        cls=next(v for v in ast.parse(source.read_text()).body if isinstance(v,ast.ClassDef) and v.name=='SystemRunLogger')
        method=copy.deepcopy(next(v for v in cls.body if isinstance(v,ast.FunctionDef) and v.name=='parse_status'))
        method.decorator_list=[];namespace=dict(json=json)
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),namespace)
        reference,observation,packet=self.inputs();diagnostic=self.call(reference,observation,packet)
        data=dict(inference_diagnostics=dict(capture_reference_clearance=diagnostic))
        actual=namespace['parse_status'](NS(data=json.dumps(data)))
        self.assertEqual(actual,json.loads(json.dumps(data)))


if __name__=='__main__':unittest.main()
