"""First reference uses three acquisitions, not two serial confirmation windows."""
import copy
import unittest
import numpy as np
from unittest.mock import patch
from test_camera_right_refinement import namespace as n
import test_stable_reference_policy as stable
from reference_test_frames import stamp_reference_frame


class RightBootstrapTests(unittest.TestCase):
    def test_initial_wait_reports_gpu_projection_and_confirmation_separately(self):
        cls=n['CameraRightPath'];node=cls.__new__(cls)
        node.geometry_diagnostics={'frame_seq':42,'right':{'selected_points':6}}
        node.final_path_diagnostics={'reason':'confirming_out_of_band_reference'}
        node.reference_outside_count=2
        for right,expected in (({'boundary_valid':False},'gpu_right_boundary_unavailable'),
            ({'boundary_valid':True,'rejection':'insufficient continuous ground boundary'},'right_ground_projection_rejected'),
            ({'boundary_valid':True,'selected_points':6},'reference_confirmation_pending')):
            node.observation_geometry_diagnostics={'frame_seq':42,'right':right}
            report=node.initial_reference_status('waiting')
            self.assertEqual(report['stage'],expected)
            self.assertEqual(report['confirmation_count'],2)
            self.assertEqual(report['observation_frame_seq'],42)

    def test_recorded_three_metre_support_does_not_need_left_or_whole_route(self):
        points=np.column_stack((np.linspace(2.6068110643,5.6082121858,6),np.full(6,-2.4),np.zeros(6)))
        with patch.dict(n,ground_points=lambda *args:points):
            curve=n['boundary_curve']([[[0,0],[1,1],[2,2]]],None,None,None,None)
        x,left,right=n['boundary_samples'](None,curve,1.,advisory=True)
        self.assertTrue(np.isnan(left).all())
        self.assertGreater(x[0],2.6);self.assertLess(x[-1],5.61)
        np.testing.assert_allclose(right,-2.4)

    def test_three_frames_with_disagreeing_per_point_cache_do_not_authorize(self):
        node,guard,clock,_,_=stable.StableReferenceTests().repair_node(2.4)
        node.cache.clear()
        with stable.StableReferenceTests().repair_context(clock):
            for frame,value in ((1,0.),(2,.3),(3,0.)):
                clock[0]+=.1;stamp_reference_frame(node,frame)
                for i in node.cache_geometry:node.cache.observe(i,value,clock[0],frame)
                result=copy.deepcopy(node.path)
                active,count=node.stable_result(result,clock[0],.1)
                node.publish_result(result,clock[0],'disagree',active,count)
            self.assertFalse(node.cache.committed)
            self.assertEqual(guard.select(clock[0],round(clock[0]*1e9))[2]['source'],'osm')

    def test_real_cache_then_reference_accepts_three_distinct_right_only_frames(self):
        node,guard,clock,_,geometry=stable.StableReferenceTests().repair_node(2.4)
        node.cache.clear()
        # Only the observed 2.6--5.6 m near segment; no left line/full route coverage.
        node.cache_geometry={i:item for i,item in geometry.items() if 2.6<=item['points_odom'][2][0]<=5.6}
        for item in node.cache_geometry.values():
            item['points_odom'][1]=np.nan;item['mode']='right_only'
        with stable.StableReferenceTests().repair_context(clock):
            for frame in (1,1,2,3):
                clock[0]+=.1;node.last_seq=frame;stamp_reference_frame(node,frame)
                for i in node.cache_geometry:node.cache.observe(i,0.,clock[0],frame)
                result=copy.deepcopy(node.path)
                active,count=node.stable_result(result,clock[0],.1)
                node.publish_result(result,clock[0],'bootstrap',active,count)
                state=guard.select(clock[0],round(clock[0]*1e9))[2]
                self.assertEqual(state['source'],'camera' if frame==3 else 'osm')

    def test_projection_diagnostics_distinguish_foldback_from_short_support(self):
        for points,reason in (([[2.,0.,0.],[4.,0.,0.],[3.,0.,0.]],'foldback'),
                              ([[2.,0.,0.],[2.5,0.,0.],[3.,0.,0.]],'support_shorter_than_2m')):
            report={}
            with patch.dict(n,ground_points=lambda *args:np.array(points)), self.assertRaises(ValueError):
                n['boundary_curve']([[[0,0],[1,1],[2,2]]],None,None,None,None,diagnostic=report)
            self.assertEqual(report['segments'][0]['rejection'],reason)


if __name__=='__main__':unittest.main()
