"""Nov9 reference requirements: short geometry and acquisition-driven evidence."""
import copy
import unittest
import numpy as np
from test_camera_right_refinement import namespace as n
import test_stable_reference_policy as stable
from reference_test_frames import stamp_reference_frame


class ReferenceEventTests(unittest.TestCase):
    def test_six_metre_remainder_is_actually_searched_and_admissible(self):
        points=np.column_stack((np.arange(0,12.01,.5),np.zeros(25)))
        geometry={i:dict(points_odom=np.array([points[i],points[i]+[0,2],points[i]+[0,-.75]]),
                        normal_odom=[0.,1.],mode='right_only') for i in range(16,25)}
        limits=dict(wheelbase=.67,max_steer=.35,max_steer_rate=.08,speed=1.5)
        candidate,report=n['stable_segment_reference'](points,points,geometry,([0.,0.],0.),geometry,[6.,0.],
                                                     limits,progress=6.,heading=0.)
        self.assertGreater(report['search_attempts'],0)
        self.assertTrue(report['changed'],report)
        self.assertLessEqual(report['transition_length_m'],6.)
        self.assertTrue(report['shape']['ok'])
        np.testing.assert_array_equal(candidate[:12],points[:12])

    def test_new_json_sequence_cannot_reuse_same_acquisition_for_confirmation(self):
        f=stable.StableReferenceTests();node,_,clock,_,_=f.repair_node()
        with f.repair_context(clock):
            for sequence in range(1,8):
                node.last_seq=sequence
                node.stable_result(copy.deepcopy(node.path),clock[0],.2)
        self.assertEqual(node.reference_outside_count,1)
        self.assertEqual(getattr(node,'reference_version',0),0)

    def test_brief_no_evidence_keeps_confirmation_not_a_fresh_sample(self):
        f=stable.StableReferenceTests();node,_,clock,_,geometry=f.repair_node()
        with f.repair_context(clock):
            for sequence in (1,2):
                stamp_reference_frame(node,sequence)
                node.stable_result(copy.deepcopy(node.path),clock[0],.2)
            node.cache_geometry={};clock[0]+=.2
            node.stable_result(copy.deepcopy(node.path),clock[0],.2)
            self.assertEqual(node.reference_outside_count,2)
            node.cache_geometry=geometry;stamp_reference_frame(node,3)
            node.stable_result(copy.deepcopy(node.path),clock[0],.2)
        self.assertEqual(node.reference_version,1)

    def test_expired_confirmation_cannot_recount_cached_capture(self):
        f=stable.StableReferenceTests();node,_,clock,_,_=f.repair_node()
        with f.repair_context(clock):
            stamp_reference_frame(node,1)
            node.stable_result(copy.deepcopy(node.path),clock[0],.2)
            self.assertEqual(node.reference_outside_count,1)
            clock[0]+=3.1
            node.stable_result(copy.deepcopy(node.path),clock[0],.2)
            self.assertEqual(node.reference_outside_count,0)

    def test_source_change_requires_three_new_captures(self):
        f=stable.StableReferenceTests();node,_,clock,_,geometry=f.repair_node()
        with f.repair_context(clock):
            for sequence in (1,2):
                stamp_reference_frame(node,sequence)
                node.stable_result(copy.deepcopy(node.path),clock[0],.2)
            for item in geometry.values():item['receiver_session_id']='new'
            stamp_reference_frame(node,3)
            node.stable_result(copy.deepcopy(node.path),clock[0],.2)
            self.assertEqual(node.reference_outside_count,1)
            self.assertEqual(getattr(node,'reference_version',0),0)
