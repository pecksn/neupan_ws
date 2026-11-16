"""Offline export must use loaded geometry, past snapshots and explicit unknowns."""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

SCRIPT=Path(__file__).resolve().parents[1]/'scripts/analyze_bridge_reference_log.py'
spec=importlib.util.spec_from_file_location('bridge_evidence_export',SCRIPT)
export=importlib.util.module_from_spec(spec);spec.loader.exec_module(export)


class BridgeEvidenceTests(unittest.TestCase):
    def sample(self,t,y,end_y,version=1,shift=0.):
        return dict(record_type='sample',data=dict(stamp=t,
            gps_guard=dict(transform_x=shift,transform_y=0.,transform_yaw_deg=0.,
                reason='runtime_position_correcting' if shift else 'gps_recovery_wait',runtime_position={}),
            global_odom=dict(x=5.,y=y,yaw_deg=0.,frame_id='map',msg_stamp=t),global_odom_age_sec=0.,
            neupan_runtime=dict(reference_receipt=dict(loaded_endpoint_map=[10.,end_y],
                selected_reference_version=version,selected_source='camera'))))

    def path(self,kind,t,y):
        return dict(record_type=kind,stamp=t,data=dict(frame_id='map',poses=[dict(x=0.,y=y),dict(x=10.,y=y)]))

    def test_loaded_geometry_and_before_window_receipt_not_original_osm_or_future_path(self):
        records=[self.path('global_path_event',0.,5.),self.path('neupan_reference_path_event',.5,0.),
            self.sample(1.,.1,0.),self.sample(2.,.1,0.),
            self.path('neupan_reference_path_event',2.5,1.),self.sample(2.6,.1,0.),self.sample(3.,1.1,1.,2,.1)]
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'input.jsonl';path.write_text(''.join(json.dumps(r)+'\n' for r in records))
            original_hash=hashlib.sha256(path.read_bytes()).hexdigest()
            rows,events,summary=export.analyze(path,1.5,4.)
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),original_hash)
        self.assertEqual(len(rows),3)
        self.assertAlmostEqual(rows[0]['vehicle_to_loaded_distance_m'],.1)
        self.assertIsNone(rows[1]['vehicle_to_loaded_distance_m'])
        self.assertEqual(rows[1]['tracking_unavailable_reason'],'receipt_and_logged_geometry_endpoint_mismatch')
        self.assertAlmostEqual(rows[2]['vehicle_to_loaded_distance_m'],.1)
        self.assertEqual(events[0]['first_matching_receipt']['stamp'],1.)
        self.assertEqual(summary['first_runtime_position_correcting_snapshot'],3.)
        self.assertEqual(summary['source_sha256'],original_hash)
        self.assertIsNone(rows[0]['near_reference_gap_m']);self.assertIsNone(rows[0]['physical_vehicle_road_gap_m'])

    def test_signed_error_and_heading_use_local_polyline_not_global_axes(self):
        result=export.project([[0.,0.],[0.,10.]],[-1.,5.],1.5707963267948966)
        self.assertAlmostEqual(result['signed_left_m'],1.)
        self.assertAlmostEqual(result['heading_error_deg'],0.)


if __name__=='__main__':unittest.main()
