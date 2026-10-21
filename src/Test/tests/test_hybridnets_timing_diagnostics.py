"""Pure diagnostic/threshold tests: no ROS master, sockets, or vehicle control."""
import ast
from collections import deque
import json
import math
from pathlib import Path
import socket
import time
from types import SimpleNamespace as NS
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
NOW = 1_000_000_000_000


class Stamp:
    def __init__(self, ns): self.ns = ns
    def to_nsec(self): return self.ns
    def to_sec(self): return self.ns/1e9
    def __sub__(self, other): return Stamp(self.ns-other.ns)


class ROSTime:
    def __new__(cls, seconds=0): return Stamp(round(seconds*1e9))
    @staticmethod
    def now(): return Stamp(NOW)
    @staticmethod
    def from_sec(value): return Stamp(round(value*1e9))


ROS = NS(Time=ROSTime, logwarn_throttle=lambda *args: None)


def load(path, extra=None):
    nodes = ast.parse(path.read_text()).body
    tree = ast.Module(body=[n for n in nodes if isinstance(n, (ast.ClassDef, ast.Assign))], type_ignores=[])
    namespace = dict(json=json, math=math, socket=socket, time=time, deque=deque,
                     np=np, rospy=ROS, Header=lambda **kw: NS(**kw))
    namespace.update(extra or {})
    exec(compile(tree, str(path), 'exec'), namespace)
    return namespace


class Publisher:
    def __init__(self): self.messages = []
    def publish(self, value): self.messages.append(value)


class TimingTests(unittest.TestCase):
    def receiver(self):
        ns = load(ROOT/'hybridnets_receiver/scripts/hybridnets_json_receiver.py')
        cls = ns['HybridNetsJsonReceiver']
        node = cls.__new__(cls)
        for key, value in dict(allowed_sender='', timeout=.8, max_source_age_ms=800,
                max_points=20000, width=1920, height=1080, require_control_fields=True,
                coordinate_space='rotated_image_pixels', min_right_points=3,
                min_left_points=3, min_road_polygon_points=3, min_control_confidence=.65,
                confirm_valid_frames=1, last_receive=None, last_frame_seq=None,
                valid_streak=0, last_sender='', last_reason='', last_result_seq=None,
                receive_times=deque(maxlen=120)).items(): setattr(node,key,value)
        for key in ('json_pub','valid_pub','hz_pub','age_pub','confidence_pub',
                    'left_valid_pub','right_valid_pub','intersection_pub','frame_seq_pub',
                    'header_pub','diag_json_pub'): setattr(node,key,Publisher())
        return node

    def packet(self, **updates):
        packet = dict(version=3, frame_seq=1, capture_timestamp_ns=NOW-100_000_000,
            publish_timestamp_ns=NOW-10_000_000, coordinate_space='rotated_image_pixels',
            image_width=1920, image_height=1080, valid=True, confidence=.9,
            source_age_ms=90., scene_type='normal', road_polygon_valid=True,
            left_boundary_valid=True,right_boundary_valid=True,
            road_polygon_px=[[100,900],[1800,900],[1800,1079],[100,1079]],
            left_boundary_px=[[100,1000],[120,950],[150,900]],
            right_boundary_px=[[1800,1000],[1780,950],[1750,900]])
        packet.update(updates)
        return packet

    def receive(self, node, packet):
        node.sock = NS(recvfrom=lambda size: (json.dumps(packet).encode(),('192.168.10.20',12345)))
        node.receive_once()
        return json.loads(node.diag_json_pub.messages[-1])

    def test_old_sender_still_accepted(self):
        node=self.receiver(); diagnostic=self.receive(node,self.packet())
        self.assertTrue(node.valid_pub.messages[-1])
        self.assertIsNone(diagnostic['result_seq'])
        self.assertEqual(diagnostic['timing_ms'],{})

    def test_stage_timing_and_sequence_gap_preserved(self):
        node=self.receiver()
        for seq in (1,3):
            diagnostic=self.receive(node,self.packet(frame_seq=seq,result_seq=seq,
                send_timestamp_ns=NOW-1_000_000,timing_ms={'model_gpu_ms':65.,'geometry_wall_ms':16.}))
        self.assertEqual(diagnostic['result_seq_gap'],1)
        self.assertEqual(diagnostic['send_to_receive_ms_clock_dependent'],1.)
        forwarded=json.loads(node.json_pub.messages[-1])
        self.assertEqual(forwarded['receiver_diagnostics']['timing_ms']['model_gpu_ms'],65.)

    def test_receiver_age_rejection_is_explicit(self):
        node=self.receiver(); diagnostic=self.receive(node,self.packet(source_age_ms=801.))
        self.assertFalse(node.valid_pub.messages[-1])
        self.assertEqual(diagnostic['receiver_reason'],'receiver_source_age_exceeded')

    def test_malformed_packet_is_logged(self):
        node=self.receiver(); diagnostic=self.receive(node,{'valid':True})
        self.assertEqual(diagnostic['event'],'bad_packet')
        self.assertFalse(node.valid_pub.messages[-1])

    def test_sender_filter_is_diagnosed_without_forwarding_untrusted_json(self):
        node = self.receiver()
        node.allowed_sender = '192.168.10.99'
        diagnostic = self.receive(node, self.packet())
        self.assertEqual(diagnostic['receiver_reason'], 'sender_ip_not_allowed')
        self.assertEqual(diagnostic['sender'], '192.168.10.20:12345')
        self.assertEqual(node.received_datagrams, 1)
        self.assertEqual(node.rejected_sender_datagrams, 1)
        self.assertEqual(node.json_pub.messages, [])
        self.assertIsNone(node.last_receive)

    def test_logger_records_each_diagnostic_event(self):
        ns=load(ROOT/'robot_base_driver/scripts/system_run_logger.py')
        cls=ns['SystemRunLogger']; node=cls.__new__(cls)
        rows=[]; updates=[]
        node.update=lambda key,value: updates.append((key,value))
        node.write_record=lambda row: rows.append(row)
        payload={'frame_seq':7,'timing_ms':{'model_gpu_ms':65.},'receiver_reason':'ok'}
        node.hybridnets_timing_cb(NS(data=json.dumps(payload)))
        self.assertEqual(updates[0],('hybridnets_timing',payload))
        self.assertEqual(rows[0]['record_type'],'hybridnets_timing_event')
        self.assertEqual(rows[0]['data'],payload)
        self.assertEqual(rows[0]['logger_receive_timestamp_ns'],NOW)

    def right_reason(self, **updates):
        ns=load(ROOT/'osm_map_handler/src/osm_camera_path_refinement.py')
        def reached(*args): raise ValueError('geometry_reached')
        ns['boundary_curve']=reached
        cls=ns['CameraRightPath']; node=cls.__new__(cls)
        node.packet=self.packet(**updates)
        node.received=10.;node.timeout=.5;node.receiver_valid=True;node.valid_received=10.
        node.safe=True;node.safe_received=10.;node.control=False
        node.listener=NS(lookupTransform=lambda *args: (np.zeros(3),[0,0,0,1]))
        node.lidar_origin=np.zeros(3);node.resolution=(1920,1080)
        for key in ('matrix','distortion','rotation','origin','ground_z'): setattr(node,key,None)
        try: node.refine(10.)
        except ValueError as error: return str(error)
        self.fail('Expected diagnostic or geometry sentinel')

    def test_distinct_confidence_and_source_age_rejections(self):
        self.assertEqual(self.right_reason(confidence=.649),'low_confidence')
        self.assertEqual(self.right_reason(source_age_ms=300.001),'source_age_exceeded_300ms')

    def test_capture_clock_rejections(self):
        self.assertEqual(self.right_reason(capture_timestamp_ns=NOW-500_001_000),
                         'capture_age_exceeded_500ms_or_clock_skew')
        self.assertEqual(self.right_reason(capture_timestamp_ns=NOW+50_001_000),
                         'capture_timestamp_in_future_clock_skew')

    def test_original_inclusive_thresholds_are_unchanged(self):
        for capture in (NOW-500_000_000,NOW+50_000_000):
            self.assertEqual(self.right_reason(confidence=.65,source_age_ms=300.,
                             capture_timestamp_ns=capture),'geometry_reached')


if __name__ == '__main__': unittest.main()
