"""Pure diagnostic/threshold tests: no ROS master, sockets, or vehicle control."""
import ast
from collections import deque
import json
import math
from pathlib import Path
import socket
import time
import threading
from types import SimpleNamespace as NS
import unittest
import numpy as np
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'hybridnets_receiver/scripts'))
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'hybridnets_receiver/src'))
from hybridnets_binding import ReceiverBinding

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
    def test_invalid_observations_keep_debug_and_metrics_but_not_control(self):
        node = self.bound_receiver()
        for seq in range(1, 6):
            self.receive(node, self.bound_packet(node, seq, valid=False, confidence=.49))
        self.assertEqual(len(node.observations_pub.messages), 5)
        self.assertEqual(len(node.hz_pub.messages), 5)
        self.assertEqual(len(node.confidence_pub.messages), 5)
        self.assertEqual(node.json_pub.messages, [])
        for raw in node.observations_pub.messages:
            p = json.loads(raw)
            self.assertFalse(p['receiver_diagnostics']['receiver_valid'])
            self.assertFalse(p['receiver_diagnostics']['submitted_to_path'])
        # Exercise the actual image/result pairing used by the overlay.
        cls = load(ROOT/'hybridnets_receiver/scripts/hybridnets_overlay.py')['HybridNetsOverlay']
        overlay = cls.__new__(cls)
        overlay.lock = threading.Lock(); overlay.video_delay = .1
        overlay.sync_tolerance = .25; overlay.max_age = .8; overlay.buffer_sec = 2.
        overlay.pending = deque(); overlay.images = deque()
        pairs = []; overlay.draw_and_publish = lambda *args: pairs.append(args)
        overlay.image_callback(NS(header=NS(stamp=Stamp(NOW))))
        for raw in node.observations_pub.messages:
            overlay.result_callback(NS(data=raw))
        self.assertEqual(len(pairs), 5)
        before = len(node.observations_pub.messages)
        self.receive(node, self.bound_packet(node, 6, producer_session_id='old'))
        self.assertEqual(len(node.observations_pub.messages), before)

    def test_candidate_observations_are_separate_from_control(self):
        node = self.bound_receiver()
        for seq in (1, 2, 3):
            self.receive(node, self.bound_packet(node, seq))
            self.assertEqual(len(node.observations_pub.messages), seq)
            self.assertEqual(len(node.json_pub.messages), int(seq == 3))

    def test_path_full_history_accepts_confirmed_lease_not_replays_or_candidates(self):
        node = self.bound_receiver()
        for seq in (1, 2, 3): self.receive(node, self.bound_packet(node, seq))
        p = json.loads(node.json_pub.messages[-1])
        cls = load(ROOT/'osm_map_handler/src/osm_camera_path_refinement.py')['CameraRightPath']
        path = cls.__new__(cls); path.lock = threading.RLock(); path.timeout = .8
        path.retired_receiver_sessions = set('old-%d' % i for i in range(1024))
        path.last_receiver_session = 'previous'; path.last_seq = 100
        path.confirmations = 2; path.cache = NS(pending={1: 'pending'}, committed={2:'map'})
        path.packet = {'original': True}
        candidate = json.loads(node.observations_pub.messages[0])
        path.json_cb(NS(data=json.dumps(candidate)))
        self.assertEqual(path.cache.pending, {1:'pending'})
        self.assertEqual(path.last_receiver_session, 'previous')
        path.json_cb(NS(data=json.dumps(p)))
        self.assertEqual(path.packet, p)
        self.assertLessEqual(len(path.retired_receiver_sessions), 1024)
        self.assertEqual(path.cache.committed, {2:'map'})
        # An old confirmed frame must not regain ownership or refresh receipt.
        before = path.received
        old = json.loads(json.dumps(p)); old['receiver_diagnostics']['receive_timestamp_ns'] -= 1
        old['frame_seq'] += 1; old['receiver_diagnostics']['frame_seq'] += 1
        path.json_cb(NS(data=json.dumps(old)))
        self.assertEqual(path.packet, p); self.assertEqual(path.received, before)
        # A newly started receiver may authorize a fresh identity without history veto.
        new = json.loads(json.dumps(p)); new['receiver_instance_id'] = 'new-receiver'
        identity = json.loads(new['receiver_session_id']); identity[-1] = 'new-receiver'
        new['receiver_session_id'] = json.dumps(identity, separators=(',', ':'))
        d = new['receiver_diagnostics']; d['receiver_session_id'] = new['receiver_session_id']
        d['binding']['receiver_instance_id'] = 'new-receiver'
        d['receive_timestamp_ns'] += 1
        path.json_cb(NS(data=json.dumps(new)))
        self.assertEqual(path.packet, new)

    def bound_receiver(self):
        node=self.receiver();node.confirm_valid_frames=3
        node.binding_client=ReceiverBinding()
        node.binding_client.bound=dict(identity=('producer-b','clock-b'),ticket='b'*64,deadline=time.monotonic()+5.)
        return node

    def bound_packet(self,node,seq,**updates):
        values=dict(frame_seq=seq,result_seq=seq,producer_session_id='producer-b',camera_time_epoch='clock-b',
                    binding_ticket=node.binding_client.bound['ticket'],receiver_instance_id=node.binding_client.instance_id)
        values.update(updates)
        return self.packet(**values)

    def test_unseen_old_A_cannot_permanently_retire_healthy_B(self):
        node=self.bound_receiver()
        for seq in (1,2,3):self.receive(node,self.bound_packet(node,seq))
        before=(node.active_session,set(node.retired_sessions),node.valid_streak,node.last_active_mono,
                len(node.json_pub.messages),len(node.valid_pub.messages))
        for seq in (98,99,100):
            d=self.receive(node,self.bound_packet(node,seq,producer_session_id='unseen-old-a',
                capture_timestamp_ns=NOW-200000000,source_age_ms=100.))
            self.assertEqual(d['receiver_reason'],'binding_identity_mismatch')
            self.assertEqual(before,(node.active_session,set(node.retired_sessions),node.valid_streak,
                node.last_active_mono,len(node.json_pub.messages),len(node.valid_pub.messages)))
        d=self.receive(node,self.bound_packet(node,4))
        self.assertTrue(d['receiver_valid']);self.assertNotEqual(d['receiver_reason'],'retired_session')

    def test_binding_candidates_do_not_reach_path_until_three(self):
        node=self.bound_receiver()
        for seq in (1,2):
            d=self.receive(node,self.bound_packet(node,seq))
            self.assertEqual(node.json_pub.messages,[])
            self.assertFalse(d['submitted_to_path'])
            self.assertEqual(d['stream_streak'],seq)
        d=self.receive(node,self.bound_packet(node,3))
        self.assertTrue(d['submitted_to_path']);self.assertEqual(len(node.json_pub.messages),1)
        node.binding_client.bound['ticket']='c'*64
        d=self.receive(node,self.bound_packet(node,1))
        self.assertFalse(d['submitted_to_path']);self.assertEqual(len(node.json_pub.messages),1)
        self.assertEqual(node.valid_streak,1)

    def test_no_road_does_not_prevent_identity_or_link_diagnosis(self):
        node=self.bound_receiver()
        for seq in (1,2,3):
            d=self.receive(node,self.bound_packet(node,seq,valid=False))
        self.assertTrue(d['binding']['confirmed'])
        self.assertEqual(d['stream_streak'],3)
        self.assertEqual(node.valid_streak,0)
        self.assertEqual(node.json_pub.messages,[])
        self.assertFalse(d['receiver_valid'])

    def test_expired_binding_and_historical_receiver_ticket_cannot_nominate_identity(self):
        node=self.bound_receiver()
        p=self.bound_packet(node,1)
        p['receiver_instance_id']='historical-receiver'
        self.assertEqual(self.receive(node,p)['receiver_reason'],'binding_receiver_mismatch')
        node.binding_client.bound['deadline']=time.monotonic()-1.
        d=self.receive(node,self.bound_packet(node,2))
        self.assertEqual(d['receiver_reason'],'binding_expired')
        self.assertIsNone(node.active_session)
        self.assertEqual(node.json_pub.messages,[])

    def test_diagnostic_retired_history_cannot_veto_authorized_binding(self):
        node=self.bound_receiver()
        node.retired_sessions={('historical',str(i),None) for i in range(1024)}
        for seq in (1,2,3):
            d=self.receive(node,self.bound_packet(node,seq))
        self.assertTrue(d['receiver_valid'])
        self.assertLessEqual(len(node.retired_sessions),1024)

    def receiver(self):
        ns = load(ROOT/'hybridnets_receiver/scripts/hybridnets_json_receiver.py')
        cls = ns['HybridNetsJsonReceiver']
        node = cls.__new__(cls)
        ns['DiagnosticStatus'] = type('DiagnosticStatus', (), {'ERROR':2, 'OK':0})
        ns['KeyValue'] = lambda key,value: NS(key=key,value=value)
        ns['DiagnosticArray'] = lambda **kw: NS(**kw)
        for key, value in dict(allowed_sender='', timeout=.8, max_source_age_ms=800,
                max_points=20000, width=1920, height=1080, require_control_fields=True,
                coordinate_space='rotated_image_pixels', min_right_points=3,
                min_left_points=3, min_road_polygon_points=3, min_control_confidence=.60,
                confirm_valid_frames=1, last_receive=None, last_frame_seq=None,
                valid_streak=0, last_sender='', last_reason='', last_result_seq=None,
                receive_times=deque(maxlen=120),last_timeout_diagnostic=None).items(): setattr(node,key,value)
        node.state_lock = threading.RLock()
        node.active_session = None
        node.retired_sessions = set()
        node.identity_seen = node.producer_identity_seen = False
        node.last_active_mono = node.last_capture_ns = None
        for key in ('observations_pub','json_pub','valid_pub','hz_pub','age_pub','confidence_pub',
                    'left_valid_pub','right_valid_pub','intersection_pub','frame_seq_pub',
                    'header_pub','diag_json_pub','diag_pub'): setattr(node,key,Publisher())
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
        if 'capture_timestamp_ns' not in updates:
            seq = packet.get('frame_seq')
            packet['capture_timestamp_ns'] += seq if isinstance(seq, int) and not isinstance(seq, bool) else 0
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

    def test_valid_right_edge_without_polygon_can_feed_advisory_consumer(self):
        node=self.receiver()
        self.receive(node,self.packet(road_polygon_valid=False,road_polygon_px=[],
            left_boundary_valid=False,left_boundary_px=[]))
        self.assertTrue(node.valid_pub.messages[-1])

    def test_missing_polygon_does_not_invent_valid_right_edge(self):
        node=self.receiver()
        self.receive(node,self.packet(road_polygon_valid=False,road_polygon_px=[],
            left_boundary_valid=False,left_boundary_px=[],right_boundary_valid=False,right_boundary_px=[]))
        self.assertFalse(node.valid_pub.messages[-1])

    def test_stage_timing_and_sequence_gap_preserved(self):
        node=self.receiver()
        for seq in (1,3):
            diagnostic=self.receive(node,self.packet(frame_seq=seq,result_seq=seq,
                send_timestamp_ns=NOW-1_000_000,timing_ms={'model_gpu_ms':65.,'geometry_wall_ms':16.}))
        self.assertEqual(diagnostic['result_seq_gap'],1)
        self.assertEqual(diagnostic['send_to_receive_ms_clock_dependent'],1.)
        forwarded=json.loads(node.json_pub.messages[-1])
        self.assertEqual(forwarded['receiver_diagnostics']['timing_ms']['model_gpu_ms'],65.)

    def test_new_epoch_resets_high_sequence_and_requires_three_fresh_frames(self):
        node = self.receiver(); node.confirm_valid_frames = 3
        for seq in (98,99,100):
            self.receive(node,self.packet(frame_seq=seq,result_seq=seq,camera_time_epoch='old'))
        self.assertTrue(node.valid_pub.messages[-1])
        for seq in (1,2,3):
            d = self.receive(node,self.packet(frame_seq=seq,result_seq=seq,camera_time_epoch='new'))
            self.assertEqual(node.valid_pub.messages[-1], seq == 3)
            self.assertEqual(d['valid_streak'], seq)
            self.assertEqual(d['session_changed'], seq == 1)
        self.assertEqual(node.last_frame_seq,3)
        self.assertEqual(node.last_result_seq,3)

    def test_two_old_frames_plus_one_new_epoch_do_not_confirm(self):
        node = self.receiver(); node.confirm_valid_frames = 3
        for seq in (98,99):
            self.receive(node,self.packet(frame_seq=seq,camera_time_epoch='old'))
        d = self.receive(node,self.packet(frame_seq=101,camera_time_epoch='new'))
        self.assertEqual(d['receiver_reason'],'confirming_valid_frames')
        self.assertEqual(node.valid_streak,1)
        self.assertFalse(node.valid_pub.messages[-1])

    def test_retired_epoch_does_not_refresh_or_downgrade_active_state(self):
        node = self.receiver(); node.confirm_valid_frames = 3
        self.receive(node,self.packet(frame_seq=99,camera_time_epoch='old'))
        for seq in (1,2,3):self.receive(node,self.packet(frame_seq=seq,camera_time_epoch='new'))
        before = (node.active_session,node.last_frame_seq,node.last_active_mono,node.valid_streak,len(node.json_pub.messages),len(node.valid_pub.messages))
        d = self.receive(node,self.packet(frame_seq=100,camera_time_epoch='old'))
        self.assertEqual(d['receiver_reason'],'retired_session')
        self.assertEqual(before,(node.active_session,node.last_frame_seq,node.last_active_mono,node.valid_streak,len(node.json_pub.messages),len(node.valid_pub.messages)))

    def test_duplicate_and_old_frames_never_refresh_or_rollback_watermarks(self):
        node = self.receiver()
        self.receive(node,self.packet(frame_seq=100,result_seq=20,camera_time_epoch='a'))
        before=(node.last_active_mono,node.last_capture_ns,len(node.json_pub.messages))
        for seq in (100,99):
            d=self.receive(node,self.packet(frame_seq=seq,result_seq=19,camera_time_epoch='a'))
            self.assertFalse(d['forwarded'])
            self.assertEqual(node.last_frame_seq,100)
            self.assertEqual(node.last_result_seq,20)
        self.assertEqual(before,(node.last_active_mono,node.last_capture_ns,len(node.json_pub.messages)))

    def test_stale_new_epoch_cannot_replace_active_session_or_poison_sequence(self):
        node=self.receiver()
        self.receive(node,self.packet(frame_seq=10,camera_time_epoch='a'))
        before=node.active_session
        d=self.receive(node,self.packet(frame_seq=999,camera_time_epoch='b',source_age_ms=801.))
        self.assertFalse(d['forwarded']);self.assertEqual(node.active_session,before)
        self.assertEqual(node.last_frame_seq,10)
        self.receive(node,self.packet(frame_seq=11,camera_time_epoch='a'))
        self.assertTrue(node.valid_pub.messages[-1])

    def test_producer_restart_with_same_epoch_resets_confirmation(self):
        node=self.receiver();node.confirm_valid_frames=3
        for seq in (98,99,100):self.receive(node,self.packet(frame_seq=seq,camera_time_epoch='a',producer_session_id='worker1'))
        d=self.receive(node,self.packet(frame_seq=1,camera_time_epoch='a',producer_session_id='worker2'))
        self.assertTrue(d['session_changed']);self.assertEqual(node.valid_streak,1)
        self.assertFalse(node.valid_pub.messages[-1])
        d=self.receive(node,self.packet(frame_seq=101,camera_time_epoch='a',producer_session_id='worker1'))
        self.assertEqual(d['receiver_reason'],'retired_session')

    def test_current_gpu_udp_endpoint_restart_is_compatible(self):
        node=self.receiver()
        self.receive(node,self.packet(frame_seq=100,camera_time_epoch='a'))
        packet=self.packet(frame_seq=1,camera_time_epoch='a')
        node.sock=NS(recvfrom=lambda size:(json.dumps(packet).encode(),('192.168.10.20',54321)))
        node.receive_once()
        d=json.loads(node.diag_json_pub.messages[-1])
        self.assertTrue(d['session_changed']);self.assertEqual(d['session_basis'],'udp_endpoint')
        self.assertEqual(node.last_frame_seq,1)

    def test_producer_identity_survives_port_change_and_is_logged(self):
        node = self.receiver(); node.confirm_valid_frames = 3
        self.receive(node, self.packet(frame_seq=1, producer_session_id='process-a', camera_time_epoch='clock-a'))
        packet = self.packet(frame_seq=2, producer_session_id='process-a', camera_time_epoch='clock-a')
        node.sock = NS(recvfrom=lambda size: (json.dumps(packet).encode(), ('192.168.10.20', 54321)))
        node.receive_once()
        d = json.loads(node.diag_json_pub.messages[-1])
        self.assertFalse(d['session_changed'])
        self.assertEqual(node.valid_streak, 2)
        self.assertEqual(d['producer_session_id'], 'process-a')
        self.assertEqual(d['session_basis'], 'producer_session')
        self.assertEqual(json.loads(node.json_pub.messages[-1])['receiver_session_id'], d['receiver_session_id'])
        self.receive(node, self.packet(frame_seq=3, producer_session_id='process-a', camera_time_epoch='clock-a'))
        self.assertTrue(node.valid_pub.messages[-1])

    def test_late_producer_packet_does_not_interrupt_new_confirmation(self):
        node = self.receiver(); node.confirm_valid_frames = 3
        for seq in (1,2,3):
            self.receive(node, self.packet(frame_seq=seq, producer_session_id='old', camera_time_epoch='same'))
        for seq in (1,2,3):
            d = self.receive(node, self.packet(frame_seq=seq, producer_session_id='new', camera_time_epoch='same'))
            self.assertEqual(node.valid_streak, seq)
            self.assertEqual(d['receiver_valid'], seq == 3)
            before = (node.valid_streak, node.last_active_mono, len(node.valid_pub.messages))
            d = self.receive(node, self.packet(frame_seq=100+seq, producer_session_id='old', camera_time_epoch='same'))
            self.assertEqual(d['receiver_reason'], 'retired_session')
            self.assertEqual(before, (node.valid_streak, node.last_active_mono, len(node.valid_pub.messages)))

    def test_huge_capture_age_cannot_fall_back_to_small_source_age(self):
        node = self.receiver()
        self.receive(node, self.packet(frame_seq=1, producer_session_id='current', camera_time_epoch='clock'))
        before = node.active_session
        for age_ms in (60001, 120000):
            d = self.receive(node, self.packet(frame_seq=2, producer_session_id='late', camera_time_epoch='old',
                capture_timestamp_ns=NOW-age_ms*1000000, source_age_ms=90.))
            self.assertFalse(d['forwarded'])
            self.assertEqual(d['receiver_reason'], 'receiver_source_age_exceeded')
            self.assertEqual(node.active_session, before)
        self.receive(node, self.packet(frame_seq=2, producer_session_id='current', camera_time_epoch='clock'))
        self.assertTrue(node.valid_pub.messages[-1])

    def test_duplicate_cannot_interrupt_confirmation_or_fresh_valid_state(self):
        node = self.receiver(); node.confirm_valid_frames = 3
        for seq in (1,2,3):
            self.receive(node, self.packet(frame_seq=seq, producer_session_id='p', camera_time_epoch='e'))
            before=(node.valid_streak,node.last_active_mono,node.last_reason,len(node.valid_pub.messages))
            d=self.receive(node,self.packet(frame_seq=seq,producer_session_id='p',camera_time_epoch='e'))
            self.assertFalse(d['forwarded'])
            self.assertEqual(before,(node.valid_streak,node.last_active_mono,node.last_reason,len(node.valid_pub.messages)))
        self.assertTrue(node.valid_pub.messages[-1])
        self.receive(node,self.packet(frame_seq=4,producer_session_id='p',camera_time_epoch='e',valid=False))
        self.assertEqual(node.valid_streak,0)
        self.assertFalse(node.valid_pub.messages[-1])

    def test_duplicates_after_timeout_cannot_keep_valid_state_alive(self):
        node=self.receiver();node.confirm_valid_frames=3
        for seq in (1,2,3):
            self.receive(node,self.packet(frame_seq=seq,producer_session_id='p',camera_time_epoch='e'))
        node.last_active_mono-=1.
        before=node.last_active_mono
        d=self.receive(node,self.packet(frame_seq=3,producer_session_id='p',camera_time_epoch='e'))
        self.assertFalse(d['forwarded']);self.assertEqual(node.last_active_mono,before)
        self.assertEqual(node.valid_streak,0);self.assertFalse(node.valid_pub.messages[-1])

    def test_timeout_reconfirms_without_allowing_same_epoch_sequence_reset(self):
        node=self.receiver();node.confirm_valid_frames=3
        for seq in (1,2,3):self.receive(node,self.packet(frame_seq=seq,camera_time_epoch='a'))
        node.last_active_mono-=1.
        self.receive(node,self.packet(frame_seq=4,camera_time_epoch='a'))
        self.assertEqual(node.valid_streak,1);self.assertFalse(node.valid_pub.messages[-1])
        node.last_active_mono-=1.
        d=self.receive(node,self.packet(frame_seq=1,camera_time_epoch='a'))
        self.assertFalse(d['forwarded']);self.assertEqual(node.last_frame_seq,4)

    def test_missing_or_invalid_session_identity_does_not_downgrade(self):
        node=self.receiver()
        self.receive(node,self.packet(frame_seq=1,camera_time_epoch='a'))
        d=self.receive(node,self.packet(frame_seq=2))
        self.assertEqual(d['receiver_reason'],'session_identity_missing')
        before=node.active_session
        for epoch in ('',12,'x'*129):
            d=self.receive(node,self.packet(frame_seq=3,camera_time_epoch=epoch))
            self.assertEqual(d['event'],'bad_packet');self.assertEqual(node.active_session,before)

    def test_invalid_future_or_backward_capture_does_not_advance_state(self):
        node=self.receiver()
        self.receive(node,self.packet(frame_seq=10,camera_time_epoch='a'))
        for capture in (NOW+51_000_000,NOW-200_000_000):
            d=self.receive(node,self.packet(frame_seq=999,camera_time_epoch='a',capture_timestamp_ns=capture,
                publish_timestamp_ns=max(NOW-10_000_000,capture+1)))
            self.assertFalse(d['forwarded']);self.assertEqual(node.last_frame_seq,10)

    def test_repeated_rejected_frames_cannot_keep_timeout_heartbeat_alive(self):
        node=self.receiver()
        self.receive(node,self.packet(frame_seq=100,camera_time_epoch='a'))
        node.last_active_mono-=1.
        for _ in range(3):self.receive(node,self.packet(frame_seq=99,camera_time_epoch='a'))
        node.health_timer(None)
        self.assertFalse(node.valid_pub.messages[-1])
        d=json.loads(node.diag_json_pub.messages[-1])
        self.assertEqual(d['receiver_reason'],'network_timeout')
        self.assertGreater(d['last_active_age_ms'],800.)

    def test_same_sequence_new_epoch_is_a_distinct_confirmation_frame(self):
        node=self.receiver();node.confirm_valid_frames=3
        self.receive(node,self.packet(frame_seq=7,camera_time_epoch='a'))
        d=self.receive(node,self.packet(frame_seq=7,camera_time_epoch='b'))
        self.assertTrue(d['forwarded']);self.assertTrue(d['session_changed'])
        self.assertEqual(node.valid_streak,1);self.assertFalse(node.valid_pub.messages[-1])

    def test_receiver_age_rejection_is_explicit(self):
        node=self.receiver(); diagnostic=self.receive(node,self.packet(source_age_ms=801.))
        self.assertFalse(node.valid_pub.messages[-1])
        self.assertEqual(diagnostic['receiver_reason'],'receiver_source_age_exceeded')

    def test_receiver_confidence_060_boundary(self):
        node = self.receiver()
        diagnostic = self.receive(node, self.packet(confidence=.599))
        self.assertFalse(node.valid_pub.messages[-1])
        self.assertEqual(diagnostic['receiver_reason'], 'low_confidence')
        self.receive(node, self.packet(frame_seq=2, confidence=.60))
        self.assertTrue(node.valid_pub.messages[-1])

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
        limits = updates.pop('_limits', {})
        ns=load(ROOT/'osm_map_handler/src/osm_camera_path_refinement.py')
        def reached(*args): raise ValueError('geometry_reached')
        ns['boundary_curve']=reached
        cls=ns['CameraRightPath']; node=cls.__new__(cls)
        node.packet=self.packet(**updates)
        node.received=10.;node.timeout=.5;node.receiver_valid=True;node.valid_received=10.
        node.safe=True;node.safe_received=10.;node.control=False
        node.listener=NS(lookupTransform=lambda *args: (np.zeros(3),[0,0,0,1]))
        node.lidar_origin=np.zeros(3);node.resolution=(1920,1080)
        for key, value in limits.items(): setattr(node,key,value)
        for key in ('matrix','distortion','rotation','origin','ground_z'): setattr(node,key,None)
        try: node.refine(10.)
        except ValueError as error: return str(error)
        self.fail('Expected diagnostic or geometry sentinel')

    def test_distinct_confidence_and_source_age_rejections(self):
        self.assertEqual(self.right_reason(confidence=.599),'low_confidence')
        self.assertEqual(self.right_reason(source_age_ms=300.001),'source_age_exceeded_300ms')

    def test_capture_clock_rejections(self):
        self.assertEqual(self.right_reason(capture_timestamp_ns=NOW-500_001_000),
                         'capture_age_exceeded_500ms_or_clock_skew')
        self.assertEqual(self.right_reason(capture_timestamp_ns=NOW+50_001_000),
                         'capture_timestamp_in_future_clock_skew')

    def test_confidence_060_inclusive_and_original_age_limits(self):
        for capture in (NOW-500_000_000,NOW+50_000_000):
            self.assertEqual(self.right_reason(confidence=.60,source_age_ms=300.,
                             capture_timestamp_ns=capture),'geometry_reached')

    def test_map_cache_profile_is_moderate_and_bounded(self):
        limits=dict(max_source_age_ms=500.,max_capture_age_ms=800.)
        self.assertEqual(self.right_reason(source_age_ms=400.,
            capture_timestamp_ns=NOW-650_000_000,_limits=limits),'geometry_reached')
        self.assertEqual(self.right_reason(source_age_ms=500.001,_limits=limits),'source_age_exceeded_500ms')
        self.assertEqual(self.right_reason(capture_timestamp_ns=NOW-800_001_000,_limits=limits),
                         'capture_age_exceeded_800ms_or_clock_skew')
        self.assertEqual(self.right_reason(confidence=.599,_limits=limits),'low_confidence')
        self.assertEqual(self.right_reason(source_age_ms=49789.,_limits=limits),'source_age_exceeded_500ms')

    def test_camera_epoch_resets_pending_not_confirmed_map(self):
        ns=load(ROOT/'osm_map_handler/src/osm_camera_path_refinement.py')
        cls=ns['CameraRightPath'];node=cls.__new__(cls)
        node.lock=threading.RLock();node.last_camera_epoch='before';node.last_seq=99
        node.received=time.monotonic();node.confirmations=2
        node.cache=NS(pending={1:(.2,2)},committed={2:(.3,10.)})
        node.json_cb(NS(data=json.dumps(dict(frame_seq=1,camera_time_epoch='after'))))
        self.assertEqual(node.cache.pending,{})
        self.assertEqual(node.cache.committed,{2:(.3,10.)})
        self.assertEqual(node.confirmations,0)
        self.assertEqual(node.last_seq,1)

    def test_path_node_resets_on_worker_restart_even_if_camera_epoch_is_same(self):
        ns=load(ROOT/'osm_map_handler/src/osm_camera_path_refinement.py')
        node=ns['CameraRightPath'].__new__(ns['CameraRightPath'])
        node.lock=threading.RLock();node.last_seq=None;node.confirmations=0
        node.received=0.;node.cache=NS(pending={},committed={2:(.3,10.)})
        node.json_cb(NS(data=json.dumps(dict(frame_seq=99,camera_time_epoch='same',receiver_session_id='worker1'))))
        node.confirmations=2;node.cache.pending[1]=(.2,2)
        node.json_cb(NS(data=json.dumps(dict(frame_seq=1,camera_time_epoch='same',receiver_session_id='worker2'))))
        self.assertEqual(node.confirmations,0);self.assertEqual(node.last_seq,1)
        self.assertEqual(node.cache.pending,{});self.assertTrue(node.cache.committed)
        packet=node.packet;received=node.received
        node.json_cb(NS(data=json.dumps(dict(frame_seq=100,camera_time_epoch='same',receiver_session_id='worker1'))))
        self.assertIs(node.packet,packet);self.assertEqual(node.received,received)

    def test_path_duplicate_cannot_refresh_packet_after_a_pause(self):
        ns=load(ROOT/'osm_map_handler/src/osm_camera_path_refinement.py')
        node=ns['CameraRightPath'].__new__(ns['CameraRightPath'])
        node.lock=threading.RLock();node.last_seq=None;node.confirmations=0;node.received=0.
        packet=dict(frame_seq=10,camera_time_epoch='same',receiver_session_id='worker1')
        node.json_cb(NS(data=json.dumps(packet)))
        node.received-=3.;received=node.received
        node.json_cb(NS(data=json.dumps(packet)))
        self.assertEqual(node.received,received);self.assertEqual(node.last_seq,10)

    def test_path_checks_this_frame_confirmation_not_only_shared_bool_topic(self):
        self.assertEqual(self.right_reason(receiver_diagnostics={'receiver_valid':False,'frame_seq':1}),
                         'receiver has not confirmed this frame')
        self.assertEqual(self.right_reason(receiver_diagnostics={'receiver_valid':True,'frame_seq':99}),
                         'receiver has not confirmed this frame')
        self.assertEqual(self.right_reason(receiver_diagnostics={'receiver_valid':True,'frame_seq':1}),
                         'geometry_reached')


if __name__ == '__main__': unittest.main()
