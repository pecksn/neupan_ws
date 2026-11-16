"""ROS-free regression tests for the single-producer challenge/lease contract."""
from pathlib import Path
import socket
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch

modules = Path(__file__).resolve().parents[2]/'hybridnets_receiver/src'
sys.path.insert(0,str(modules))
from hybridnets_binding import ProducerAuthority, ReceiverBinding

PEER=('192.168.10.21',12345)


class BindingTests(unittest.TestCase):
    def setUp(self):
        self.current=['epoch-b',True]
        self.server=ProducerAuthority.__new__(ProducerAuthority)
        self.server.producer='producer-b'
        self.server.identity=lambda:tuple(self.current)
        self.server.peer=PEER[0]
        self.server.lock=threading.Lock()
        self.server.lease=None
        self.server.challenges={}
        self.client=ReceiverBinding()

    def handshake(self,now=10.,client=None):
        c=client or self.client
        probe=c.request(now)
        self.assertIsNotNone(probe)
        identity=self.server.handle(probe,PEER,now+.01)
        bind=c.reply(identity,now+.02)
        self.assertIsNotNone(bind)
        ack=self.server.handle(bind,PEER,now+.03)
        self.assertIsNotNone(ack)
        c.reply(ack,now+.04)
        self.assertTrue(c.alive(now+.04))
        return probe,bind,ack

    def packet(self):
        b=self.client.bound
        return dict(producer_session_id=b['identity'][0],camera_time_epoch=b['identity'][1],
                    binding_ticket=b['ticket'],receiver_instance_id=self.client.instance_id)

    def test_complete_handshake_and_nonce_size(self):
        probe,bind,ack=self.handshake()
        self.assertEqual(len(probe['nonce']),64)
        self.assertNotEqual(probe['nonce'],bind['nonce'])
        self.assertTrue(self.client.accepts(self.packet(),10.1)[0])

    def test_unseen_old_A_never_replaces_current_B(self):
        self.handshake()
        before=dict(self.client.bound)
        for _ in range(3):
            p=self.packet();p['producer_session_id']='unseen-old-a'
            self.assertEqual(self.client.accepts(p,10.1)[1],'binding_identity_mismatch')
        self.assertEqual(self.client.bound,before)
        self.assertTrue(self.client.accepts(self.packet(),10.2)[0])

    def test_old_ticket_or_receiver_rejected(self):
        self.handshake()
        p=self.packet();p['binding_ticket']='0'*64
        self.assertFalse(self.client.accepts(p,10.1)[0])
        p=self.packet();p['receiver_instance_id']='0'*64
        self.assertFalse(self.client.accepts(p,10.1)[0])

    def test_wrong_nonce_and_late_identity_do_not_commit(self):
        p=self.client.request(10.)
        response=self.server.handle(p,PEER,10.01)
        response['nonce']='0'*64
        self.assertIsNone(self.client.reply(response,10.02))
        response['nonce']=p['nonce']
        self.assertIsNone(self.client.reply(response,11.01))
        self.assertIsNone(self.client.bound)

    def test_late_or_wrong_receiver_ack_does_not_commit(self):
        p=self.client.request(10.)
        bind=self.client.reply(self.server.handle(p,PEER,10.01),10.02)
        ack=self.server.handle(bind,PEER,10.03)
        wrong=dict(ack,receiver_instance_id='0'*64)
        self.assertIsNone(self.client.reply(wrong,10.04))
        self.client.reply(ack,11.03)
        self.assertIsNone(self.client.bound)

    def test_epoch_change_during_handshake_rejects_bind(self):
        p=self.client.request(10.)
        bind=self.client.reply(self.server.handle(p,PEER,10.01),10.02)
        self.current[0]='epoch-new'
        self.assertIsNone(self.server.handle(bind,PEER,10.03))
        self.assertIsNone(self.server.lease)

    def test_not_ready_clock_cannot_bind_or_stamp(self):
        self.current[1]=False
        p=self.client.request(10.)
        self.assertIsNone(self.client.reply(self.server.handle(p,PEER,10.01),10.02))
        self.assertIsNone(self.server.snapshot(10**30))

    def test_pre_authorization_frame_cannot_get_new_ticket(self):
        with patch('hybridnets_binding.time.time_ns',return_value=1000):
            self.handshake()
        with patch('hybridnets_binding.time.monotonic',return_value=10.1):
            self.assertIsNone(self.server.snapshot(999))
            self.assertEqual(self.server.snapshot(1001),self.client.bound['ticket'])

    def test_inflight_old_ticket_cannot_be_relabelled_after_rebind(self):
        self.handshake()
        old=self.packet()
        self.current[0]='epoch-new'
        self.client.next_probe=11.
        self.handshake(11.)
        self.assertFalse(self.client.accepts(old,11.1)[0])
        self.assertNotEqual(old['binding_ticket'],self.client.bound['ticket'])

    def test_lease_expires_without_data_extending_it(self):
        self.handshake()
        p=self.packet()
        self.assertFalse(self.client.accepts(p,16.)[0])
        with patch('hybridnets_binding.time.monotonic',return_value=16.):
            self.assertIsNone(self.server.snapshot(10**30))

    def test_renewal_keeps_ticket_and_capture_activation(self):
        self.handshake()
        old=dict(self.server.lease)
        self.handshake(12.)
        self.assertEqual(old['ticket'],self.server.lease['ticket'])
        self.assertEqual(old['activation_ns'],self.server.lease['activation_ns'])
        self.assertGreater(self.server.lease['deadline'],old['deadline'])

    def test_second_receiver_cannot_take_live_lease(self):
        self.handshake()
        other=ReceiverBinding()
        p=other.request(11.)
        bind=other.reply(self.server.handle(p,PEER,11.01),11.02)
        before=dict(self.server.lease)
        self.assertIsNone(self.server.handle(bind,PEER,11.03))
        self.assertEqual(self.server.lease,before)
        self.assertIsNone(other.bound)

    def test_receiver_restart_gets_fresh_ticket_after_expiry(self):
        self.handshake()
        old=self.packet()
        new=ReceiverBinding()
        self.handshake(16.,new)
        self.assertNotEqual(new.instance_id,self.client.instance_id)
        self.assertNotEqual(new.bound['ticket'],old['binding_ticket'])
        self.assertFalse(new.accepts(old,16.1)[0])

    def test_old_bind_request_cannot_be_replayed(self):
        _,bind,_=self.handshake()
        before=dict(self.server.lease)
        self.assertIsNone(self.server.handle(bind,PEER,12.))
        self.assertEqual(self.server.lease,before)

    def test_wrong_peer_cannot_query_authority(self):
        self.assertIsNone(self.server.handle(self.client.request(10.),('192.168.10.99',12345),10.))

    def test_authority_reply_revokes_old_producer_before_new_bind_ack(self):
        self.handshake()
        self.server.producer='producer-new'
        self.server.invalidate()
        probe=self.client.request(12.)
        bind=self.client.reply(self.server.handle(probe,PEER,12.01),12.02)
        self.assertIsNotNone(bind)
        self.assertIsNone(self.client.bound)
        self.assertEqual(self.client.reason,'authority_changed_waiting_bind')

    def test_authority_clock_failure_clears_both_sides_lease(self):
        self.handshake()
        self.current[1]=False
        probe=self.client.request(12.)
        self.client.reply(self.server.handle(probe,PEER,12.01),12.02)
        self.assertIsNone(self.client.bound)
        self.assertIsNone(self.server.lease)
        self.current[1]=True
        self.assertIsNone(self.server.snapshot(10**30))

    def test_lost_probe_cannot_extend_lease_and_cancelled_reply_is_ignored(self):
        self.handshake()
        old_deadline=self.client.bound['deadline']
        probe=self.client.request(12.)
        old_reply=self.server.handle(probe,PEER,12.01)
        self.client.request(13.1)  # timeout cancels the pending nonce
        self.client.reply(old_reply,13.11)
        self.assertEqual(self.client.bound['deadline'],old_deadline)
        self.assertFalse(self.client.alive(16.))

    def test_previous_receiver_ack_cannot_bind_new_receiver(self):
        _,_,ack=self.handshake()
        other=ReceiverBinding()
        other.request(10.)
        other.reply(ack,10.1)
        self.assertIsNone(other.bound)

    def test_fixed_port_and_lock_prevent_second_producer(self):
        with tempfile.TemporaryDirectory() as tmp:
            a=ProducerAuthority('a',lambda:(None,True),'127.0.0.1',0,tmp+'/lock')
            try:
                with self.assertRaises(RuntimeError):
                    ProducerAuthority('b',lambda:(None,True),'127.0.0.1',0,tmp+'/lock')
                with self.assertRaises(RuntimeError):
                    ProducerAuthority('b',lambda:(None,True),'127.0.0.1',a.sock.getsockname()[1],tmp+'/other-lock')
            finally:
                a.close()
            b=ProducerAuthority('b',lambda:(None,True),'127.0.0.1',0,tmp+'/lock')
            b.close()


if __name__=='__main__':
    unittest.main()
