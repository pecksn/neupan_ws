"""Single-producer authority and challenge/lease protocol (stdlib, ROS independent).

This prevents accidental replay/session takeover on a trusted isolated LAN.
It is NOT cryptographic sender authentication; UDP peers still need IP filtering.
"""
import fcntl
import json
import os
import secrets
import socket
import threading
import time

PROTOCOL = 'hybridnets-binding-v1'
LEASE_MS = 5000


def opaque(value):
    return isinstance(value, str) and len(value) == 64 and all(c in '0123456789abcdef' for c in value)


class ProducerAuthority:
    def __init__(self, producer, identity, peer, port=6001, lock_path=None, on_bind=None):
        self.producer = producer
        self.identity = identity  # returns (camera_epoch, time_ready)
        self.on_bind = on_bind
        self.peer = peer
        self.lock = threading.Lock()
        self.lease = None
        self.challenges = {}
        self.stop = threading.Event()
        path = lock_path or '/tmp/hybridnets-producer-%d.lock' % os.getuid()
        self.fd = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
        self.sock = None
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # No REUSEADDR: only the lock holder can own the fixed management port.
            self.sock.bind(('0.0.0.0', port))
            self.sock.settimeout(.1)
        except Exception:
            if self.sock is not None:
                self.sock.close()
            os.close(self.fd)
            raise RuntimeError('HybridNets publisher already running, or management port unavailable')
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def handle(self, request, sender, now=None):
        now = time.monotonic() if now is None else now
        if sender[0] != self.peer or not isinstance(request, dict):
            return None
        if (request.get('protocol') != PROTOCOL or not opaque(request.get('nonce')) or
                not opaque(request.get('receiver_instance_id'))):
            return None
        epoch, ready = self.identity()
        with self.lock:
            if not ready or (self.lease and self.lease['epoch'] != epoch):
                self.lease = None
        answer = dict(protocol=PROTOCOL, nonce=request['nonce'],
                      receiver_instance_id=request['receiver_instance_id'],
                      producer_session_id=self.producer, camera_time_epoch=epoch)
        if request.get('op') == 'probe':
            with self.lock:
                self.challenges = {key: value for key,value in self.challenges.items() if value[0]>now}
                if len(self.challenges) >= 64:
                    return None
                self.challenges[(sender, request['nonce'])] = (now+1., epoch, request['receiver_instance_id'])
            answer.update(op='identity', ready=bool(ready))
            return answer
        if request.get('op') != 'bind' or not ready:
            return None
        if (request.get('producer_session_id') != self.producer or
                request.get('camera_time_epoch') != epoch or not opaque(request.get('binding_ticket'))):
            return None
        ticket = request['binding_ticket']
        with self.lock:
            challenge = self.challenges.pop((sender, request.get('probe_nonce')), None)
            if (challenge is None or challenge[0] <= now or challenge[1] != epoch or
                    challenge[2] != request['receiver_instance_id']):
                return None
            # One receiver owns a live lease. A second receiver cannot replace it.
            if (self.lease and self.lease['deadline'] > now and
                    self.lease['epoch'] == epoch and (self.lease['ticket'] != ticket or
                    self.lease['receiver'] != request['receiver_instance_id'])):
                return None
            renewing = (self.lease and self.lease['ticket'] == ticket and
                        self.lease['epoch'] == epoch and self.lease['deadline'] > now)
            activation = self.lease['activation_ns'] if renewing else time.time_ns()
            self.lease = dict(ticket=ticket, epoch=epoch, deadline=now+LEASE_MS/1000.,
                              activation_ns=activation, receiver=request['receiver_instance_id'])
        if not renewing and getattr(self, 'on_bind', None):
            self.on_bind()  # outside authority lock: drop queued old image context
        answer.update(op='bound', binding_ticket=ticket, lease_ms=LEASE_MS)
        return answer

    def snapshot(self, capture_ns):
        return self.frame_binding(capture_ns).get('binding_ticket')

    def invalidate(self):
        with self.lock:
            self.lease = None
            self.challenges.clear()

    def frame_binding(self, capture_ns):
        epoch, ready = self.identity()
        with self.lock:
            lease = self.lease
            if not ready or (lease and lease['epoch'] != epoch):
                self.lease = None
                return {}
            if (not ready or not lease or lease['deadline'] <= time.monotonic() or
                    lease['epoch'] != epoch or capture_ns < lease['activation_ns']):
                return {}
            return dict(binding_ticket=lease['ticket'], receiver_instance_id=lease['receiver'])

    def run(self):
        while not self.stop.is_set():
            try:
                raw, sender = self.sock.recvfrom(4096)
                response = self.handle(json.loads(raw.decode('utf8')), sender)
                if response is not None:
                    self.sock.sendto(json.dumps(response).encode('utf8'), sender)
            except socket.timeout:
                pass
            except (ValueError, UnicodeError, TypeError, OSError):
                pass  # malformed management traffic never changes the authority

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1.)
        self.sock.close()
        os.close(self.fd)  # release flock; never unlink the lock inode


class ReceiverBinding:
    """Monotonic lease state machine. Only challenged replies can change binding."""
    def __init__(self):
        self.instance_id = secrets.token_hex(32)
        self.bound = None
        self.pending = None
        self.next_probe = 0.
        self.reason = 'binding_unconfirmed'

    def alive(self, now):
        return self.bound is not None and self.bound['deadline'] > now

    def request(self, now):
        if self.pending:
            if now <= self.pending['sent']+1.:
                return None
            self.pending = None
            self.next_probe = now+.2
            self.reason = 'binding_reply_timeout'
        if now < self.next_probe:
            return None
        message = dict(protocol=PROTOCOL, op='probe', nonce=secrets.token_hex(32), receiver_instance_id=self.instance_id)
        self.pending = dict(message=message, sent=now)
        return message

    def reply(self, response, now):
        pending = self.pending
        if not pending or not isinstance(response, dict) or now > pending['sent']+1.:
            return None
        request = pending['message']
        if (response.get('protocol') != PROTOCOL or response.get('nonce') != request['nonce'] or
                response.get('receiver_instance_id') != self.instance_id):
            return None
        producer, epoch = response.get('producer_session_id'), response.get('camera_time_epoch')
        if (not isinstance(producer, str) or not producer or len(producer)>128 or
                (epoch is not None and (not isinstance(epoch, str) or not epoch or len(epoch)>128))):
            return None
        identity = (producer, epoch)
        if request['op'] == 'probe':
            if response.get('op') != 'identity' or type(response.get('ready')) is not bool:
                return None
            if response['ready'] is False:
                self.bound = self.pending = None
                self.reason = 'authority_time_not_ready'
                self.next_probe = now+.2
                return None
            if self.bound and self.bound['identity'] != identity:
                # A challenged authority reply explicitly invalidates the old
                # producer. Do not keep using it while its replacement binds.
                self.bound = None
                self.reason = 'authority_changed_waiting_bind'
            reuse = self.alive(now) and self.bound['identity'] == identity
            ticket = self.bound['ticket'] if reuse else secrets.token_hex(32)
            message = dict(protocol=PROTOCOL, op='bind', nonce=secrets.token_hex(32),
                           receiver_instance_id=self.instance_id,
                           probe_nonce=request['nonce'],
                           producer_session_id=producer, camera_time_epoch=epoch, binding_ticket=ticket)
            self.pending = dict(message=message, sent=now)
            return message
        if (response.get('op') != 'bound' or response.get('binding_ticket') != request['binding_ticket'] or
                identity != (request['producer_session_id'], request['camera_time_epoch']) or
                type(response.get('lease_ms')) is not int or not 1000 <= response['lease_ms'] <= LEASE_MS):
            return None
        deadline = pending['sent']+response['lease_ms']/1000.-.2
        if deadline <= now:
            return None
        self.bound = dict(identity=identity, ticket=request['binding_ticket'], deadline=deadline)
        self.pending = None
        self.next_probe = now+1.5
        self.reason = 'bound'
        return None

    def accepts(self, packet, now):
        if not self.alive(now):
            return False, 'binding_expired' if self.bound else 'binding_unconfirmed'
        if (packet.get('producer_session_id'), packet.get('camera_time_epoch')) != self.bound['identity']:
            return False, 'binding_identity_mismatch'
        if packet.get('binding_ticket') != self.bound['ticket']:
            return False, 'binding_ticket_mismatch'
        if packet.get('receiver_instance_id') != self.instance_id:
            return False, 'binding_receiver_mismatch'
        return True, 'bound'

    def status(self, now):
        return dict(confirmed=self.alive(now), reason=self.reason if self.alive(now) else
                    ('binding_expired' if self.bound else self.reason),
                    receiver_instance_id=self.instance_id,
                    producer_session_id=self.bound['identity'][0] if self.bound else None,
                    camera_time_epoch=self.bound['identity'][1] if self.bound else None,
                    lease_remaining_ms=max(0,(self.bound['deadline']-now)*1000) if self.bound else 0)
