"""Exact source-time admission shared by GPS projection and map alignment.

No ROS dependency. Measurement metadata is a bounded side channel because
standard NavSatFix/Gpgga cannot carry the receiver identity/clock provenance.
"""
from collections import OrderedDict


class GnssTimebase:
    def __init__(self, mode='shadow', verified=False):
        if verified not in (True, False, 'auto'):
            raise ValueError('clock verification must be bool or auto')
        if mode not in ('host', 'shadow', 'measurement') or (mode == 'measurement' and not verified):
            raise ValueError('measurement time requires explicit verified-clock deployment')
        self.mode = mode
        self.auto_clock = verified == 'auto'
        self.clock_ready = verified is True
        self.clock_established = self.clock_ready
        self.wait_reason = 'waiting_for_time_metadata'
        self.fault = None
        self.session = None
        self.previous_clock = None
        self.metadata = OrderedDict()
        self.latest = {}
        self.watermarks = {}

    def tick(self, ros_ns, mono_ns):
        if self.mode != 'measurement':
            return
        if self.previous_clock is not None:
            r, m = self.previous_clock
            if mono_ns < m or abs((ros_ns-r)-(mono_ns-m)) > 250000000:
                if not (self.auto_clock and not self.clock_established):
                    self.fault = 'host_clock_jump_restart_required'
        self.previous_clock = ros_ns, mono_ns

    def receive(self, event, ros_ns, mono_ns):
        self.tick(ros_ns, mono_ns)
        incoming = event.get('mode')
        if (incoming == 'measurement') != (self.mode == 'measurement'):
            self.fault = 'time_mode_mismatch_restart_required'
            return
        if self.mode != 'measurement':
            return
        session = event.get('session_id'), event.get('clock_epoch')
        if not isinstance(session[0], str) or not session[0] or type(session[1]) is not int:
            self.fault = 'invalid_time_session'
            return
        if (self.session is not None and session != self.session
                and not (self.auto_clock and not self.clock_established)):
            self.fault = 'time_session_changed_restart_required'
        self.session = session
        if event.get('clock_verified') is not True or event.get('clock_jump'):
            self.wait_reason = event.get('clock_gate_reason', 'waiting_for_verified_clock')
            recoverable=(event.get('clock_waiting') is True and self.auto_clock
                         and (not event.get('clock_jump') or not self.clock_established))
            if recoverable:self.clock_ready=False
            else:
                self.fault = 'receiver_clock_unverified_restart_required'
            return
        self.clock_ready = True
        self.clock_established = True
        self.wait_reason = 'ready'
        family = event.get('family')
        if family not in ('GGA', 'PVTSLNA'):
            return
        stamp = event.get('measurement_stamp_ns')
        valid = (event.get('time_valid') is True and type(stamp) is int and stamp > 0
                 and event.get('observation_id') == '{}:{}:{}:{}'.format(*session, family, stamp)
                 and stamp <= ros_ns+50000000)
        previous = self.latest.get(family)
        if not valid:
            self.latest[family] = dict(event, time_valid=False)
            return
        key = family, stamp
        if key not in self.metadata:
            self.metadata[key] = dict(event)
            if len(self.metadata) > 256:
                self.metadata.popitem(last=False)
        # A duplicate cannot revive an invalid latest observation. No invalid
        # stamp advances a source watermark; out-of-order data stays historical.
        old_stamp = self.watermarks.get(family, 0)
        if stamp > old_stamp:
            self.watermarks[family] = stamp
            self.latest[family] = self.metadata[key]

    def check(self, family, stamp, ros_ns, mono_ns, timeout, period_sec=0., margin_sec=.3):
        self.tick(ros_ns, mono_ns)
        if self.fault:
            return False, self.fault
        if self.mode != 'measurement':
            return True, 'host_contract'
        if not self.clock_ready:
            return False, 'waiting_for_measurement_clock'
        latest = self.latest.get(family)
        if latest is not None and latest.get('time_valid') is not True:
            return False, 'measurement_time_invalid_'+family
        event = self.metadata.get((family, stamp))
        if event is None:
            return False, 'position_source_time_mismatch'
        read = event.get('host_read_mono_ns')
        receive_deadline = period_sec+margin_sec if period_sec else timeout
        source_deadline = timeout+receive_deadline if period_sec else timeout
        if period_sec:
            read_ros = event.get('host_read_ros_ns')
            if type(read_ros) is not int or not -.05 <= (read_ros-stamp)*1e-9 <= timeout:
                return False, 'measurement_transport_late_'+family
        if type(read) is not int or not 0 <= mono_ns-read <= receive_deadline*1e9:
            return False, 'measurement_receive_stale_'+family
        if not -.05 <= (ros_ns-stamp)*1e-9 <= source_deadline:
            return False, 'measurement_source_stale_'+family
        return True, 'measurement_paired'
