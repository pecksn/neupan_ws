"""Passive GNSS observation timing; never changes a navigation timestamp.

N4 ASCII header: message,idle,time_ref,time_status,week,milliseconds,
version,reserved,leap_seconds,output_delay (N4 manual table 7-49).
Only GPS/GPST + FINE is supported; other time systems fail closed.
"""
import calendar
import ctypes
from collections import Counter, deque
import datetime as dt
import re
import threading
import uuid

from libnmea_navsat_driver.checksum_utils import check_nmea_checksum
from libnmea_navsat_driver.unicore import crc32

NS = 1000000000
DAY_NS = 86400 * NS
GPS_EPOCH = calendar.timegm((1980, 1, 6, 0, 0, 0))


def kernel_clock_synchronized():
    """Read-only Linux clock state. Zero modes never adjusts the clock.

    Use aligned over-sized output storage and only the documented return code,
    avoiding dependence on timex field offsets. TIME_OK=0; any error/leap state
    is not admitted for automatic startup. This is not a UTC accuracy proof.
    """
    try:
        query = ctypes.CDLL(None, use_errno=True).adjtimex
        query.argtypes, query.restype = [ctypes.c_void_p], ctypes.c_int
        storage = (ctypes.c_long * 64)()  # Zero initialized, including modes=0.
        return query(ctypes.byref(storage)) == 0
    except (AttributeError, OSError):
        return False


def utc_day_time(text):
    """NMEA UTC time-of-day to integer ns, with no host date or float roundtrip."""
    if not re.fullmatch(r'\d{6}(?:\.\d{1,9})?', text):
        raise ValueError('invalid_utc_time')
    hour, minute, second = int(text[:2]), int(text[2:4]), int(text[4:6])
    if hour > 23 or minute > 59 or second > 59:
        raise ValueError('utc_range_or_leap_second_unsupported')
    fraction = text.partition('.')[2]
    return (hour * 3600 + minute * 60 + second) * NS + int((fraction+'0'*9)[:9])


def pvtslna_time(sentence):
    body, checksum = sentence.strip().rsplit('*', 1)
    if len(checksum) != 8 or crc32(body[1:]) != int(checksum, 16):
        raise ValueError('invalid_pvtslna_crc')
    header = body.split(';', 1)[0].split(',')
    if len(header) != 10 or header[0] != '#PVTSLNA':
        raise ValueError('invalid_pvtslna_header')
    raw = dict(zip(('time_reference', 'time_status', 'week', 'week_ms',
                    'version', 'reserved', 'leap_seconds', 'output_delay_ms'), header[2:]))
    if header[2].upper() not in ('GPS', 'GPST'):
        raise ValueError('unsupported_time_reference')
    if header[3].upper() != 'FINE':
        raise ValueError('receiver_time_not_fine')
    week, milliseconds, leap, delay = (int(header[i]) for i in (4, 5, 8, 9))
    if not (1024 <= week <= 65535 and 0 <= milliseconds < 604800000
            and 0 <= leap <= 64 and 0 <= delay <= 65535):
        raise ValueError('receiver_time_fields_out_of_range')
    # Leap offset comes from this CRC-checked receiver header, never a constant.
    stamp = (GPS_EPOCH + week * 604800 - leap) * NS + milliseconds * 1000000
    return stamp, raw


class ObservationTiming:
    """Bounded shadow instrumentation, independent from Fixed/Float quality.

    clock_verified is a deployment assertion, NOT inferred from a small delta.
    A diagnostic ROS clock jump invalidates date anchors/statistics but never
    touches the navigation bridge, its stable windows or continuation budgets.
    """
    def __init__(self, mode='shadow', clock_verified=False, clock_probe=None):
        if mode not in ('host', 'shadow', 'measurement'):
            raise ValueError('time_mode must be host/shadow/measurement')
        if mode == 'measurement' and not clock_verified:
            raise ValueError('measurement mode requires independently verified common UTC clocks')
        self.mode = mode
        if clock_verified not in (True, False, 'auto'):
            raise ValueError('clock verification must be bool or auto')
        self.auto_clock = clock_verified == 'auto'
        self.clock_verified = clock_verified is True
        self.clock_probe = clock_probe or kernel_clock_synchronized
        self.host_clock_synced = False
        self.last_clock_probe = None
        self.auto_samples = {k: deque(maxlen=80) for k in ('GGA', 'PVTSLNA')}
        self.auto_failed = False
        self.auto_reason = 'waiting_for_valid_measurement_time' if self.auto_clock else 'explicit_confirmation'
        self.session_id = uuid.uuid4().hex
        self.lock = threading.RLock()
        self.clock_epoch = self.sequence = 0
        self.previous_clock = self.anchor = None
        self.watermarks = {}
        self.counts = Counter()
        self.ages = {k: deque(maxlen=512) for k in ('GGA', 'PVTSLNA', 'RMC')}

    def _clock(self, ros_ns, mono_ns):
        jumped = False
        if self.previous_clock is not None:
            r, m = self.previous_clock
            jumped = mono_ns < m or abs((ros_ns-r)-(mono_ns-m)) > 250000000
        self.previous_clock = (ros_ns, mono_ns)
        if jumped:
            was_verified = self.clock_verified
            self.clock_epoch += 1
            self.clock_verified = False
            self.auto_failed = self.auto_failed or (self.auto_clock and was_verified)
            self.auto_reason = ('clock_jump_restart_required' if self.auto_failed
                                else 'waiting_after_startup_clock_adjustment')
            for samples in self.auto_samples.values():
                samples.clear()
            self.anchor = None
            self.watermarks.clear()
            for samples in self.ages.values():
                samples.clear()
            self.counts['clock_jumps'] += 1
        return jumped

    def _auto_verify(self, event):
        if not self.auto_clock:
            return
        mono = event['host_read_mono_ns']
        if self.last_clock_probe is None or mono-self.last_clock_probe >= NS:
            self.host_clock_synced = self.clock_probe() is True
            self.last_clock_probe = mono
        valid = (event['time_valid'] and event['epoch_order'] == 'new'
                 and -.05 <= event['candidate_age_ms']/1000. <= .5)
        if self.clock_verified:
            if not self.host_clock_synced or (event['time_valid'] and
                    not -.05 <= event['candidate_age_ms']/1000. <= .5):
                self.auto_failed = True
                self.auto_reason = 'clock_disagreement_or_unsynchronized_restart_required'
                self.clock_verified = False
        elif not self.auto_failed:
            family = event['family']
            if family in self.auto_samples:
                if valid and self.host_clock_synced:
                    samples = self.auto_samples[family]
                    if samples and mono-samples[-1] > 500000000:
                        samples.clear()
                    samples.append(mono)
                    # Only a continuous recent window can qualify startup.
                    while samples and mono-samples[0] > 3*NS:
                        samples.popleft()
                elif event.get('epoch_order') not in ('duplicate', 'out_of_order'):
                    self.auto_samples[family].clear()
            histories = self.auto_samples.values()
            if self.host_clock_synced and all(len(s) >= 10 and
                    s[-1]-s[0] >= 2*NS and mono-s[-1] <= 500000000 for s in histories):
                self.clock_verified = True
                self.auto_reason = 'kernel_sync_and_gnss_utc_consistent'
            elif not self.host_clock_synced:
                self.auto_reason = 'waiting_for_nuc_clock_sync'
            else:
                self.auto_reason = 'waiting_for_fine_gnss_and_gga_consistency'
        event.update(clock_verified=self.clock_verified,
            clock_waiting=not self.clock_verified and not self.auto_failed,
            clock_verification='auto_consistency_not_hardware_sync',
            host_clock_synchronized=self.host_clock_synced, clock_gate_reason=self.auto_reason)
        if self.clock_verified and event['time_valid']:
            event['measurement_age_ms'] = event['candidate_age_ms']
            self.ages[event['family']].append(event['measurement_age_ms'])
        else:
            event['measurement_age_ms'] = None

    def observe(self, sentence, read_ros_ns, read_mono_ns, capture='serial_read_complete'):
        token = sentence.split(',', 1)[0]
        family = 'PVTSLNA' if token == '#PVTSLNA' else token[-3:] if token.startswith('$') else ''
        with self.lock:
            jumped = self._clock(read_ros_ns, read_mono_ns)
            if family not in self.ages or self.mode == 'host':
                return None
            self.sequence += 1
            event = dict(session_id=self.session_id, sequence=self.sequence, mode=self.mode,
                         clock_epoch=self.clock_epoch, clock_jump=jumped, family=family,
                         host_read_ros_ns=read_ros_ns, host_read_mono_ns=read_mono_ns,
                         read_capture=capture, measurement_stamp_ns=None, observation_id=None,
                         time_valid=False, time_source=None, clock_verified=self.clock_verified,
                         candidate_age_ms=None, measurement_age_ms=None, reason='unparsed',
                         epoch_order=None, raw_time_fields={}, published={})
            try:
                if family == 'PVTSLNA':
                    # Bounded header is diagnostic only; CRC must pass before use.
                    event['raw_time_fields']['header'] = sentence.split(';', 1)[0][:240]
                    stamp, raw = pvtslna_time(sentence)
                    event['raw_time_fields'].update(raw)
                    source = 'pvtslna_gpst_utc'
                else:
                    if not check_nmea_checksum(sentence):
                        raise ValueError('invalid_nmea_checksum')
                    fields = sentence.split('*', 1)[0].split(',')
                    event['raw_time_fields']['utc'] = fields[1][:32]
                    day_time = utc_day_time(fields[1])
                    if family == 'RMC':
                        date = fields[9]
                        event['raw_time_fields']['date'] = date[:16]
                        if fields[2] != 'A' or not re.fullmatch(r'\d{6}', date):
                            raise ValueError('invalid_rmc_date_or_status')
                        yy = int(date[4:])
                        year = 1900+yy if yy >= 80 else 2000+yy
                        date = dt.datetime(year, int(date[2:4]), int(date[:2]))
                        stamp = calendar.timegm(date.timetuple()) * NS + day_time
                        source = 'rmc_utc'
                    else:
                        if self.anchor is None or not 0 <= read_mono_ns-self.anchor[1] <= 60*NS:
                            raise ValueError('receiver_date_missing_or_stale')
                        day = self.anchor[0] // DAY_NS * DAY_NS
                        stamp = min((day+offset*DAY_NS+day_time for offset in (-1, 0, 1)),
                                    key=lambda value: abs(value-self.anchor[0]))
                        if abs(stamp-self.anchor[0]) > 60*NS:
                            raise ValueError('gga_date_reference_too_far')
                        source = 'gga_utc_receiver_date'
                previous = self.watermarks.get(family)
                order = 'new' if previous is None or stamp > previous else 'duplicate' if stamp == previous else 'out_of_order'
                candidate_age = (read_ros_ns-stamp)/1000000.
                admissible = (self.mode != 'measurement' or
                    -50 <= candidate_age <= (500 if family == 'PVTSLNA' else 1000))
                if not admissible:
                    order = 'rejected_age'
                event.update(measurement_stamp_ns=stamp, time_valid=True, time_source=source,
                             observation_id='{}:{}:{}:{}'.format(self.session_id, self.clock_epoch, family, stamp),
                             reason='ok', epoch_order=order,
                             candidate_age_ms=candidate_age)
                # Positive and negative differences are retained, never clamped
                # into a healthy age; no inferred clock correction is applied.
                if self.clock_verified and not self.auto_clock:
                    event['measurement_age_ms'] = event['candidate_age_ms']
                    self.ages[family].append(event['measurement_age_ms'])
                if order == 'new':
                    self.watermarks[family] = stamp
                if admissible and family != 'GGA' and (self.anchor is None or stamp > self.anchor[0]):
                    self.anchor = (stamp, read_mono_ns)
                self.counts[family+'_'+order] += 1
            except (ValueError, IndexError, OverflowError) as error:
                event['reason'] = str(error)[:120]
                self.counts[family+'_invalid_time'] += 1
            self._auto_verify(event)
            return event

    def summary(self):
        with self.lock:
            stats = {}
            for family, values in self.ages.items():
                samples = sorted(values)
                stats[family] = {'samples': len(samples), 'p50_ms': None, 'p95_ms': None,
                                 'p99_ms': None, 'max_ms': None}
                if samples:
                    for key, percentile in (('p50_ms', .50), ('p95_ms', .95), ('p99_ms', .99)):
                        stats[family][key] = samples[min(len(samples)-1, int((len(samples)-1)*percentile))]
                    stats[family]['max_ms'] = samples[-1]
            return dict(mode=self.mode, clock_epoch=self.clock_epoch, session_id=self.session_id,
                        clock_verified=self.clock_verified, clock_verification='auto' if self.auto_clock else 'explicit',
                        clock_gate_reason=self.auto_reason, host_clock_synchronized=self.host_clock_synced,
                        counts=dict(self.counts), measurement_age=stats)
