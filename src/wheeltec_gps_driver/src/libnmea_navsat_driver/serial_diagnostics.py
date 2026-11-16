# Module contract: Observe GNSS serial cadence, backlog, parsing, and capture health.
# Serial arrival gaps and receiver solution quality are different diagnostic dimensions.

"""Bounded, passive counters for the ONE serial reader; no ROS dependency."""

from collections import Counter
import hashlib
from pathlib import Path
import threading
import time
import uuid
import json


class BoundedRawCapture:
    """Optional tap on the existing reader, never another serial consumer."""

    # Configure the bounded raw serial capture destination and capture lifetime.
    # Inputs: directory, duration, clock, max_bytes.
    # Explicit rejection paths: ValueError.
    def __init__(
        self, directory, duration=0.0, clock=time.monotonic, max_bytes=1000000
    ):
        if duration and not 30.0 <= duration <= 60.0:
            raise ValueError('raw capture duration must be 30..60 seconds or zero')
        self.clock = clock
        self.end = clock() + duration
        self.file = None
        self.bytes = 0
        self.max_bytes = min(1000000, max_bytes)
        self.path = None
        self.error = None
        if duration:
            directory = Path(directory)
            directory.mkdir(parents=True, exist_ok=True)
            self.path = str(
                directory
                / (
                    'gnss_'
                    + time.strftime('%Y%m%d_%H%M%S')
                    + '_'
                    + uuid.uuid4().hex[:8]
                    + '.jsonl'
                )
            )
            self.file = open(self.path, 'x', encoding='utf-8')

    # Record the received serial evidence in this helper's diagnostic/capture state.
    # Inputs: raw, ros_ns (nanoseconds), mono_ns (nanoseconds), backlog.
    def observe(self, raw, ros_ns, mono_ns, backlog):
        if self.file is None:
            return
        if self.clock() >= self.end:
            self.close()
            return
        token = raw.split(b',', 1)[0]
        if token != b'#PVTSLNA' and not (
            token.startswith(b'$') and token[-3:] in (b'GGA', b'RMC')
        ):
            return
        record = (
            json.dumps(
                dict(
                    host_read_ros_ns=ros_ns,
                    host_read_mono_ns=mono_ns,
                    in_waiting_bytes=backlog,
                    raw=raw.decode('ascii', errors='backslashreplace'),
                ),
                ensure_ascii=True,
            )
            + '\n'
        )
        size = len(record.encode('utf-8'))
        if self.bytes + size > self.max_bytes:
            self.close()
            return
        try:
            self.file.write(record)
            self.bytes += size
        except OSError as error:
            self.error = str(error)[:200]
            self.close()

    # Close the owned raw-capture output when this helper is finished.
    def close(self):
        if self.file is not None:
            try:
                self.file.close()
            except OSError as error:
                self.error = str(error)[:200]
            self.file = None


# Accumulate serial I/O and parsed-message timing evidence without changing receiver
# configuration.
class SerialDiagnostics:
    COUNTERS = (
        'read_calls',
        'read_bytes',
        'empty_reads',
        'complete_lines',
        'partial_reads',
        'ascii_decoded',
        'ascii_decode_errors',
        'blank_lines',
        'raw_pvtslna_prefix',
        'embedded_pvtslna',
        'alternate_pvtsln',
        'parser_calls',
        'pvtslna_recognized',
        'pvtslna_parse_ok',
        'pvtslna_parse_errors',
        'pvtslna_crc_errors',
        'nmea_checksum_rejected',
        'unsupported_or_unparsed',
        'driver_value_errors',
        'unhandled_errors',
        'serial_io_errors',
        'diagnostic_publish_errors',
        'solution_publish_calls',
        'solution_publish_errors',
        'heading_publish_calls',
        'heading_publish_errors',
        'fix_publish_calls',
        'fix_publish_errors',
        'gga_publish_calls',
        'gga_publish_errors',
        'family_GGA',
        'family_GST',
        'family_RMC',
        'family_HDT',
        'family_HPR',
        'family_VTG',
        'family_GSV',
        'family_PVTSLNA',
        'family_PVTSLN',
        'family_BESTNAV',
        'family_KSXT',
        'family_AGRIC',
        'family_OTHER',
    )

    # Configure serial diagnostic sampling/capture and initialize cadence, backlog,
    # processing, and error counters.
    # Inputs: metadata, clock.
    def __init__(self, metadata=None, clock=time.monotonic):
        self.lock = threading.Lock()
        self.clock = clock
        self.started = self.window_start = clock()
        self.total = Counter({k: 0 for k in self.COUNTERS})
        self.window = self.total.copy()
        self.metadata = dict(metadata or {})
        self.session_id = uuid.uuid4().hex
        self.sequence = 0
        self.state = 'opening'
        self.last_error = None
        self.last_pvtslna = None
        self.max_gap = self.window_max_gap = 0.0
        self.processing = dict(
            in_waiting_bytes=None,
            max_in_waiting_bytes=0,
            last_read_bytes=0,
            last_parse_publish_ms=None,
            max_parse_publish_ms=0.0,
        )

    # Record message-processing latency separately from receiver source cadence.
    # Inputs: raw_bytes, elapsed_ms (milliseconds).
    def observe_processing(self, raw_bytes, elapsed_ms):
        with self.lock:
            self.processing.update(
                last_read_bytes=raw_bytes, last_parse_publish_ms=elapsed_ms
            )
            self.processing['max_parse_publish_ms'] = max(
                self.processing['max_parse_publish_ms'], elapsed_ms
            )

    # Record the observed serial backlog for diagnosing delayed host consumption.
    # Inputs: queued.
    def observe_backlog(self, queued):
        with self.lock:
            self.processing['in_waiting_bytes'] = queued
            self.processing['max_in_waiting_bytes'] = max(
                self.processing['max_in_waiting_bytes'], queued
            )

    # Increment the named diagnostic counter.
    # Inputs: key, amount.
    # Explicit rejection paths: ValueError.
    def count(self, key, amount=1):
        if key not in self.COUNTERS:
            raise ValueError('unknown diagnostic counter: ' + key)
        with self.lock:
            self.total[key] += amount
            self.window[key] += amount

    # Record an observed serial/parsing error in diagnostic state.
    # Inputs: key, message.
    def error(self, key, message):
        self.count(key)
        with self.lock:
            self.last_error = {'counter': key, 'detail': str(message)[:200]}

    # Update serial diagnostic state and its reported context.
    # Inputs: state.
    def set_state(self, state):
        with self.lock:
            self.state = state

    # Record one serial-read event and its timing/size evidence.
    # Inputs: raw.
    def observe_read(self, raw):
        self.count('read_calls')
        self.count('read_bytes', len(raw))  # BEFORE strip/decode, including CR/LF.
        if not raw:
            self.count('empty_reads')
            return
        self.count('complete_lines' if raw.endswith(b'\n') else 'partial_reads')
        line = raw.strip()
        if not line:
            self.count('blank_lines')
            return
        if line.startswith(b'#PVTSLNA,'):
            self.count('raw_pvtslna_prefix')
        elif b'#PVTSLNA,' in line:
            self.count('embedded_pvtslna')
        token = line.split(b',', 1)[0].split(b';', 1)[0][:32]
        if token.startswith(b'$') and len(token) == 6:
            family = token[-3:].decode('ascii', errors='replace')
        elif token.startswith(b'#'):
            family = token[1:].decode('ascii', errors='replace')
            if family in ('BESTNAVA', 'AGRICA'):
                family = family[:-1]
        else:
            family = 'OTHER'
        key = 'family_' + family
        self.count(key if key in self.COUNTERS else 'family_OTHER')
        if family == 'PVTSLN':
            self.count('alternate_pvtsln')

    # Record recognition of a supported receiver solution message.
    def recognized_pvtslna(self):
        self.count('pvtslna_recognized')
        now = self.clock()
        with self.lock:
            if self.last_pvtslna is not None:
                gap = max(0.0, now - self.last_pvtslna)
                self.max_gap = max(self.max_gap, gap)
                self.window_max_gap = max(self.window_max_gap, gap)
            self.last_pvtslna = now

    # Return accumulated serial diagnostic evidence for status publication.
    def snapshot(self):
        now = self.clock()
        with self.lock:
            elapsed = max(0.0, now - self.window_start)
            self.sequence += 1
            report = {
                'schema_version': 1,
                'session_id': self.session_id,
                'sequence': self.sequence,
                'state': self.state,
                'uptime_sec': max(0.0, now - self.started),
                'window_sec': elapsed,
                'metadata': self.metadata,
                'total': dict(self.total),
                'window': dict(self.window),
                'rates_per_sec': {
                    k: v / elapsed if elapsed > 0 else 0.0
                    for k, v in self.window.items()
                },
                'pvtslna_receive_age_sec': (
                    None
                    if self.last_pvtslna is None
                    else max(0.0, now - self.last_pvtslna)
                ),
                'pvtslna_max_receive_gap_sec': self.max_gap,
                'pvtslna_window_max_receive_gap_sec': self.window_max_gap,
                'last_error': self.last_error,
                'processing': dict(self.processing),
                'semantics': 'Host-read/parse/publish-call counters, not GNSS measurement rate, wire loss or ROS delivery acknowledgements.',
            }
            self.window = Counter({k: 0 for k in self.COUNTERS})
            self.window_start = now
            self.window_max_gap = 0.0
            return report


def source_identity(path):
    """Hash the actually imported file once, never invoke git or expose env."""
    resolved = Path(path).resolve()
    try:
        digest = hashlib.sha256(resolved.read_bytes()).hexdigest()
    except OSError as exc:
        return {'path': str(resolved), 'error': str(exc)[:200]}
    return {'path': str(resolved), 'sha256': digest}
