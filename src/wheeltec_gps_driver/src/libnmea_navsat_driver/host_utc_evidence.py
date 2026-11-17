# Module contract: Collect bounded host UTC synchronization evidence for GNSS
# source-clock admission. Host time display alone does not prove NTP/UTC
# synchronization.

"""Read independent host NTP evidence. Never adjust either clock."""

import argparse
import datetime
import ipaddress
import json
import math
import os
from pathlib import Path
import re
import subprocess
import time
import tempfile
import threading


class HostUtcMonitor:
    """Bounded background refresh; serial callbacks never wait for SSH/NTP."""

    def __init__(self, target, refresh_fn=None):
        self.target = target
        self.refresh_fn = refresh_fn or refresh
        self.stop = threading.Event()
        self.latest = {'qualified': False, 'error': 'host_utc_query_pending'}
        self.worker = threading.Thread(target=self.run, daemon=True)

    def run(self):
        while not self.stop.is_set():
            try:
                self.latest = self.refresh_fn(self.target)
            except Exception as error:
                self.latest = {'qualified': False, 'error': str(error)[:200]}
            self.stop.wait(5.0)

    def close(self):
        self.stop.set()
        # Do not turn process shutdown into a wait for a network request.
        # Queries already have an external timeout; the worker is a daemon.


def retained_sample(current, previous):
    """Hold only qualified accepted evidence; never refresh its sample epoch.

    The snapshot is freshly queried on every invocation. Its accepted-sample
    age advances by monotonic elapsed time even if the latest packet is ignored.
    20 ppm drift growth and the original offset bound limit holdover uncertainty.
    """
    try:
        elapsed = (current['monotonic_ns'] - previous['monotonic_ns']) / 1e9
        wall = (current['utc_ns'] - previous['utc_ns']) / 1e9
        age = previous['sample_age_sec'] + elapsed
        bound = (
            abs(previous['offset_sec']) + previous['root_distance_sec'] + age * 20e-6
        )
        if previous.get('qualified') is not True:
            return None, 'no_accepted_host_utc_history'
        if current['boot_id'] != previous['boot_id']:
            return None, 'host_utc_history_boot_changed'
        if current.get('server') != previous.get('server'):
            return None, 'host_utc_history_source_changed'
        if elapsed < 0 or not math.isfinite(wall) or abs(wall - elapsed) >= 0.1:
            return None, 'host_utc_clock_discontinuous'
        if not 0 <= age <= previous['max_sample_age_sec']:
            return None, 'host_utc_history_expired'
        if not math.isfinite(bound) or bound > 0.1:
            return None, 'host_utc_history_error_bound_exceeded'
        held = dict(previous)
        held.update(
            utc_ns=current['utc_ns'],
            monotonic_ns=current['monotonic_ns'],
            sample_age_sec=age,
            qualified=True,
            evidence_basis='last_accepted_sample',
            error_bound_sec=bound,
            latest_sample_ignored=True,
            latest_properties=current['properties'],
            latest_status=current['status'],
            error=None,
        )
        return held, None
    except (KeyError, TypeError, ValueError, OverflowError):
        return None, 'no_usable_host_utc_history'


# Query the supported host time-synchronization source and retain the returned evidence.
# Inputs: *args.
def query(*args):
    return subprocess.check_output(
        args,
        text=True,
        timeout=12 if args[0] == 'ssh' else 3,
        stderr=subprocess.PIPE,
        env=dict(os.environ, LC_ALL='C'),
    )


# Persist the collected UTC evidence through the existing evidence-file path.
# Inputs: evidence, target.
def save(evidence, target):
    target = Path(target)
    target.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode='w', dir=target.parent, delete=False
    ) as stream:
        temporary = Path(stream.name)
        try:
            json.dump(evidence, stream, ensure_ascii=False, allow_nan=False)
            stream.flush()
            temporary.replace(target)
        finally:
            if temporary.exists():
                temporary.unlink()


def refresh(target):
    """One bounded, pinned, loopback-only query per startup/retry, no old fallback."""
    channel = Path(__file__).resolve().parents[4] / '.runtime_diagnostics/host_utc_ssh'
    try:
        data = json.loads(
            query(
                'ssh',
                '-T',
                '-F',
                '/dev/null',
                '-o',
                'BatchMode=yes',
                '-o',
                'ConnectTimeout=2',
                '-o',
                'ConnectionAttempts=1',
                '-o',
                'StrictHostKeyChecking=yes',
                '-o',
                'IdentitiesOnly=yes',
                '-o',
                'UserKnownHostsFile=' + str(channel / 'known_hosts'),
                '-i',
                str(channel / 'id_ed25519'),
                'nuc15@127.0.0.1',
            )
        )
        if not isinstance(data, dict):
            raise ValueError('host evidence must be an object')
        if not validate(data):
            data.update(
                qualified=False,
                error=data.get('error') or validation_error(data),
            )
    except (OSError, ValueError, TypeError, subprocess.SubprocessError) as error:
        data = dict(
            qualified=False,
            error=(str(error) + ' ' + str(getattr(error, 'stderr', '') or ''))[:500],
        )
    data['transport'] = 'pinned_loopback_ssh'
    try:
        save(data, target)
    except (OSError, ValueError) as error:
        data.update(
            qualified=False, error='cannot save startup evidence: ' + str(error)[:200]
        )
    return data


# Convert the supplied time quantity into seconds for the evidence representation.
# Inputs: text.
# Explicit rejection paths: ValueError.
def seconds(text):
    units = {'s': 1.0, 'ms': 0.001, 'us': 1.0e-6, 'min': 60.0, 'h': 3600.0}
    parts = re.findall(r'([+-]?\d+(?:\.\d+)?)\s*(min|ms|us|s|h)', text)
    if not parts:
        raise ValueError('unsupported time value: ' + text)
    return sum(float(value) * units[unit] for value, unit in parts)


def validation_error(evidence, max_capture_age=120.0):
    """Return the actual rejection reason; None means the snapshot is usable."""
    try:
        age = (time.monotonic_ns() - evidence['monotonic_ns']) * 1.0e-9
        elapsed = (time.time_ns() - evidence['utc_ns']) * 1.0e-9
        if evidence['qualified'] is not True:
            return evidence.get('error') or 'host_utc_not_qualified'
        if not 0 <= age <= max_capture_age:
            return 'host_utc_snapshot_expired'
        if not abs(elapsed - age) < 0.1:
            return 'host_utc_clock_discontinuous'
        if not 0 <= evidence['sample_age_sec'] + age <= evidence['max_sample_age_sec']:
            return 'host_utc_sample_expired'
        if (
            evidence['boot_id']
            != Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        ):
            return 'host_utc_boot_changed'
        return None
    except (KeyError, TypeError, ValueError, OSError, OverflowError):
        return 'host_utc_evidence_invalid'


def validate(evidence, max_capture_age=120.0):
    """Fail closed on stale snapshot, reboot, clock step or unqualified NTP."""
    return validation_error(evidence, max_capture_age) is None


# Collect host UTC verification evidence and its supporting status fields.
def collect(history_file=None):
    evidence = dict(
        utc_ns=time.time_ns(),
        monotonic_ns=time.monotonic_ns(),
        boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        qualified=False,
        scope='Independent host NTP snapshot; receiver FINE GPST/NMEA consistency remains separately required',
    )
    try:
        sync = query('timedatectl', 'show', '-p', 'NTPSynchronized', '--value').strip()
        properties = query('timedatectl', 'show-timesync', '--all')
        status = query('timedatectl', 'timesync-status')
        evidence.update(ntp_synchronized=sync, properties=properties, status=status)
        props = dict(
            line.split('=', 1) for line in properties.splitlines() if '=' in line
        )
        message = props.get('NTPMessage', '')
        server = ipaddress.ip_address(props['ServerAddress'])
        stamp = re.search(
            r'DestinationTimestamp=(\w+ \d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2})', message
        ).group(1)
        sampled = (
            datetime.datetime.strptime(stamp, '%a %Y-%m-%d %H:%M:%S')
            .astimezone()
            .timestamp()
        )
        offset = seconds(re.search(r'^\s*Offset:\s*(.+)$', status, re.M).group(1))
        delay = seconds(re.search(r'^\s*Delay:\s*(.+)$', status, re.M).group(1))
        root = seconds(
            re.search(r'^\s*Root distance:\s*(.+?)\s*\(', status, re.M).group(1)
        )
        age = time.time() - sampled
        max_age = min(3600.0, max(90.0, 2 * seconds(props['PollIntervalUSec']) + 60.0))
        stratum = int(re.search(r'Stratum=(\d+)', message).group(1))
        qualified = (
            sync == 'yes'
            and not server.is_loopback
            and not server.is_unspecified
            and 'Leap=0,' in message
            and 'Mode=4,' in message
            and 'Ignored=no' in message
            and 1 <= stratum <= 15
            and abs(offset) <= 0.1
            and 0 <= delay <= 1.0
            and 0 <= root <= 0.25
            and 0 <= age <= max_age
            and all(math.isfinite(v) for v in (offset, delay, root, age))
        )
        evidence.update(
            qualified=qualified,
            server=str(server),
            offset_sec=offset,
            delay_sec=delay,
            root_distance_sec=root,
            sample_age_sec=age,
            max_sample_age_sec=max_age,
        )
        evidence['evidence_basis'] = 'latest_accepted_sample'
        evidence['sample_identity'] = str(server) + ':' + stamp
        previous = None
        if history_file:
            try:
                previous = json.loads(Path(history_file).read_text())
            except (OSError, ValueError):
                pass
        # A filtered response is not an accepted synchronization measurement.
        # Fresh accepted samples must pass every numeric gate. Filtered samples
        # may only retain prior accepted quality under the bounded history policy.
        numeric_ok = (
            sync == 'yes'
            and not server.is_loopback
            and not server.is_unspecified
            and 'Leap=0,' in message
            and 'Mode=4,' in message
            and 1 <= stratum <= 15
            and abs(offset) <= 0.1
            and 0 <= delay <= 1
            and 0 <= root <= 0.25
            and 0 <= age <= max_age
            and all(math.isfinite(v) for v in (offset, delay, root, age))
        )
        if previous and previous.get('boot_id') == evidence['boot_id']:
            elapsed = (evidence['monotonic_ns'] - previous['monotonic_ns']) / 1e9
            wall = (evidence['utc_ns'] - previous['utc_ns']) / 1e9
            stepped = elapsed < 0 or abs(wall - elapsed) >= 0.1
            invalidated_same = (
                previous.get('invalidated_sample') == evidence['sample_identity']
            )
            if stepped or invalidated_same:
                evidence.update(
                    qualified=False,
                    error='host_utc_clock_discontinuous',
                    invalidated_sample=evidence['sample_identity'],
                )
                if history_file:
                    save(evidence, history_file)
                return evidence
        service_ok = (
            sync == 'yes'
            and not server.is_loopback
            and not server.is_unspecified
            and 'Leap=0,' in message
            and 'Mode=4,' in message
            and 1 <= stratum <= 15
        )
        if not service_ok:
            evidence.update(
                qualified=False, error='host_utc_current_quality_or_sync_invalid'
            )
        elif 'Ignored=yes' in message:
            # Offset and delay of a discarded response describe that response,
            # not an applied correction to CLOCK_REALTIME. Evaluate holdover
            # using the last accepted quality and UTC/monotonic continuity;
            # never copy discarded quality into the accepted-history record.
            held, reason = retained_sample(evidence, previous or {})
            if held is None:
                evidence.update(
                    qualified=False, error=reason, latest_sample_ignored=True
                )
            else:
                evidence = held
        elif not numeric_ok:
            evidence.update(
                qualified=False, error='host_utc_current_quality_or_sync_invalid'
            )
        elif not qualified:
            evidence['error'] = 'host_utc_latest_sample_not_accepted'
        if (
            evidence['qualified']
            and evidence['evidence_basis'] == 'latest_accepted_sample'
            and history_file
        ):
            save(evidence, history_file)
    except (
        OSError,
        ValueError,
        KeyError,
        AttributeError,
        subprocess.SubprocessError,
    ) as error:
        evidence['error'] = str(error)[:300]
    return evidence


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--output')
    mode.add_argument('--stdout-only', action='store_true')
    parser.add_argument('--require-valid', action='store_true')
    args = parser.parse_args()
    # Host-owned cache is updated only by fresh accepted samples, not by an
    # ignored response, container startup, or a previously qualified snapshot.
    history = (
        Path(__file__).resolve().parents[4]
        / '.runtime_diagnostics/host_utc_last_accepted.json'
    )
    data = collect(history)
    if args.output:
        save(data, args.output)
    print(json.dumps(data, ensure_ascii=False))
    if args.require_valid and not validate(data):
        raise SystemExit(
            'Host UTC evidence unavailable/unqualified: keep transport candidate disabled'
        )
