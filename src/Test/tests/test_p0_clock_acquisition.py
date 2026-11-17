"""P0 replay: real collector and serial entry, no ROS master or hardware."""

import ast
import json
import os
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/wheeltec_gps_driver/src'))
from libnmea_navsat_driver import host_utc_evidence as host
from libnmea_navsat_driver.serial_diagnostics import (
    SerialDiagnostics,
    BoundedRawCapture,
)
import test_gnss_auto_start as auto_start


def snapshot(suffix):
    rows = json.loads(
        (
            ROOT
            / (
                'tracker_logs/review_20261117/startup_afternoon/system_run_full_20261117_'
                + suffix
                + '_serial.json'
            )
        ).read_text()
    )
    return rows[0]['host_clock_evidence']['host_snapshot']


def collect_recorded(s, history):
    read = Path.read_text
    replies = {
        ('timedatectl', 'show', '-p', 'NTPSynchronized', '--value'): s[
            'ntp_synchronized'
        ],
        ('timedatectl', 'show-timesync', '--all'): s['properties'],
        ('timedatectl', 'timesync-status'): s['status'],
    }
    with patch.object(host, 'query', side_effect=lambda *a: replies[a]), patch.object(
        host.time, 'time', return_value=s['utc_ns'] / 1e9
    ), patch.object(host.time, 'time_ns', return_value=s['utc_ns']), patch.object(
        host.time, 'monotonic_ns', return_value=s['monotonic_ns']
    ), patch.object(
        host.Path,
        'read_text',
        autospec=True,
        side_effect=lambda p, **kw: (
            s['boot_id'] if str(p).startswith('/proc/') else read(p, **kw)
        ),
    ):
        return host.collect(history)


def serial_entry(evidence):
    source = (
        ROOT
        / 'src/wheeltec_gps_driver/src/libnmea_navsat_driver/nodes/nmea_serial_driver.py'
    )
    functions = [
        n for n in ast.parse(source.read_text()).body if isinstance(n, ast.FunctionDef)
    ]
    ros = Mock()
    ros.get_param.side_effect = lambda k, default=None: {
        '~gnss_common_utc_verified': True,
        '~host_clock_evidence_file': 'offline.json',
    }.get(k, default)
    ros.is_shutdown.side_effect = [False, True]
    ros.Time.now.return_value.to_sec.return_value = 1.0
    ros.ROSInterruptException = InterruptedError
    device = Mock(in_waiting=0)
    device.readline.return_value = b''
    driver = Mock()
    driver.observation_timing.summary.return_value = {}
    cls = Mock(return_value=driver)
    space = dict(
        rospy=ros,
        RosNMEADriver=cls,
        refresh_host_utc=lambda p: evidence,
        SerialDiagnostics=SerialDiagnostics,
        BoundedRawCapture=BoundedRawCapture,
        source_identity=lambda p: {},
        __file__=str(source),
        driver_module=NS(__file__='driver.py'),
        unicore=NS(__file__='unicore.py'),
        observation_time=NS(__file__='time.py', host_clock_evidence=lambda p: {}),
        time=time,
        os=os,
        sys=sys,
        json=json,
        String=lambda **kw: NS(**kw),
        serial=NS(Serial=Mock(return_value=device), SerialException=OSError),
    )
    exec(
        compile(ast.Module(body=functions, type_ignores=[]), str(source), 'exec'), space
    )
    space['main']()
    return space['serial'].Serial.call_count, device.readline.call_count


class P0ClockTests(unittest.TestCase):
    @unittest.skipUnless(auto_start.pipeline.ROS, 'requires ROS1, fake publishers only')
    def test_real_driver_reports_fixed_without_navigation_when_clock_denied(self):
        pipeline = auto_start.pipeline
        harness = pipeline.MeasurementPipelineTests()
        harness.setUp()
        try:
            harness.driver.observation_timing.independent_clock_probe = lambda: False
            harness.send(pipeline.pvts(harness.epoch, position='NARROW_INT'))
            harness.send(
                pipeline.nmea(
                    'GNGGA,000000.000,2302.4,N,11323.4,E,4,12,1.0,5,M,0,M,,0000'
                )
            )
            report = json.loads(
                harness.driver.solution_pub.publish.call_args[0][0].data
            )
            self.assertEqual(report['position_type'], 'NARROW_INT')
            self.assertTrue(report['receiver_heading_fixed'])
            self.assertFalse(report['clock_verified'])
            self.assertEqual(report['stamp_ns'], 0)
            self.assertFalse(harness.driver.heading_pub.publish.called)
            self.assertFalse(harness.driver.fix_pub.publish.called)
        finally:
            harness.doCleanups()

    def test_five_recorded_starts_receive_with_and_without_history(self):
        with tempfile.TemporaryDirectory() as d:
            history = Path(d) / 'accepted.json'
            rows = []
            for suffix in ('165544', '170220', '170338', '171541', '171715'):
                evidence = collect_recorded(snapshot(suffix), history)
                rows.append(evidence)
                # The last accepted 16:53 sample expires before the second
                # ignored startup. Do not extend its lifetime to force a pass.
                self.assertEqual(
                    evidence['qualified'], suffix != '170338', (suffix, evidence)
                )
                self.assertEqual(serial_entry(evidence), (1, 1))
            self.assertEqual(rows[1]['evidence_basis'], 'last_accepted_sample')
            self.assertEqual(rows[2]['error'], 'host_utc_history_expired')
            for suffix in ('170220', '170338'):
                missing = collect_recorded(
                    snapshot(suffix), Path(d) / 'no-history.json'
                )
                self.assertFalse(missing['qualified'])
                self.assertEqual(serial_entry(missing), (1, 1))

    def test_history_cannot_cross_expiry_boot_step_or_error_bound(self):
        good = dict(
            qualified=True,
            utc_ns=100_000_000_000,
            monotonic_ns=50_000_000_000,
            boot_id='boot',
            sample_age_sec=20.0,
            max_sample_age_sec=90.0,
            offset_sec=0.001,
            root_distance_sec=0.001,
        )
        now = dict(
            utc_ns=101_000_000_000,
            monotonic_ns=51_000_000_000,
            boot_id='boot',
            properties='ignored',
            status='sync',
        )
        self.assertTrue(host.retained_sample(now, good)[0]['qualified'])
        for change in (
            {'boot_id': 'other'},
            {'utc_ns': 102_000_000_000},
            {'monotonic_ns': 49_000_000_000},
        ):
            self.assertIsNone(host.retained_sample(dict(now, **change), good)[0])
        for change in (
            {'sample_age_sec': 91.0},
            {'offset_sec': 0.101},
            {'qualified': False},
        ):
            self.assertIsNone(host.retained_sample(now, dict(good, **change))[0])

    def test_query_in_flight_cannot_block_acquisition(self):
        import threading

        entered, release = threading.Event(), threading.Event()

        def slow(_):
            entered.set()
            release.wait(1.0)
            return {'qualified': False}

        monitor = host.HostUtcMonitor('unused', slow)
        monitor.worker.start()
        try:
            self.assertTrue(entered.wait(0.5))
            self.assertFalse(monitor.latest['qualified'])
            self.assertEqual(serial_entry({'qualified': False}), (1, 1))
        finally:
            monitor.close()
            release.set()
            monitor.worker.join(2.0)

    def test_current_unsync_or_bad_error_does_not_use_history(self):
        with tempfile.TemporaryDirectory() as d:
            history = Path(d) / 'accepted.json'
            collect_recorded(snapshot('165544'), history)
            ignored = snapshot('170220')
            before = history.read_bytes()
            for changed in (
                dict(ignored, ntp_synchronized='no'),
                dict(
                    ignored,
                    status=ignored['status'].replace('-19.240ms', '900ms'),
                    properties=ignored['properties'].replace(
                        'Ignored=yes', 'Ignored=no'
                    ),
                ),
            ):
                # The recorded text uses locale-independent units; explicitly
                # replace the complete offset line for the numerical rejection.
                if changed.get('ntp_synchronized') == 'yes':
                    import re

                    changed['status'] = re.sub(
                        r'(?m)^(\s*Offset:).+$', r'\1 900ms', changed['status']
                    )
                self.assertFalse(collect_recorded(changed, history)['qualified'])
            self.assertEqual(history.read_bytes(), before)

    def test_clock_step_requires_new_accepted_sample_not_requery_of_old_one(self):
        with tempfile.TemporaryDirectory() as d:
            history = Path(d) / 'accepted.json'
            s = snapshot('165544')
            self.assertTrue(collect_recorded(s, history)['qualified'])
            stepped = dict(
                s,
                utc_ns=s['utc_ns'] + 2_000_000_000,
                monotonic_ns=s['monotonic_ns'] + 1_000_000_000,
            )
            self.assertFalse(collect_recorded(stepped, history)['qualified'])
            self.assertFalse(collect_recorded(stepped, history)['qualified'])
            import datetime
            import re

            fresh = dict(
                stepped,
                utc_ns=stepped['utc_ns'] + 1_000_000_000,
                monotonic_ns=stepped['monotonic_ns'] + 1_000_000_000,
            )
            stamp = datetime.datetime.fromtimestamp(fresh['utc_ns'] / 1e9).strftime(
                '%a %Y-%m-%d %H:%M:%S'
            )
            fresh['properties'] = re.sub(
                r'DestinationTimestamp=\w+ \d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}',
                'DestinationTimestamp=' + stamp,
                fresh['properties'],
            )
            self.assertTrue(collect_recorded(fresh, history)['qualified'])

    def test_kernel_sync_alone_cannot_qualify_then_new_epochs_recover(self):
        harness = auto_start.AutoStartTests()
        harness.setUp()
        allowed = [False]
        harness.clock.independent_clock_probe = lambda: allowed[0]
        harness.ready()
        self.assertFalse(harness.clock.clock_verified)
        allowed[0] = True
        harness.ready(30)
        self.assertTrue(harness.clock.clock_verified)
        allowed[0] = False
        harness.sample(56)
        self.assertFalse(harness.clock.clock_verified)
        allowed[0] = True
        harness.ready(60)
        self.assertTrue(harness.clock.clock_verified)
