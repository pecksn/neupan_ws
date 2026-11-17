"""Host deployment policy and bounded holdover; never contact an NTP server."""

import configparser
from pathlib import Path
import sys
import unittest
import json
import tempfile
from test_p0_clock_acquisition import collect_recorded

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/wheeltec_gps_driver/src'))
from libnmea_navsat_driver.host_utc_evidence import retained_sample


class NtpDeploymentTests(unittest.TestCase):
    def test_live_discarded_outlier_does_not_replace_accepted_error_bound(self):
        fixture = json.loads(
            (
                Path(__file__).parent / 'fixtures/ntp_discarded_outlier_20261117.json'
            ).read_text()
        )
        accepted, ignored = fixture['accepted'], fixture['ignored']
        self.assertIn('Ignored=yes', ignored['properties'])
        self.assertIn('+1.116893s', ignored['status'])
        with tempfile.TemporaryDirectory() as d:
            history = Path(d) / 'accepted.json'
            self.assertTrue(collect_recorded(accepted, history)['qualified'])
            before = history.read_bytes()
            held = collect_recorded(ignored, history)
            self.assertTrue(held['qualified'], held)
            self.assertEqual(held['evidence_basis'], 'last_accepted_sample')
            self.assertAlmostEqual(held['offset_sec'], 0.074767)
            self.assertEqual(history.read_bytes(), before)
            self.assertFalse(
                collect_recorded(ignored, Path(d) / 'absent.json')['qualified']
            )

    def test_source_and_polling_match_the_documented_host_policy(self):
        cfg = configparser.ConfigParser(strict=False)
        cfg.read(ROOT / 'src/robot_base_driver/config/60-neupan-ntp.conf')
        self.assertEqual(
            cfg['Time']['NTP'].split(), ['ntp.tencent.com', 'ntp1.aliyun.com']
        )
        self.assertEqual(cfg['Time']['PollIntervalMinSec'], '32s')
        self.assertEqual(cfg['Time']['PollIntervalMaxSec'], '64s')

    def test_one_filtered_response_fits_existing_not_extended_sample_lifetime(self):
        for poll, next_accepted in ((32.0, 96.0), (64.0, 128.0)):
            lifetime = min(3600.0, max(90.0, 2 * poll + 60.0))
            previous = dict(
                qualified=True,
                utc_ns=1_000_000_000_000,
                monotonic_ns=100_000_000_000,
                boot_id='same_boot',
                server='118.31.3.89',
                sample_age_sec=0.0,
                max_sample_age_sec=lifetime,
                offset_sec=0.008,
                root_distance_sec=0.013,
            )
            for elapsed in (poll, next_accepted):
                current = dict(
                    utc_ns=previous['utc_ns'] + int(elapsed * 1e9),
                    monotonic_ns=previous['monotonic_ns'] + int(elapsed * 1e9),
                    boot_id='same_boot',
                    server=previous['server'],
                    properties='ignored',
                    status='synced',
                )
                held, reason = retained_sample(current, previous)
                self.assertIsNone(reason)
                self.assertTrue(held['qualified'])
                self.assertEqual(held['sample_age_sec'], elapsed)
                self.assertEqual(held['max_sample_age_sec'], lifetime)
            expired = dict(
                current,
                utc_ns=previous['utc_ns'] + int((lifetime + 1) * 1e9),
                monotonic_ns=previous['monotonic_ns'] + int((lifetime + 1) * 1e9),
            )
            self.assertEqual(
                retained_sample(expired, previous)[1], 'host_utc_history_expired'
            )

    def test_filtered_new_source_cannot_borrow_old_source_qualification(self):
        previous = dict(
            qualified=True,
            utc_ns=100_000_000_000,
            monotonic_ns=50_000_000_000,
            boot_id='boot',
            server='old',
            sample_age_sec=1.0,
            max_sample_age_sec=188.0,
            offset_sec=0.001,
            root_distance_sec=0.001,
        )
        current = dict(
            utc_ns=101_000_000_000,
            monotonic_ns=51_000_000_000,
            boot_id='boot',
            server='new',
            properties='ignored',
            status='synced',
        )
        self.assertEqual(
            retained_sample(current, previous)[1], 'host_utc_history_source_changed'
        )
