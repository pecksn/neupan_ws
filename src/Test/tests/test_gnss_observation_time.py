"""GNSS timing shadow regressions: fake clocks, no master or hardware."""
import calendar
import datetime as dt
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'src/wheeltec_gps_driver/src'))
from libnmea_navsat_driver.observation_time import NS, GPS_EPOCH, ObservationTiming, utc_day_time
from libnmea_navsat_driver.unicore import crc32
from test_unicore_solution import solution


def nmea(body):
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    return '${}*{:02X}'.format(body, checksum)


def pvts(stamp, state='FINE', reference='GPS', leap=18, heading='NARROW_INT', position='SINGLE'):
    milliseconds = ((stamp // 1000000) - (GPS_EPOCH-leap)*1000)
    week, ms = divmod(milliseconds, 604800000)
    payload = solution(heading=heading, position=position).split(';')[1].split('*')[0]
    body = 'PVTSLNA,95,{},{},{},{},0,0,{},2;{}'.format(reference, state, week, ms, leap, payload)
    return '#{}*{:08x}'.format(body, crc32(body))


def stamp(year=2026, month=10, day=21, hour=0, minute=0, second=0):
    return calendar.timegm((year, month, day, hour, minute, second)) * NS


class TimingTests(unittest.TestCase):
    def setUp(self):
        self.time = ObservationTiming(clock_verified=True)
        self.epoch = stamp()

    def observe(self, sentence, offset=0, mono=0):
        return self.time.observe(sentence, self.epoch+offset, mono)

    def test_integer_fraction_no_float_roundtrip(self):
        self.assertEqual(utc_day_time('000000.000000001'), 1)
        self.assertEqual(utc_day_time('235959.123456789'), 86399*NS+123456789)
        for bad in ('', '1200', '240000', '126000', '235960', '120000.1234567890', '12nan0'):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                utc_day_time(bad)

    def test_receiver_week_milliseconds_and_dynamic_leap(self):
        for leap in (17, 18, 19):
            result = self.observe(pvts(self.epoch+123000000, leap=leap))
            self.assertEqual(result['measurement_stamp_ns'], self.epoch+123000000)
            self.assertEqual(result['raw_time_fields']['leap_seconds'], str(leap))

    def test_delay_does_not_change_measurement_epoch(self):
        for delay in (0, 50, 120, 200):
            monitor = ObservationTiming(clock_verified=True)
            event = monitor.observe(pvts(self.epoch), self.epoch+delay*1000000, 0)
            self.assertEqual(event['measurement_stamp_ns'], self.epoch)
            self.assertEqual(event['measurement_age_ms'], delay)

    def test_unverified_clock_never_claims_transport_latency(self):
        event = ObservationTiming().observe(pvts(self.epoch), self.epoch+200*NS, 0)
        self.assertTrue(event['time_valid'])
        self.assertEqual(event['candidate_age_ms'], 200000)
        self.assertIsNone(event['measurement_age_ms'])

    def test_heading_float_does_not_invalidate_measurement_time(self):
        self.assertTrue(self.observe(pvts(self.epoch, heading='NARROW_FLOAT'))['time_valid'])

    def test_bad_status_system_crc_and_range_fail_closed(self):
        for text in (pvts(self.epoch, state='UNKNOWN'), pvts(self.epoch, reference='BDST'),
                     pvts(self.epoch)[:-8]+'00000000', '#PVTSLNA,cut', pvts(self.epoch, leap=100)):
            with self.subTest(text=text):
                result = self.observe(text)
                self.assertFalse(result['time_valid'])
                self.assertIsNone(result['measurement_stamp_ns'])
                self.assertIsNone(result['observation_id'])
        self.assertEqual(self.time.watermarks, {})

    def test_missing_gga_date_never_borrows_host_date(self):
        event = self.observe(nmea('GNGGA,000000.1'))
        self.assertFalse(event['time_valid'])
        self.assertEqual(event['reason'], 'receiver_date_missing_or_stale')

    def test_late_date_can_validate_epoch_without_invalid_packet_advancing_watermark(self):
        text = nmea('GNGGA,000000.00')
        self.assertFalse(self.observe(text)['time_valid'])
        self.assertNotIn('GGA', self.time.watermarks)
        self.observe(pvts(self.epoch))
        self.assertEqual(self.observe(text)['epoch_order'], 'new')

    def test_future_and_expired_age_are_not_rewritten_as_fresh(self):
        future = self.observe(pvts(self.epoch+NS))
        self.assertEqual(future['measurement_age_ms'], -1000)
        stale = self.observe(pvts(self.epoch-NS))
        self.assertEqual(stale['measurement_age_ms'], 1000)

    def test_rmc_supplies_date_without_switching_position_source(self):
        self.assertTrue(self.observe(nmea('GNRMC,000000.00,A,,,,,,,211026'))['time_valid'])
        event = self.observe(nmea('GNGGA,000000.123456789'))
        self.assertEqual(event['measurement_stamp_ns'], self.epoch+123456789)
        self.assertEqual(event['time_source'], 'gga_utc_receiver_date')

    def test_midnight_month_year_and_delayed_previous_day(self):
        for year, month, day in ((2026, 10, 31), (2026, 12, 31), (2028, 2, 29)):
            epoch = stamp(year, month, day, 23, 59, 59)
            monitor = ObservationTiming()
            monitor.observe(pvts(epoch), epoch, 0)
            event = monitor.observe(nmea('GNGGA,000000.10'), epoch+NS, NS)
            self.assertEqual(event['measurement_stamp_ns'], epoch+1100000000)
            # A previous-day packet arriving after the receiver date advances.
            monitor.observe(pvts(epoch+NS), epoch+NS, NS)
            event = monitor.observe(nmea('GNGGA,235959.90'), epoch+NS, NS)
            self.assertEqual(event['measurement_stamp_ns'], epoch+900000000)

    def test_gps_week_rollover(self):
        epoch = (GPS_EPOCH+2400*604800-18)*NS
        monitor = ObservationTiming()
        before = monitor.observe(pvts(epoch-1000000), epoch, 0)
        after = monitor.observe(pvts(epoch), epoch, 0)
        self.assertEqual(after['measurement_stamp_ns']-before['measurement_stamp_ns'], 1000000)

    def test_duplicate_identity_and_out_of_order_do_not_advance_watermark(self):
        first = self.observe(pvts(self.epoch))
        duplicate = self.observe(pvts(self.epoch), 100000000, 100000000)
        self.assertEqual(first['observation_id'], duplicate['observation_id'])
        self.assertEqual(duplicate['epoch_order'], 'duplicate')
        self.assertEqual(duplicate['measurement_age_ms'], 100)
        old = self.observe(pvts(self.epoch-NS), 200000000, 200000000)
        self.assertEqual(old['epoch_order'], 'out_of_order')
        self.assertEqual(self.time.watermarks['PVTSLNA'], self.epoch)
        self.assertEqual(self.time.anchor[0], self.epoch)

    def test_clock_step_new_epoch_invalidates_verification_and_date(self):
        first = self.observe(pvts(self.epoch))
        event = self.observe(nmea('GNGGA,000000.10'), 10*NS, NS)
        self.assertTrue(event['clock_jump'])
        self.assertEqual(event['clock_epoch'], first['clock_epoch']+1)
        self.assertFalse(event['clock_verified'])
        self.assertFalse(event['time_valid'])
        self.assertEqual(self.time.summary()['measurement_age']['PVTSLNA']['samples'], 0)

    def test_stale_date_and_remote_gga_time_rejected(self):
        self.observe(pvts(self.epoch))
        event = self.observe(nmea('GNGGA,120000.00'))
        self.assertFalse(event['time_valid'])
        event = self.observe(nmea('GNGGA,000101.00'), 61*NS, 61*NS)
        self.assertFalse(event['time_valid'])

    def test_bad_rmc_or_checksum_cannot_poison_anchor(self):
        self.observe(pvts(self.epoch))
        for text in (nmea('GNRMC,000000,V,,,,,,,221026'),
                     nmea('GNRMC,000000,A,,,,,,,321026'), nmea('GNGGA,000001')[:-2]+'XX'):
            self.assertFalse(self.observe(text)['time_valid'])
            self.assertEqual(self.time.anchor[0], self.epoch)

    def test_bounded_percentiles_and_strict_json(self):
        for i in range(600):
            self.observe(pvts(self.epoch+i*1000000), i*1000000+120000000, i*1000000+120000000)
        result = self.time.summary()
        self.assertEqual(result['measurement_age']['PVTSLNA']['samples'], 512)
        self.assertEqual(result['measurement_age']['PVTSLNA']['p95_ms'], 120)
        json.dumps(result, allow_nan=False)

    def test_host_mode_disables_stream_and_measurement_requires_explicit_verification(self):
        self.assertIsNone(ObservationTiming('host').observe(pvts(self.epoch), self.epoch, 0))
        with self.assertRaises(ValueError):
            ObservationTiming('measurement')
        self.assertEqual(ObservationTiming('measurement', True).mode, 'measurement')


try:
    import rospy
    from libnmea_navsat_driver.driver import RosNMEADriver
    ROS = True
except ImportError:
    ROS = False


@unittest.skipUnless(ROS, 'requires ROS1 imports, not a running master')
class TimingPipelineTests(unittest.TestCase):
    def setUp(self):
        self.params = {}
        self.now = rospy.Time(1792540800)
        for stub in (patch.object(rospy, 'Publisher', side_effect=lambda *a, **k: Mock()),
                     patch.object(rospy, 'get_param', lambda k, d=None: self.params.get(k, d)),
                     patch.object(rospy.Time, 'now', lambda: self.now),
                     patch.object(rospy, 'get_rostime', lambda: self.now),
                     patch.object(rospy, 'logwarn_throttle')):
            stub.start()
            self.addCleanup(stub.stop)

    def test_shadow_keeps_fix_gga_heading_solution_original_time_contract(self):
        driver = RosNMEADriver()
        original = self.now.to_nsec()
        text = pvts(original-120000000)
        driver.add_sentence(text, 'gps', host_read_ros_ns=original, host_read_mono_ns=0)
        event = json.loads(driver.time_diagnostics_pub.publish.call_args[0][0].data)
        self.assertEqual(event['measurement_stamp_ns'], original-120000000)
        self.assertEqual(driver.heading_pub.publish.call_args[0][0].header.stamp, self.now)
        state = json.loads(driver.solution_pub.publish.call_args[0][0].data)
        self.assertEqual(state['stamp_ns'], original)
        driver.add_sentence(nmea('GNGGA,000000.00,2302.4,N,11323.4,E,4,12,1.0,5,M,0,M,,0000'), 'gps')
        self.assertEqual(driver.fix_pub.publish.call_args[0][0].header.stamp, self.now)
        self.assertEqual(driver.gpgga_pub.publish.call_args[0][0].header.stamp, self.now)
        event = json.loads(driver.time_diagnostics_pub.publish.call_args[0][0].data)
        self.assertEqual(event['published']['fix']['source_stamp_ns'], original)
        self.assertTrue(event['published']['gga']['publish_call_ok'])

    def test_shadow_invalid_time_does_not_invent_heading_quality(self):
        driver = RosNMEADriver()
        driver.add_sentence(solution(heading='NARROW_FLOAT'), 'gps')
        self.assertFalse(driver.heading_pub.publish.called)
        event = json.loads(driver.time_diagnostics_pub.publish.call_args[0][0].data)
        self.assertFalse(event['time_valid'])
        self.assertEqual(event['reason'], 'receiver_time_not_fine')

    def test_diagnostic_publish_failure_does_not_change_driver_result(self):
        driver = RosNMEADriver()
        driver.time_diagnostics_pub.publish.side_effect = RuntimeError('test')
        self.assertTrue(driver.add_sentence(solution(), 'gps'))
        self.assertTrue(driver.heading_pub.publish.called)

    def test_diagnostic_parser_failure_is_not_a_navigation_failure(self):
        driver = RosNMEADriver()
        with patch.object(driver.observation_timing, 'observe', side_effect=RuntimeError('test')):
            self.assertTrue(driver.add_sentence(solution(), 'gps'))
        self.assertTrue(driver.heading_pub.publish.called)

    def test_unique_serial_reader_captures_before_diagnostic_or_parser_work(self):
        from libnmea_navsat_driver.nodes.nmea_serial_driver import process_serial_read
        driver, counters = RosNMEADriver(), Mock()
        read_ns = self.now.to_nsec()
        def delay(_):
            self.now = rospy.Time(self.now.secs, 120000000)
        counters.observe_read.side_effect = delay
        raw = pvts(read_ns).encode('ascii')+b'\r\n'
        process_serial_read(raw, driver, counters, 'gps')
        event = json.loads(driver.time_diagnostics_pub.publish.call_args[0][0].data)
        self.assertEqual(event['host_read_ros_ns'], read_ns)
        self.assertEqual(event['read_capture'], 'serial_read_complete')
        self.assertEqual(event['published']['heading']['publish_ros_ns'], read_ns+120000000)


if __name__ == '__main__':
    unittest.main()
