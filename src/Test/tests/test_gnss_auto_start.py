"""Automatic measurement startup: never asserts clocks valid before evidence."""
import unittest
from test_gnss_observation_time import NS, ObservationTiming, pvts, nmea, stamp
import test_gnss_measurement_contract as pipeline

GnssTimebase, ROS = pipeline.GnssTimebase, pipeline.ROS


def gga(epoch):
    seconds, fraction = divmod(epoch, NS)
    seconds %= 86400
    hour, seconds = divmod(seconds, 3600)
    minute, second = divmod(seconds, 60)
    return nmea('GNGGA,{:02d}{:02d}{:02d}.{:09d}'.format(hour, minute, second, fraction))


class AutoStartTests(unittest.TestCase):
    def setUp(self):
        self.synced = True
        self.clock = ObservationTiming('measurement', 'auto', lambda: self.synced)
        self.gate = GnssTimebase('measurement', 'auto')
        self.epoch = stamp()

    def sample(self, index, fine=True, source_shift=0):
        delta = index*100000000
        now, mono = self.epoch+delta, NS+delta
        source = now+source_shift
        events = [self.clock.observe(pvts(source-100000000, state='FINE' if fine else 'UNKNOWN'), now, mono),
                  self.clock.observe(gga(source-50000000) if fine else nmea('GNGGA,'), now, mono)]
        for event in events:
            self.gate.receive(event, now, mono)
        return events

    def ready(self, start=0):
        for index in range(start, start+25):
            self.sample(index)

    def test_unknown_time_starts_waiting_without_latching_and_becomes_ready_outdoors(self):
        for i in range(30):
            events = self.sample(i, fine=False)
            self.assertTrue(events[-1]['clock_waiting'])
            self.assertFalse(self.gate.clock_ready)
            self.assertIsNone(self.gate.fault)
        self.ready(30)
        self.assertTrue(self.clock.clock_verified)
        self.assertTrue(self.gate.clock_ready)
        self.assertIsNone(self.gate.fault)

    def test_single_fine_packet_is_not_enough_and_full_window_required(self):
        for i in range(20):
            self.sample(i)
            self.assertFalse(self.clock.clock_verified)
        self.sample(20)
        self.assertTrue(self.clock.clock_verified)

    def test_unsynchronized_nuc_never_qualifies(self):
        self.synced = False
        self.ready()
        self.assertFalse(self.clock.clock_verified)
        self.assertFalse(self.gate.clock_ready)
        self.assertIsNone(self.gate.fault)
        self.assertEqual(self.clock.auto_reason, 'waiting_for_nuc_clock_sync')

    def test_long_gap_resets_startup_window(self):
        for i in range(19):
            self.sample(i)
        self.sample(30)
        self.assertFalse(self.clock.clock_verified)
        self.assertEqual(len(self.clock.auto_samples['PVTSLNA']), 1)

    def test_duplicates_cannot_accumulate_qualification(self):
        for i in range(40):
            now, mono = self.epoch+i*10000000, NS+i*10000000
            self.clock.observe(pvts(self.epoch), now, mono)
            self.clock.observe(gga(self.epoch), now, mono)
        self.assertFalse(self.clock.clock_verified)
        self.assertEqual(len(self.clock.auto_samples['PVTSLNA']), 1)

    def test_clock_sync_loss_after_ready_latches_not_a_restartable_startup_wait(self):
        self.ready()
        self.synced = False
        self.sample(40)
        self.assertTrue(self.clock.auto_failed)
        self.assertIsNotNone(self.gate.fault)
        self.synced = True
        self.ready(41)
        self.assertFalse(self.clock.clock_verified)

    def test_normal_gnss_loss_after_ready_does_not_claim_host_clock_failure(self):
        self.ready()
        self.sample(25, fine=False)
        self.assertTrue(self.clock.clock_verified)
        self.assertIsNone(self.gate.fault)
        self.assertFalse(self.gate.latest['GGA']['time_valid'])

    def test_future_epoch_does_not_poison_later_valid_time_or_receiver_date(self):
        self.sample(0, source_shift=86400*NS)
        self.assertFalse(self.clock.clock_verified)
        self.assertNotIn('PVTSLNA', self.clock.watermarks)
        self.assertIsNone(self.clock.anchor)
        self.ready(1)
        self.assertTrue(self.clock.clock_verified)

    def test_clock_step_never_automatically_requalifies(self):
        self.ready()
        event = self.clock.observe(pvts(self.epoch+10*NS), self.epoch+10*NS, 4*NS)
        self.assertTrue(event['clock_jump'])
        self.assertTrue(self.clock.auto_failed)
        self.assertFalse(event['clock_waiting'])
        self.ready(31)
        self.assertFalse(self.clock.clock_verified)

    def test_initial_nuc_clock_adjustment_restarts_wait_not_permanent_fault(self):
        self.sample(0, fine=False)
        event = self.clock.observe(pvts(self.epoch+10*NS, state='UNKNOWN'),
                                   self.epoch+10*NS, 2*NS)
        self.gate.receive(event, self.epoch+10*NS, 2*NS)
        self.assertTrue(event['clock_waiting'])
        self.assertIsNone(self.gate.fault)
        # New ROS/monotonic baseline, still before any navigation initialization.
        for i in range(25):
            now, mono = self.epoch+10*NS+i*100000000, 2*NS+i*100000000
            for text in (pvts(now-100000000), gga(now-50000000)):
                event = self.clock.observe(text, now, mono)
                self.gate.receive(event, now, mono)
        self.assertTrue(self.gate.clock_ready)
        self.assertIsNone(self.gate.fault)


@unittest.skipUnless(ROS, 'requires ROS1 fake callbacks, no master or devices')
class DriverAutoStartTests(unittest.TestCase):
    fix = pipeline.MeasurementPipelineTests.fix
    send = pipeline.MeasurementPipelineTests.send

    def setUp(self):
        pipeline.MeasurementPipelineTests.setUp(self)
        self.driver.observation_timing = ObservationTiming('measurement', 'auto', lambda: True)
        self.b.gnss_timebase = GnssTimebase('measurement', 'auto')

    def test_indoor_wait_then_valid_outdoor_time_publishes_without_restart_or_param_change(self):
        from test_gnss_measurement_contract import rospy
        self.send(pvts(self.epoch, state='UNKNOWN'))
        self.send(nmea('GNGGA,,2302.4,N,11323.4,E,0,0,99,5,M,0,M,,0000'))
        self.assertFalse(self.driver.fix_pub.publish.called)
        self.assertFalse(self.driver.heading_pub.publish.called)
        self.assertIsNone(self.b.gnss_timebase.fault)
        self.assertFalse(self.b.gnss_timebase.clock_ready)
        for i in range(26):
            self.now = rospy.Time(self.epoch//NS, i*100000000)
            self.mono = NS+i*100000000
            self.send(pvts(self.now.to_nsec()-100000000))
            body = gga(self.now.to_nsec()-50000000).split('*')[0][1:]
            self.send(nmea(body+',2302.4,N,11323.4,E,4,12,1,5,M,0,M,,0000'))
        self.assertTrue(self.driver.heading_pub.publish.called)
        self.assertTrue(self.driver.fix_pub.publish.called)
        self.assertTrue(self.b.gnss_timebase.clock_ready)
        self.assertIsNone(self.b.gnss_timebase.fault)


if __name__ == '__main__':
    unittest.main()
