"""Passive serial pipeline counters. No hardware, ROS master or extra reader."""
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'src/wheeltec_gps_driver/src'))
from libnmea_navsat_driver.serial_diagnostics import SerialDiagnostics, source_identity


class CounterTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.
        self.stats = SerialDiagnostics(clock=lambda: self.now)

    def test_counts_bytes_before_strip_and_empty_timeout(self):
        raw = b'  #PVTSLNA,test\r\n'
        self.stats.observe_read(raw)
        self.stats.observe_read(b'')
        self.assertEqual(self.stats.total['read_bytes'], len(raw))
        self.assertEqual(self.stats.total['read_calls'], 2)
        self.assertEqual(self.stats.total['complete_lines'], 1)
        self.assertEqual(self.stats.total['empty_reads'], 1)
        self.assertEqual(self.stats.total['raw_pvtslna_prefix'], 1)

    def test_partial_blank_alternate_and_embedded_prefix_are_visible(self):
        for line in (b'#PVTSLNA,partial', b'\r\n', b'#PVTSLN,x\r\n', b'garbage#PVTSLNA,x\r\n'):
            self.stats.observe_read(line)
        self.assertEqual(self.stats.total['partial_reads'], 1)
        self.assertEqual(self.stats.total['blank_lines'], 1)
        self.assertEqual(self.stats.total['alternate_pvtsln'], 1)
        self.assertEqual(self.stats.total['embedded_pvtslna'], 1)

    def test_bounded_family_names_count_all_traffic_not_only_pvtslna(self):
        for line in (b'$GNGGA,x\n',b'$GPHPR,x\n',b'$GPGST,x\n',b'#BESTNAVA,x\n',
                     b'#KSXT,x\n',b'#AGRICA,x\n',b'#UNKNOWN,x\n',b'\xff\x80\n'):
            self.stats.observe_read(line)
        for family in ('GGA','HPR','GST','BESTNAV','KSXT','AGRIC'):
            self.assertEqual(self.stats.total['family_'+family],1)
        self.assertEqual(self.stats.total['family_OTHER'],2)
        self.assertEqual(len(self.stats.total),len(SerialDiagnostics.COUNTERS))

    def test_actual_elapsed_rates_reset_window_not_totals(self):
        self.stats.count('solution_publish_calls', 20)
        self.now=2.
        first=self.stats.snapshot()
        self.assertEqual(first['rates_per_sec']['solution_publish_calls'],10.)
        self.now=3.
        second=self.stats.snapshot()
        self.assertEqual(second['window']['solution_publish_calls'],0)
        self.assertEqual(second['total']['solution_publish_calls'],20)
        self.assertEqual(second['session_id'],first['session_id'])
        self.assertEqual(second['sequence'],2)

    def test_receive_gaps_and_age_do_not_claim_device_measurement_time(self):
        self.stats.recognized_pvtslna()
        self.now=.1
        self.stats.recognized_pvtslna()
        self.now=1.
        report=self.stats.snapshot()
        self.assertAlmostEqual(report['pvtslna_max_receive_gap_sec'],.1)
        self.assertAlmostEqual(report['pvtslna_receive_age_sec'],.9)

    def test_zero_elapsed_report_is_strict_json(self):
        json.dumps(self.stats.snapshot(),allow_nan=False)

    def test_source_identity_hashes_actual_file(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'source.py'
            # Test data, not an implementation file.
            path.touch()
            result=source_identity(path)
            self.assertEqual(result['path'],str(path.resolve()))
            self.assertEqual(len(result['sha256']),64)
        self.assertIn('error',source_identity(path))

    def test_concurrent_updates_and_snapshots_do_not_lose_counts(self):
        threads=[threading.Thread(target=lambda:[self.stats.count('read_calls') for _ in range(1000)])
                 for _ in range(4)]
        for t in threads:t.start()
        for _ in range(10):self.stats.snapshot()
        for t in threads:t.join()
        self.assertEqual(self.stats.snapshot()['total']['read_calls'],4000)

    def test_error_detail_is_bounded(self):
        self.stats.error('ascii_decode_errors','x'*1000)
        self.assertLessEqual(len(self.stats.snapshot()['last_error']['detail']),200)


try:
    import rospy
    from libnmea_navsat_driver.driver import RosNMEADriver
    from libnmea_navsat_driver.nodes.nmea_serial_driver import process_serial_read
    from test_unicore_solution import solution
    ROS_AVAILABLE=True
except ImportError:
    ROS_AVAILABLE=False


@unittest.skipUnless(ROS_AVAILABLE,'requires ROS1 imports; still no ROS master/hardware')
class PipelineTests(unittest.TestCase):
    def setUp(self):
        self.now=100.
        for p in (patch.object(rospy,'Publisher',side_effect=lambda *a,**k:Mock()),
                  patch.object(rospy,'get_param',lambda k,default=None:default),
                  patch.object(rospy.Time,'now',lambda:rospy.Time.from_sec(self.now)),
                  patch.object(rospy,'get_rostime',lambda:rospy.Time.from_sec(self.now)),
                  patch.object(rospy,'logwarn_throttle')):
            p.start();self.addCleanup(p.stop)
        self.stats=SerialDiagnostics(clock=lambda:self.now)
        self.driver=RosNMEADriver()
        self.driver.serial_diagnostics=self.stats

    def test_real_parser_10hz_and_20hz_publish_one_status_per_packet(self):
        for hz in (10,20):
            before=self.stats.total['solution_publish_calls']
            for _ in range(hz*2):
                self.now+=1./hz
                process_serial_read((solution()+'\r\n').encode(),self.driver,self.stats,'gps_link')
            self.assertEqual(self.stats.total['solution_publish_calls']-before,hz*2)
            self.assertEqual(self.stats.total['pvtslna_parse_ok'],self.stats.total['solution_publish_calls'])

    def test_float_bad_crc_non_ascii_and_partial_are_distinct(self):
        process_serial_read((solution(heading='NARROW_FLOAT')+'\r\n').encode(),self.driver,self.stats,'gps')
        bad=solution()[:-8]+'00000000'
        process_serial_read((bad+'\r\n').encode(),self.driver,self.stats,'gps')
        process_serial_read(b'#PVTSLNA,\xff\r\n',self.driver,self.stats,'gps')
        process_serial_read(b'#PVTSLNA,cut',self.driver,self.stats,'gps')
        self.assertEqual(self.stats.total['raw_pvtslna_prefix'],4)
        self.assertEqual(self.stats.total['pvtslna_recognized'],3)
        self.assertEqual(self.stats.total['ascii_decode_errors'],1)
        self.assertEqual(self.stats.total['partial_reads'],1)
        self.assertEqual(self.stats.total['pvtslna_parse_ok'],1)
        self.assertEqual(self.stats.total['pvtslna_parse_errors'],2)
        self.assertEqual(self.stats.total['pvtslna_crc_errors'],1)
        self.assertEqual(self.stats.total['solution_publish_calls'],3) # Includes error statuses.
        self.assertEqual(self.stats.total['heading_publish_calls'],0)

    def test_publish_failure_is_not_a_success_or_silently_swallowed(self):
        self.driver.solution_pub.publish.side_effect=RuntimeError('injected')
        with self.assertRaises(RuntimeError):
            process_serial_read((solution()+'\n').encode(),self.driver,self.stats,'gps')
        self.assertEqual(self.stats.total['solution_publish_calls'],0)
        self.assertEqual(self.stats.total['solution_publish_errors'],1)
        self.assertEqual(self.stats.total['unhandled_errors'],1)

    def test_empty_read_never_enters_parser_and_value_errors_are_counted(self):
        process_serial_read(b'',self.driver,self.stats,'gps')
        self.assertEqual(self.stats.total['parser_calls'],0)
        self.driver.add_sentence=Mock(side_effect=ValueError('injected'))
        process_serial_read(b'$GNGGA,x\r\n',self.driver,self.stats,'gps')
        self.assertEqual(self.stats.total['driver_value_errors'],1)

    def test_main_uses_one_reader_and_timer_can_report_when_no_data(self):
        from libnmea_navsat_driver.nodes import nmea_serial_driver as node
        captured=[]
        timer=Mock()
        callback=[]
        def make_timer(duration, cb):
            callback.append(cb)
            return timer
        port=Mock()
        def readline():
            callback[0](None) # Heartbeat without any completed serial read.
            return b''
        port.readline.side_effect=readline
        def publisher(topic,*a,**k):
            p=Mock()
            if topic=='/gnss/serial_diagnostics':
                p.publish.side_effect=lambda msg:captured.append(json.loads(msg.data))
            return p
        with patch.object(rospy,'init_node'),patch.object(rospy,'get_name',return_value='/test_gps'), \
             patch.object(node.RosNMEADriver,'get_frame_id',return_value='gps'), \
             patch.object(rospy,'is_shutdown',side_effect=[False,True]), \
             patch.object(rospy,'Timer',side_effect=make_timer), \
             patch.object(rospy,'Publisher',side_effect=publisher), \
             patch.object(rospy,'loginfo'),patch.object(node.serial,'Serial',return_value=port) as opened:
            node.main()
        opened.assert_called_once()
        port.readline.assert_called_once()
        port.write.assert_not_called()
        port.close.assert_called_once()
        timer.shutdown.assert_called_once()
        self.assertEqual(captured[0]['total']['read_calls'],0)
        self.assertEqual(captured[-1]['total']['empty_reads'],1)
        self.assertEqual(captured[-1]['state'],'closed')
        self.assertIn('sha256',captured[-1]['metadata']['driver'])


if __name__=='__main__':unittest.main()
