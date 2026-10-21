"""Run with python3 -m unittest discover
-s src/Test/tests -p test_unicore_solution.py
"""
import unittest
import json
from pathlib import Path
import sys
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'src/wheeltec_gps_driver/src'))
from libnmea_navsat_driver.unicore import crc32, parse_pvtslna


RAW = ('#PVTSLNA,95,GPS,UNKNOWN,1,66874000,0,0,18,2;'
       'NONE,0.0000,0.00000000000,0.00000000000,0.0000,0.0000,0.0000,0.000,'
       'NONE,0.0000,0.00000000000,0.00000000000,0.0000,0,0,0,0,0.0000,0.0000,'
       '-0.0000,NONE,0.0000,0.0000,0.0000,0,0,0,0,0.0000,0.0000,0.0000,'
       '0.0000,0.0000,0.0,0*3624c30f')


def solution(position='SINGLE', heading='NARROW_INT', baseline='1.0', angle='0.0'):
    header, payload = RAW.split('*')[0].split(';')
    fields = payload.split(',')
    fields[0], fields[20], fields[21], fields[22] = position, heading, baseline, angle
    fields[25] = '12'
    body = header + ';' + ','.join(fields)
    return body + '*{:08x}'.format(crc32(body[1:]))


class SolutionTests(unittest.TestCase):
    def test_real_receiver_no_solution(self):
        result = parse_pvtslna(RAW)
        self.assertEqual(result['heading_type'], 'NONE')
        self.assertFalse(result['heading_valid'])

    def test_position_and_heading_are_independent(self):
        result = parse_pvtslna(solution())
        self.assertFalse(result['position_fixed'])
        self.assertTrue(result['heading_valid'])  # True north (zero) is valid when fixed.
        result = parse_pvtslna(solution(position='NARROW_INT', heading='NARROW_FLOAT'))
        self.assertTrue(result['position_fixed'])
        self.assertFalse(result['heading_valid'])

    def test_degenerate_baseline(self):
        self.assertFalse(parse_pvtslna(solution(baseline='0'))['heading_valid'])

    def test_bad_crc_truncation_and_nonfinite(self):
        for text in (RAW[:-8] + '00000000', solution(angle='nan'),
                     solution(angle='361'), '#PVTSLNA,0;NONE*00000000'):
            with self.subTest(text=text), self.assertRaises(ValueError):
                parse_pvtslna(text)


class DriverTests(unittest.TestCase):
    def test_heading_gate(self):
        try:
            import rospy
            from libnmea_navsat_driver.driver import RosNMEADriver
        except ImportError:
            self.skipTest('Requires ROS1 environment')

        class Publisher:
            def __init__(self, *args, **kwargs):
                self.messages = []

            def publish(self, msg):
                self.messages.append(msg)

        now = [rospy.Time(100)]
        with patch.object(rospy, 'Publisher', Publisher), \
                patch.object(rospy, 'get_param', lambda key, default=None: default), \
                patch.object(rospy.Time, 'now', lambda: now[0]), \
                patch.object(rospy, 'get_rostime', lambda: now[0]):
            driver = RosNMEADriver()
            hdt = '$GPHDT,0.0000,T*05'
            driver.add_sentence(hdt, 'gps_link')
            self.assertFalse(driver.heading_pub.messages)
            driver.add_sentence(RAW, 'gps_link')
            driver.add_sentence(hdt, 'gps_link')
            self.assertFalse(driver.heading_pub.messages)
            driver.add_sentence(solution(), 'gps_link')
            self.assertEqual(len(driver.heading_pub.messages), 1)
            status = json.loads(driver.solution_pub.messages[-1].data)
            self.assertEqual(status['stamp_ns'],driver.heading_pub.messages[-1].header.stamp.to_nsec())
            driver.add_sentence(hdt, 'gps_link')
            self.assertEqual(len(driver.heading_pub.messages), 1)
            self.assertFalse(json.loads(driver.solution_pub.messages[-1].data)['position_fixed'])
            now[0] = rospy.Time(103)
            driver.add_sentence(hdt, 'gps_link')
            self.assertEqual(len(driver.heading_pub.messages), 1)
            for text in (solution(heading='NARROW_FLOAT'), solution()[:-8] + '00000000'):
                driver.add_sentence(text, 'gps_link')
                driver.add_sentence(hdt, 'gps_link')
                self.assertEqual(len(driver.heading_pub.messages), 1)

    def test_new_hdt_cannot_borrow_previous_fixed_and_pvtslna_pairs_are_atomic(self):
        try:
            import rospy
            from libnmea_navsat_driver.driver import RosNMEADriver
        except ImportError:
            self.skipTest('Requires ROS1 environment')
        from unittest.mock import Mock
        now=[rospy.Time(100)]
        with patch.object(rospy,'Publisher',side_effect=lambda *a,**k:Mock()), \
             patch.object(rospy,'get_param',lambda key,default=None:default), \
             patch.object(rospy.Time,'now',lambda:now[0]), \
             patch.object(rospy,'get_rostime',lambda:now[0]):
            driver=RosNMEADriver()
            driver.add_sentence(solution(angle='10'),'gps_link')
            self.assertEqual(driver.heading_pub.publish.call_count,1)
            now[0]=rospy.Time(100,10000000)
            driver.add_sentence('$GPHDT,0.0000,T*05','gps_link')
            self.assertEqual(driver.heading_pub.publish.call_count,1)
            driver.add_sentence(solution(heading='NARROW_FLOAT',angle='20'),'gps_link')
            self.assertEqual(driver.heading_pub.publish.call_count,1)
            now[0]=rospy.Time(100,20000000)
            driver.add_sentence(solution(angle='30'),'gps_link')
            self.assertEqual(driver.heading_pub.publish.call_count,2)
            heading=driver.heading_pub.publish.call_args[0][0]
            status=json.loads(driver.solution_pub.publish.call_args[0][0].data)
            self.assertEqual(heading.header.stamp.to_nsec(),status['stamp_ns'])
            self.assertTrue(status['heading_fixed'])


if __name__ == '__main__':
    unittest.main()
