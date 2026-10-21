#!/usr/bin/env python3
"""Passive indoor timing capture; subscribes only, never commands the vehicle."""
import argparse
from collections import Counter, defaultdict
from datetime import datetime, timedelta, timezone
import json
from pathlib import Path
import struct
import threading
import time

import rospy
from std_msgs.msg import String


def distribution(values):
    values = sorted(values)
    if not values:
        return None
    return dict(count=len(values), min=values[0], max=values[-1], **{
        key: values[int((len(values)-1)*q)] for key, q in (('p50', .5), ('p95', .95), ('p99', .99))})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=60.)
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 300:
        parser.error('--seconds must be between 1 and 300')
    root = Path(__file__).resolve().parents[3]
    output = args.output_dir or root/'tracker_logs'/('sensor_time_'+datetime.now(
        timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S'))
    output.mkdir(parents=True, exist_ok=False)
    rospy.init_node('passive_sensor_time_check', anonymous=True, disable_signals=True)
    lock = threading.Lock()
    counts, reasons, quality = Counter(), Counter(), Counter()
    stamps, arrivals, ages = defaultdict(list), defaultdict(list), defaultdict(list)
    family_counts, valid_counts = Counter(), Counter()
    last_fields, streams, latest_serial = {}, {}, {}
    log = (output/'observations.jsonl').open('x', buffering=1)
    capture_open = [True]

    def save(topic, data, source_stamp=None):
        now, mono = rospy.Time.now().to_nsec(), time.monotonic_ns()
        with lock:
            if not capture_open[0]:
                return
            counts[topic] += 1
            arrivals[topic].append(mono)
            if source_stamp is not None:
                stamps[topic].append(source_stamp)
                ages[topic].append((now-source_stamp)/1e6)
            log.write(json.dumps(dict(topic=topic, host_receive_ros_ns=now,
                      host_receive_mono_ns=mono, source_stamp_ns=source_stamp,
                      data=data), separators=(',', ':'), allow_nan=False)+'\n')

    def diagnostic(msg, topic):
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            data = {'parse_error': True}
        with lock:
            if not capture_open[0]:
                return
            if topic == '/gnss/time_diagnostics':
                family = data.get('family', 'unknown')
                family_counts[family] += 1
                valid_counts[family] += int(data.get('time_valid') is True)
                reasons[family+':'+str(data.get('reason'))] += 1
                last_fields[family] = data.get('raw_time_fields')
                age = data.get('candidate_age_ms')
                if isinstance(age, (int, float)):
                    ages['GNSS_candidate_'+family].append(age)
            elif topic == '/gnss/solution_status':
                quality['position:'+str(data.get('position_type'))] += 1
                quality['heading:'+str(data.get('heading_type'))] += 1
            else:
                latest_serial.clear()
                latest_serial.update(data)
        save(topic, data)

    def header(msg, topic):
        # Known ROS1 messages below start with Header. Avoid deserializing all
        # Livox points in Python just to read 12 bytes of timing information.
        raw = msg._buff
        kind = msg._connection_header.get('type')
        data = {'ros_type': kind}
        if kind not in ('livox_ros_driver2/CustomMsg', 'sensor_msgs/Imu',
                        'nav_msgs/Odometry', 'sensor_msgs/NavSatFix', 'nmea_msgs/Gpgga') or len(raw) < 16:
            save(topic, dict(data, error='unexpected_header_layout'))
            return
        _, seconds, nanoseconds, frame_length = struct.unpack_from('<IIII', raw)
        data['frame_id'] = bytes(raw[16:16+frame_length]).decode('utf8', errors='replace')
        if kind == 'livox_ros_driver2/CustomMsg' and len(raw) >= 28+frame_length:
            data['timebase_ns'], data['point_count'] = struct.unpack_from('<QI', raw, 16+frame_length)
        save(topic, data, seconds*1000000000+nanoseconds)

    subscribers = []
    for topic in ('/gnss/time_diagnostics', '/gnss/solution_status', '/gnss/serial_diagnostics'):
        subscribers.append(rospy.Subscriber(topic, String, diagnostic, callback_args=topic, queue_size=200))
    for topic in ('/livox/lidar', '/livox/imu', '/Odometry', '/odometry/lio_guarded',
                  '/odometry/local', '/gps/fix', '/gnss/gpgga'):
        subscribers.append(rospy.Subscriber(topic, rospy.AnyMsg, header,
                           callback_args=topic, queue_size=10, buff_size=2**24))
    started = time.monotonic()
    print('Capturing static sensor timing to '+str(output), flush=True)
    try:
        while not rospy.is_shutdown() and time.monotonic()-started < args.seconds:
            time.sleep(.1)
    finally:
        for subscriber in subscribers:
            subscriber.unregister()
        with lock:
            capture_open[0] = False
            log.close()
            for topic, count in counts.items():
                a, s = arrivals[topic], stamps[topic]
                streams[topic] = dict(count=count,
                    receive_hz=None if len(a) < 2 else (len(a)-1)*1e9/(a[-1]-a[0]),
                    header_minus_host_warning='header age is not certified transport latency',
                    candidate_header_age_ms=distribution(ages[topic]),
                    source_step_ms=distribution([(y-x)/1e6 for x, y in zip(s, s[1:])]),
                    source_repeated=sum(y == x for x, y in zip(s, s[1:])),
                    source_backwards=sum(y < x for x, y in zip(s, s[1:])))
            report = dict(duration_sec=time.monotonic()-started, streams=streams,
                gnss_family_counts=dict(family_counts), gnss_valid_time_counts=dict(valid_counts),
                gnss_time_reasons=dict(reasons), gnss_quality_counts=dict(quality),
                latest_time_fields=last_fields, latest_serial=latest_serial,
                candidate_gnss_age_ms={family: distribution(ages['GNSS_candidate_'+family])
                    for family in family_counts},
                limitation='Stationary shadow check only; no navigation, no measurement-mode switch; '
                           'LIO guard/EKF topics may be absent because their stack is not started.')
            (output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        print(json.dumps(report, indent=2, allow_nan=False), flush=True)
        rospy.signal_shutdown('passive timing check complete')


if __name__ == '__main__':
    main()
