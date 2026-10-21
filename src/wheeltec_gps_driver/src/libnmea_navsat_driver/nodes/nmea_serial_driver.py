# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Eric Perko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Defines the main method for the nmea_serial_driver executable."""

import serial
import sys
import json
import os
import time

import rospy

from libnmea_navsat_driver.driver import RosNMEADriver
from libnmea_navsat_driver import driver as driver_module, unicore
from libnmea_navsat_driver import observation_time
from libnmea_navsat_driver.serial_diagnostics import SerialDiagnostics, source_identity
from std_msgs.msg import String


def process_serial_read(raw, driver, diagnostics, frame_id):
    """Tap the existing read result, not a second reader; preserve parse policy."""
    read_mono_ns = time.monotonic_ns()
    read_ros_ns = rospy.Time.now().to_nsec()
    diagnostics.observe_read(raw)
    if not raw:
        return
    try:
        sentence = raw.strip().decode('ascii')
    except UnicodeError as exc:
        diagnostics.error('ascii_decode_errors', exc)
        return
    diagnostics.count('ascii_decoded')
    try:
        driver.add_sentence(sentence, frame_id, host_read_ros_ns=read_ros_ns,
                            host_read_mono_ns=read_mono_ns)
    except ValueError as exc:
        diagnostics.error('driver_value_errors', exc)
        rospy.logwarn_throttle(2., 'GNSS sentence ValueError: %s; see /gnss/serial_diagnostics', str(exc))
    except Exception as exc:
        diagnostics.error('unhandled_errors', exc)
        raise  # Do not mask a real driver fault as successful parsing.


def main():
    """Create and run the nmea_serial_driver ROS node.

    Creates a ROS NMEA Driver and feeds it NMEA sentence strings from a serial device.

    :ROS Parameters:
        - ~port (str)
            Path of the serial device to open.
        - ~baud (int)
            Baud rate to configure the serial device.
    """
    rospy.init_node('nmea_serial_driver')

    serial_port = rospy.get_param('~port', '/dev/ttyUSB0')
    serial_baud = rospy.get_param('~baud', 4800)
    frame_id = RosNMEADriver.get_frame_id()
    diagnostics = SerialDiagnostics({
        'port': serial_port, 'baud': serial_baud, 'framing': '8N1',
        'nominal_payload_bytes_per_sec': float(serial_baud)/10.,
        'serial_timeout_sec': 2., 'pid': os.getpid(), 'python_executable': sys.executable,
        'node_name': rospy.get_name(),
        'reader': source_identity(__file__), 'driver': source_identity(driver_module.__file__),
        'unicore_parser': source_identity(unicore.__file__),
        'observation_time': source_identity(observation_time.__file__),
    })
    publisher = rospy.Publisher('/gnss/serial_diagnostics', String, queue_size=5, latch=True)
    print_summary = bool(rospy.get_param('~serial_diagnostics_print', False))
    driver = None

    def publish_diagnostics(_event=None):
        report = diagnostics.snapshot()
        report['stamp'] = rospy.Time.now().to_sec()
        if driver is not None:
            report['observation_timing'] = driver.observation_timing.summary()
        try:
            publisher.publish(String(data=json.dumps(report, allow_nan=False, separators=(',', ':'))))
            if print_summary:
                rate = report['rates_per_sec']
                rospy.loginfo('GNSS serial [%s] RX=%.0fB/s lines=%.1f/s PVTSLNA=%.1f/s '
                              'parsed=%.1f/s status_pub=%.1f/s decode_err=%d partial=%d',
                              report['state'],rate['read_bytes'],rate['complete_lines'],
                              rate['pvtslna_recognized'],rate['pvtslna_parse_ok'],
                              rate['solution_publish_calls'],report['window']['ascii_decode_errors'],
                              report['window']['partial_reads'])
        except Exception as exc:
            diagnostics.error('diagnostic_publish_errors', exc)
            rospy.logwarn_throttle(5., 'GNSS diagnostics publish failed: %s', str(exc))

    # Independent heartbeat continues even when readline is waiting on its 2s
    # timeout. Rates use measured monotonic elapsed time, not an assumed 1s.
    timer = rospy.Timer(rospy.Duration(1.), publish_diagnostics)
    GPS = None
    try:
        GPS = serial.Serial(port=serial_port, baudrate=serial_baud, timeout=2)
        diagnostics.set_state('open')
        driver = RosNMEADriver()
        driver.serial_diagnostics = diagnostics
        while not rospy.is_shutdown():
            process_serial_read(GPS.readline(), driver, diagnostics, frame_id)
    except rospy.ROSInterruptException:
        pass
    except serial.SerialException as ex:
        diagnostics.set_state('serial_error')
        diagnostics.error('serial_io_errors', ex)
        rospy.logfatal(
            "GNSS serial I/O error on %s: %s", serial_port, str(ex))
    except Exception as exc:
        diagnostics.set_state('driver_error')
        raise
    finally:
        timer.shutdown()
        try:
            if GPS is not None:
                GPS.close()
        except (serial.SerialException, OSError) as exc:
            diagnostics.set_state('serial_error')
            diagnostics.error('serial_io_errors', exc)
        if diagnostics.state not in ('serial_error', 'driver_error'):
            diagnostics.set_state('closed')
        publish_diagnostics()
