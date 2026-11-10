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

"""Provides a driver for NMEA GNSS devices."""

import math
import json
import time

import rospy

from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from geometry_msgs.msg import TwistStamped, QuaternionStamped
from tf.transformations import quaternion_from_euler

from libnmea_navsat_driver.checksum_utils import check_nmea_checksum
import libnmea_navsat_driver.parser

from nmea_msgs.msg import Sentence #wheeltec tues
from nmea_msgs.msg import Gpgga 
from geometry_msgs.msg import PoseStamped 
from std_msgs.msg import Float32, String
from libnmea_navsat_driver.unicore import parse_pvtslna
from libnmea_navsat_driver.observation_time import ObservationTiming
from geometry_msgs.msg import TwistWithCovarianceStamped

class RosNMEADriver(object):
    """ROS driver for NMEA GNSS devices."""

    @staticmethod
    def _normalize_deg(angle_deg):
        """Normalize an angle to [-180, 180)."""
        wrapped = (angle_deg + 180.0) % 360.0 - 180.0
        # Keep 180 represented as 180 instead of -180 for easier human reading.
        if wrapped == -180.0 and angle_deg > 0.0:
            return 180.0
        return wrapped

    def __init__(self):
        """Initialize the ROS NMEA driver.

        :ROS Publishers:
            - NavSatFix publisher on the 'fix' channel.
            - TwistStamped publisher on the 'vel' channel.
            - QuaternionStamped publisher on the 'heading' channel.
            - TimeReference publisher on the 'time_reference' channel.

        :ROS Parameters:
            - ~time_ref_source (str)
                The name of the source in published TimeReference messages. (default None)
            - ~useRMC (bool)
                If true, use RMC NMEA messages. If false, use GGA and VTG messages. (default False)
            - ~epe_quality0 (float)
                Value to use for default EPE quality for fix type 0. (default 1000000)
            - ~epe_quality1 (float)
                Value to use for default EPE quality for fix type 1. (default 4.0)
            - ~epe_quality2 (float)
                Value to use for default EPE quality for fix type 2. (default (0.1)
            - ~epe_quality4 (float)
                Value to use for default EPE quality for fix type 4. (default 0.02)
            - ~epe_quality5 (float)
                Value to use for default EPE quality for fix type 5. (default 4.0)
            - ~epe_quality9 (float)
                Value to use for default EPE quality for fix type 9. (default 3.0)
        """
        self.fix_pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=1)
        self.vel_pub = rospy.Publisher('/gps/vel', TwistStamped, queue_size=1)
        self.heading_pub = rospy.Publisher('heading', QuaternionStamped, queue_size=1)
        self.solution_pub = rospy.Publisher('/gnss/solution_status', String, queue_size=10)
        self.require_heading_fixed = rospy.get_param('~require_heading_fixed', True)
        self.solution_timeout = max(0.1, float(rospy.get_param('~solution_timeout_sec', 2.5)))
        self.last_solution = None
        self.last_solution_recv = None
        self.serial_diagnostics = None  # Attached only by the existing serial reader.
        self.pvt_period_sec = float(rospy.get_param('~pvt_period_sec', .1))
        self.gga_period_sec = float(rospy.get_param('~gga_period_sec', .1))
        self.cadence_margin_sec = float(rospy.get_param('~cadence_margin_sec', .3))
        self.observation_timing = ObservationTiming(
            rospy.get_param('~time_mode', 'shadow'),
            rospy.get_param('~time_clock_verified', False),
            pvt_period_sec=self.pvt_period_sec, gga_period_sec=self.gga_period_sec,
            cadence_margin_sec=self.cadence_margin_sec,
            transport_limit_sec=rospy.get_param('~gnss_transport_limit_sec',0.),
            common_utc_verified=rospy.get_param('~gnss_common_utc_verified',False))
        # Coarse G90 installation sanity envelope, NOT an exact 0.67m
        # calibration. Only the newly admitted L1 class needs this extra gate;
        # existing NARROW_INT continuity/quality contract is unchanged.
        self.l1_baseline_min=float(rospy.get_param('~l1_heading_baseline_min_m',.3))
        self.l1_baseline_max=float(rospy.get_param('~l1_heading_baseline_max_m',1.2))
        self.l1_pitch_max=float(rospy.get_param('~l1_heading_pitch_max_deg',45.))
        if not (0<self.l1_baseline_min<self.l1_baseline_max<=3. and 0<self.l1_pitch_max<=45.):
            raise ValueError('invalid L1 heading installation sanity envelope')
        rospy.loginfo('GNSS configured cadence: PVT %.3fs, GGA %.3fs, margin %.3fs; '
                      'transport_limit=%s common_UTC_asserted=%s (not hardware proof)',
                      self.pvt_period_sec, self.gga_period_sec, self.cadence_margin_sec,
                      self.observation_timing.transport_limit_sec or 'legacy: PVT .5s, GGA 1s',
                      self.observation_timing.common_utc_verified)
        self.time_diagnostics_pub = rospy.Publisher('/gnss/time_diagnostics', String, queue_size=200)
        self._time_event = None

        self.eagleye_vel_pub =rospy.Publisher('/gps/nmea/vel',TwistWithCovarianceStamped,  queue_size=1)

        self.pose_pub = rospy.Publisher('/gps/pose',PoseStamped,queue_size=1)
        self.nmea_pub = rospy.Publisher('/gps/nmea_sentence', Sentence, queue_size=1)
        self.gpgga_pub = rospy.Publisher('/gnss/gpgga',Gpgga,queue_size=1)
        self.heading_deg_pub = rospy.Publisher("heading_deg", Float32, queue_size=10)

        self.use_GNSS_time = rospy.get_param('~use_GNSS_time', False)
        if self.use_GNSS_time:
            rospy.logwarn('Legacy use_GNSS_time only changes NMEA stamps, not the PVTSLNA chain; '
                          'it is NOT unified measurement-time alignment. Use host stamps for shadow acceptance.')
        if not self.use_GNSS_time:
            self.time_ref_pub = rospy.Publisher(
                'time_reference', TimeReference, queue_size=1)

        self.time_ref_source = rospy.get_param('~time_ref_source', None)
        self.use_RMC = rospy.get_param('~useRMC', False)
        if self.observation_timing.mode == 'measurement' and (
                self.use_GNSS_time or self.use_RMC or not self.require_heading_fixed):
            raise ValueError('measurement mode requires GGA position, same-PVTSLNA heading, and legacy use_GNSS_time=false')
        self._measurement_event = None

        self.rotation_data = rospy.get_param('~rotation', 90.0)
        self.heading_source_is_north_cw = rospy.get_param('~heading_source_is_north_cw', True)
        self.debug_output = rospy.get_param('~debug_output', False)
        rospy.loginfo('G90 heading rotation offset: %.2f deg', self.rotation_data)
 
        self.valid_fix = False

        # epe = estimated position error
        self.default_epe_quality0 = rospy.get_param('~epe_quality0', 1000000)
        self.default_epe_quality1 = rospy.get_param('~epe_quality1', 4.0)
        self.default_epe_quality2 = rospy.get_param('~epe_quality2', 0.1)
        self.default_epe_quality4 = rospy.get_param('~epe_quality4', 0.02)
        self.default_epe_quality5 = rospy.get_param('~epe_quality5', 4.0)
        self.default_epe_quality9 = rospy.get_param('~epe_quality9', 3.0)
        self.using_receiver_epe = False

        self.lon_std_dev = float("nan")
        self.lat_std_dev = float("nan")
        self.alt_std_dev = float("nan")

        """Format for this dictionary is the fix type from a GGA message as the key, with
        each entry containing a tuple consisting of a default estimated
        position error, a NavSatStatus value, and a NavSatFix covariance value."""
        self.gps_qualities = {
            # Unknown
            -1: [
                self.default_epe_quality0,
                NavSatStatus.STATUS_NO_FIX,
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # Invalid
            0: [
                self.default_epe_quality0,
                NavSatStatus.STATUS_NO_FIX,
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # SPS
            1: [
                self.default_epe_quality1,
                NavSatStatus.STATUS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # DGPS
            2: [
                self.default_epe_quality2,
                NavSatStatus.STATUS_SBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Fix
            4: [
                self.default_epe_quality4,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Float
            5: [
                self.default_epe_quality5,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # WAAS
            9: [
                self.default_epe_quality9,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ]
        }

    def _log_fix(self, current_fix, fix_type):
        if not self.debug_output:
            return
        rospy.loginfo_throttle(
            1.0,
            (
                "GPS fix published: lat=%.9f lon=%.9f alt=%.3f "
                "status=%d frame=%s fix_type=%s"
            ),
            current_fix.latitude,
            current_fix.longitude,
            current_fix.altitude,
            current_fix.status.status,
            current_fix.header.frame_id,
            str(fix_type),
        )

    def _publish_counted(self, publisher, message, kind):
        if self._time_event is not None:
            self._time_event['published'][kind] = {
                'publish_ros_ns': rospy.Time.now().to_nsec(),
                'source_stamp_ns': message.header.stamp.to_nsec() if hasattr(message, 'header') else None,
                'publish_call_ok': False,
            }
        try:
            publisher.publish(message)
        except Exception as exc:
            if self.serial_diagnostics is not None:
                self.serial_diagnostics.error(kind+'_publish_errors', exc)
            raise
        if self._time_event is not None:
            self._time_event['published'][kind]['publish_call_ok'] = True
        if self.serial_diagnostics is not None:
            self.serial_diagnostics.count(kind+'_publish_calls')

    def add_sentence(self, nmea_string, frame_id, timestamp=None,
                     host_read_ros_ns=None, host_read_mono_ns=None):
        """Observe timing without changing the existing host-time data contract."""
        captured = host_read_ros_ns is not None and host_read_mono_ns is not None
        read_ros = rospy.Time.now().to_nsec() if host_read_ros_ns is None else host_read_ros_ns
        read_mono = time.monotonic_ns() if host_read_mono_ns is None else host_read_mono_ns
        publications = {'published': {}}
        measurement = self.observation_timing.mode == 'measurement'
        self._time_event = publications if self.observation_timing.mode != 'host' else None
        event = None
        if measurement:
            # Measurement stamps must be resolved before any sensor data is
            # published. Shadow keeps its original post-publication tap.
            event = self.observation_timing.observe(nmea_string, read_ros, read_mono,
                        'serial_read_complete' if captured else 'parser_entry_not_serial_read')
        self._measurement_event = event
        result = None
        try:
            result = self._add_sentence(nmea_string, frame_id, timestamp)
            return result
        finally:
            self._time_event = None
            try:
                # Parse the diagnostic AFTER data publication, not on the
                # critical path before choosing the original host timestamps.
                if not measurement:
                    event = self.observation_timing.observe(nmea_string, read_ros, read_mono,
                                'serial_read_complete' if captured else 'parser_entry_not_serial_read')
                if event is not None:
                    event.update(publications)
                    event['navigation_stamp_policy'] = ('measurement' if measurement else
                        'legacy_nmea_gnss_pvts_host' if self.use_GNSS_time else 'host')
                    event['parser_accepted'] = None if result is None else bool(result)
                    event['diagnostic_publish_ros_ns'] = rospy.Time.now().to_nsec()
                    event['processing_ms'] = (time.monotonic_ns()-read_mono)/1000000.
                    self.time_diagnostics_pub.publish(String(data=json.dumps(
                        event, allow_nan=False, separators=(',', ':'))))
            except Exception as exc:
                # Diagnostics cannot suppress a parser/IO exception or change
                # normal position/heading publication behavior.
                rospy.logwarn_throttle(5., 'GNSS time diagnostics publish failed: %s', str(exc))
            self._measurement_event = None

    def measurement_accepted(self):
        event = self._measurement_event or {}
        timeout = self.observation_timing.transport_limit_sec or (.5 if event.get('family') == 'PVTSLNA' else 1.)
        return (event.get('time_valid') is True and event.get('clock_verified') is True
                and event.get('epoch_order') == 'new'
                and -.05 <= event.get('measurement_age_ms', float('inf'))/1000. <= timeout)

    def measurement_stamp(self):
        seconds, nanos = divmod(self._measurement_event['measurement_stamp_ns'], 1000000000)
        return rospy.Time(seconds, nanos)

    def _add_sentence(self, nmea_string, frame_id, timestamp=None):
        """Public method to provide a new NMEA sentence to the driver.

        Args:
            nmea_string (str): NMEA sentence in string form.
            frame_id (str): TF frame ID of the GPS receiver.
            timestamp(rospy.Time, optional): Time the sentence was received.
                If timestamp is not specified, the current time is used.

        Returns:
            bool: True if the NMEA string is successfully processed, False if there is an error.
        """
        if self.serial_diagnostics is not None:
            self.serial_diagnostics.count('parser_calls')
        if nmea_string.startswith('#PVTSLNA,'):
            if self.serial_diagnostics is not None:
                self.serial_diagnostics.recognized_pvtslna()
            now = rospy.Time.now()
            if (self.require_heading_fixed and self.last_solution_recv is not None
                    and (now-self.last_solution_recv).to_sec() > self.pvt_period_sec+self.cadence_margin_sec):
                rospy.logwarn_throttle(8., 'PVTSLNA host arrival gap exceeds configured %.3fs; inspect '
                                      '/gnss/serial_diagnostics and actual serial output. '
                                      'This does not prove the receiver configuration is slow.',
                                      self.pvt_period_sec+self.cadence_margin_sec)
            try:
                self.last_solution = parse_pvtslna(nmea_string)
                if self.last_solution['heading_type']=='L1_INT':
                    geometry_ok=(self.l1_baseline_min<=self.last_solution['baseline_m']<=self.l1_baseline_max
                        and abs(self.last_solution['pitch_deg'])<=self.l1_pitch_max
                        and self.last_solution['heading_satellites']>=4)
                    self.last_solution['heading_geometry_valid']=geometry_ok
                    self.last_solution['heading_valid'] &= geometry_ok
                if self.serial_diagnostics is not None:
                    self.serial_diagnostics.count('pvtslna_parse_ok')
            except (ValueError, IndexError, UnicodeError) as exc:
                self.last_solution = {'source': 'PVTSLNA', 'heading_valid': False,
                                      'heading_fixed': False, 'error': str(exc)}
                rospy.logwarn_throttle(2.0, 'Rejected GNSS solution status: %s', str(exc))
                if self.serial_diagnostics is not None:
                    self.serial_diagnostics.error('pvtslna_parse_errors', exc)
                    if 'CRC32' in str(exc):
                        self.serial_diagnostics.count('pvtslna_crc_errors')
            self.last_solution_recv = now
            self.last_solution.update(receiver_heading_fixed=self.last_solution.get('heading_fixed'),
                                      receiver_heading_valid=self.last_solution.get('heading_valid'))
            source_time = now
            if self.observation_timing.mode == 'measurement':
                event = self._measurement_event or {}
                stamp_ns = event.get('measurement_stamp_ns')
                # Invalid latest time/CRC is still published as failed health;
                # never stamp it with "now" as a healthy GNSS observation.
                if event.get('time_valid') and event.get('clock_verified'):
                    source_time = self.measurement_stamp()
                    if event.get('epoch_order')=='rejected_age':
                        self.last_solution.update(heading_valid=False,error='measurement_transport_age_rejected')
                else:
                    source_time = rospy.Time(0)
                    self.last_solution.update(heading_valid=False, heading_fixed=False,
                        error='measurement_time_invalid:'+str(event.get('clock_gate_reason', event.get('reason'))))
                self.last_solution.update(time_mode='measurement',
                    measurement_stamp_ns=stamp_ns, time_valid=event.get('time_valid') is True,
                    clock_verified=event.get('clock_verified') is True,
                    clock_waiting=event.get('clock_waiting', False),
                    clock_gate_reason=event.get('clock_gate_reason'),
                    clock_epoch=event.get('clock_epoch'), session_id=event.get('session_id'),
                    observation_id=event.get('observation_id'),
                    host_read_ros_ns=event.get('host_read_ros_ns'), host_read_mono_ns=event.get('host_read_mono_ns'))
            self._publish_counted(self.solution_pub, String(data=json.dumps(
                dict(self.last_solution, stamp=source_time.to_sec(), stamp_ns=source_time.to_nsec()), allow_nan=False)), 'solution')
            # HDT has no measurement epoch or quality. In Fixed-gated mode use
            # heading and quality from ONE CRC-checked PVTSLNA packet instead
            # of letting a new HDT borrow the previous packet's Fixed state.
            if (self.require_heading_fixed and self.last_solution.get('heading_valid') is True
                    and (self.observation_timing.mode != 'measurement' or self.measurement_accepted())):
                angle = self.last_solution['heading_deg'] + self.rotation_data
                yaw_deg = self._normalize_deg(90.0-angle if self.heading_source_is_north_cw else angle)
                q = quaternion_from_euler(0, 0, math.radians(yaw_deg))
                heading = QuaternionStamped()
                heading.header.stamp, heading.header.frame_id = source_time, frame_id
                heading.quaternion.x, heading.quaternion.y, heading.quaternion.z, heading.quaternion.w = q
                self._publish_counted(self.heading_pub, heading, 'heading')
                self.heading_deg_pub.publish(Float32(data=self._normalize_deg(angle)))
            return 'error' not in self.last_solution
        if not check_nmea_checksum(nmea_string):
            if self.serial_diagnostics is not None:
                self.serial_diagnostics.count('nmea_checksum_rejected' if nmea_string.startswith('$')
                                              else 'unsupported_or_unparsed')
            #rospy.logwarn("Received a sentence with an invalid checksum. " +
            #              "Sentence was: %s" % repr(nmea_string))
            
            return False

        parsed_sentence = libnmea_navsat_driver.parser.parse_nmea_sentence(
            nmea_string)
        if not parsed_sentence:
            if self.serial_diagnostics is not None:
                self.serial_diagnostics.count('unsupported_or_unparsed')
            rospy.logdebug(
                "Failed to parse NMEA sentence. Sentence was: %s" %
                nmea_string)
            return False

        if timestamp:
            current_time = timestamp
        else:
            current_time = rospy.get_rostime()
        current_fix = NavSatFix()
        current_fix.header.stamp = current_time
        current_fix.header.frame_id = frame_id
        if not self.use_GNSS_time:
            current_time_ref = TimeReference()
            current_time_ref.header.stamp = current_time
            current_time_ref.header.frame_id = frame_id
            if self.time_ref_source:
                current_time_ref.source = self.time_ref_source
            else:
                current_time_ref.source = frame_id

        if not self.use_RMC and 'GGA' in parsed_sentence:
            if self.observation_timing.mode == 'measurement':
                if not self.measurement_accepted():
                    return False
                current_fix.header.stamp = self.measurement_stamp()
            current_fix.position_covariance_type = \
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED

            data = parsed_sentence['GGA']

            if self.use_GNSS_time:
                if math.isnan(data['utc_time'][0]):
                    rospy.logwarn("Time in the NMEA sentence is NOT valid")
                    return False
                current_fix.header.stamp = rospy.Time(data['utc_time'][0], data['utc_time'][1])

            fix_type = data['fix_type']
            if not (fix_type in self.gps_qualities):
                fix_type = -1
            gps_qual = self.gps_qualities[fix_type]
            default_epe = gps_qual[0]
            current_fix.status.status = gps_qual[1]
            current_fix.position_covariance_type = gps_qual[2]

            self.valid_fix = (fix_type > 0)

            current_fix.status.service = NavSatStatus.SERVICE_GPS

            latitude = data['latitude']
            if data['latitude_direction'] == 'S':
                latitude = -latitude
            current_fix.latitude = latitude

            longitude = data['longitude']
            if data['longitude_direction'] == 'W':
                longitude = -longitude
            current_fix.longitude = longitude

            # Altitude is above ellipsoid, so adjust for mean-sea-level
            altitude = data['altitude'] + data['mean_sea_level']
            current_fix.altitude = altitude

            # use default epe std_dev unless we've received a GST sentence with
            # epes
            if not self.using_receiver_epe or math.isnan(self.lon_std_dev):
                self.lon_std_dev = default_epe
            if not self.using_receiver_epe or math.isnan(self.lat_std_dev):
                self.lat_std_dev = default_epe
            if not self.using_receiver_epe or math.isnan(self.alt_std_dev):
                self.alt_std_dev = default_epe * 2

            hdop = data['hdop']
            current_fix.position_covariance[0] = (hdop * self.lon_std_dev) ** 2
            current_fix.position_covariance[4] = (hdop * self.lat_std_dev) ** 2
            current_fix.position_covariance[8] = (
                2 * hdop * self.alt_std_dev) ** 2  # FIXME

            self._publish_counted(self.fix_pub, current_fix, 'fix')
            self._log_fix(current_fix, fix_type)

            if not (math.isnan(data['utc_time'][0]) or self.use_GNSS_time):
                current_time_ref.time_ref = (current_fix.header.stamp if self.observation_timing.mode == 'measurement'
                    else rospy.Time(data['utc_time'][0], data['utc_time'][1]))
                self.last_valid_fix_time = current_time_ref
                self.time_ref_pub.publish(current_time_ref)

            # wheeltec tues debug    
            gga_msg = Gpgga()
            gga_msg.header.stamp = current_fix.header.stamp
            gga_msg.header.frame_id = frame_id
            
            #gga_msg.utc_seconds = float(data['utc_time']) if not math.isnan(data['utc_time']) else 0.0
            gga_msg.utc_seconds = float(data['utc_time'][0]) if not math.isnan(data['utc_time'][0]) else 0.0
            if self.observation_timing.mode == 'measurement':
                gga_msg.utc_seconds = current_fix.header.stamp.to_sec()
            gga_msg.lat = latitude
            gga_msg.lat_dir = data['latitude_direction']
            gga_msg.lon = longitude
            gga_msg.lon_dir = data['longitude_direction']
            gga_msg.gps_qual = data['fix_type']
            gga_msg.num_sats = data['num_satellites']
            gga_msg.hdop = data['hdop']
            gga_msg.alt = altitude
            gga_msg.altitude_units = data.get('altitude_units', 'M')  
            gga_msg.undulation = data['mean_sea_level']
            gga_msg.undulation_units = data.get('mean_sea_level_units', 'M')
            gga_msg.diff_age = int(round(data.get('age_gps_data', 0.0)))
            gga_msg.station_id = data.get('ref_station_id', '')

            self._publish_counted(self.gpgga_pub, gga_msg, 'gga')

            nmea_msg = Sentence()
            nmea_msg.header.stamp = current_time
            nmea_msg.header.frame_id = frame_id
            nmea_msg.sentence = nmea_string
            self.nmea_pub.publish(nmea_msg)
        elif not self.use_RMC and 'VTG' in parsed_sentence:
            data = parsed_sentence['VTG']

            # Only report VTG data when you've received a valid GGA fix as
            # well.
            if self.valid_fix:
                current_vel = TwistStamped()
                current_vel.header.stamp = current_time
                current_vel.header.frame_id = frame_id
                current_vel.twist.linear.x = data['speed'] * math.sin(data['true_course'])
                current_vel.twist.linear.y = data['speed'] * math.cos(data['true_course'])
                self.vel_pub.publish(current_vel)

                eagleye_current_vel = TwistWithCovarianceStamped()
                eagleye_current_vel.header.stamp = current_time
                eagleye_current_vel.header.frame_id = frame_id
                eagleye_current_vel.twist.twist.linear.x = data['speed'] * math.sin(data['true_course'])
                eagleye_current_vel.twist.twist.linear.y = data['speed'] * math.cos(data['true_course'])

                std_dev = 0.2  # m/s
                var = std_dev ** 2  # = 0.004
                eagleye_current_vel.twist.covariance = [
                var, 0.0, 0.0, 0.0, 0.0, 0.0,   # vx
                0.0, var, 0.0, 0.0, 0.0, 0.0,   # vy
                0.0, 0.0, 9999.0, 0.0, 0.0, 0.0,  # vz unknown
                0.0, 0.0, 0.0, 9999.0, 0.0, 0.0,  # roll rate
                0.0, 0.0, 0.0, 0.0, 9999.0, 0.0,  # pitch rate
                0.0, 0.0, 0.0, 0.0, 0.0, 9999.0   # yaw rate
                ]

                self.eagleye_vel_pub.publish(eagleye_current_vel)
                
        elif 'RMC' in parsed_sentence:
            data = parsed_sentence['RMC']

            if self.use_GNSS_time:
                if math.isnan(data['utc_time'][0]):
                    rospy.logwarn("Time in the NMEA sentence is NOT valid")
                    return False
                current_fix.header.stamp = rospy.Time(data['utc_time'][0], data['utc_time'][1])

            # Only publish a fix from RMC if the use_RMC flag is set.
            if self.use_RMC:
                if data['fix_valid']:
                    current_fix.status.status = NavSatStatus.STATUS_FIX
                else:
                    current_fix.status.status = NavSatStatus.STATUS_NO_FIX

                current_fix.status.service = NavSatStatus.SERVICE_GPS

                latitude = data['latitude']
                if data['latitude_direction'] == 'S':
                    latitude = -latitude
                current_fix.latitude = latitude

                longitude = data['longitude']
                if data['longitude_direction'] == 'W':
                    longitude = -longitude
                current_fix.longitude = longitude

                current_fix.altitude = float('NaN')
                current_fix.position_covariance_type = \
                    NavSatFix.COVARIANCE_TYPE_UNKNOWN

                self._publish_counted(self.fix_pub, current_fix, 'fix')
                self._log_fix(current_fix, 'RMC')

                if not (math.isnan(data['utc_time'][0]) or self.use_GNSS_time):
                    current_time_ref.time_ref = rospy.Time(
                        data['utc_time'][0], data['utc_time'][1])
                    self.time_ref_pub.publish(current_time_ref)

            # Publish velocity from RMC regardless, since GGA doesn't provide
            # it.
            if data['fix_valid']:
                current_vel = TwistStamped()
                current_vel.header.stamp = current_time
                current_vel.header.frame_id = frame_id
                current_vel.twist.linear.x = data['speed'] * \
                    math.sin(data['true_course'])
                current_vel.twist.linear.y = data['speed'] * \
                    math.cos(data['true_course'])
                self.vel_pub.publish(current_vel)
        elif 'GST' in parsed_sentence:
            data = parsed_sentence['GST']

            # Use receiver-provided error estimate if available
            self.using_receiver_epe = True
            self.lon_std_dev = data['lon_std_dev']
            self.lat_std_dev = data['lat_std_dev']
            self.alt_std_dev = data['alt_std_dev']
        elif 'HDT' in parsed_sentence:
            if self.require_heading_fixed:
                # The heading was published atomically from PVTSLNA above.
                # No temporal-nearest fallback to an unqualified HDT packet.
                return False
            data = parsed_sentence['HDT']
            heading_value = data.get('heading', None)
            if heading_value is not None and math.isfinite(heading_value) and 0 <= heading_value <= 360:
                current_heading = QuaternionStamped()
                current_heading.header.stamp = current_time
                current_heading.header.frame_id = frame_id

                # G90 HDT heading is true heading in degrees, typically
                # 0 deg = North and positive clockwise. ROS yaw uses ENU:
                # 0 deg = +X(East) and positive counter-clockwise.
                heading_north_cw_deg = heading_value + self.rotation_data
                if self.heading_source_is_north_cw:
                    yaw_deg = self._normalize_deg(90.0 - heading_north_cw_deg)
                else:
                    yaw_deg = self._normalize_deg(heading_north_cw_deg)

                yaw = math.radians(yaw_deg)
                q = quaternion_from_euler(0, 0, yaw)

                current_heading.quaternion.x = q[0]
                current_heading.quaternion.y = q[1]
                current_heading.quaternion.z = q[2]
                current_heading.quaternion.w = q[3]
                self._publish_counted(self.heading_pub, current_heading, 'heading')
                if self.debug_output:
                    rospy.loginfo_throttle(
                        1.0,
                        "G90 heading: raw_hdt=%.2fdeg install_offset=%.2fdeg north_cw=%.2fdeg ros_yaw=%.2fdeg",
                        heading_value,
                        self.rotation_data,
                        self._normalize_deg(heading_north_cw_deg),
                        yaw_deg,
                    )

                #wheeltec_tues debug
                pose_msg = PoseStamped()
                pose_msg.header.stamp = current_time
                pose_msg.header.frame_id = frame_id
                pose_msg.pose.position.x = current_fix.latitude
                pose_msg.pose.position.y = current_fix.longitude
                pose_msg.pose.position.z = current_fix.altitude
                pose_msg.pose.orientation.x = q[0]
                pose_msg.pose.orientation.y = q[1]
                pose_msg.pose.orientation.z = q[2]
                pose_msg.pose.orientation.w = q[3]
                self.pose_pub.publish(pose_msg)      

                heading_deg_msg = Float32() 
                heading_deg_msg.data = self._normalize_deg(heading_north_cw_deg)
                self.heading_deg_pub.publish(heading_deg_msg)          
        else:
            return False

    @staticmethod
    def get_frame_id():
        """Get the TF frame_id.

        Queries rosparam for the ~frame_id param. If a tf_prefix param is set,
        the frame_id is prefixed with the prefix.

        Returns:
            str: The fully-qualified TF frame ID.
        """
        frame_id = rospy.get_param('~frame_id', 'gps')
        # Add the TF prefix
        prefix = ""
        prefix_param = rospy.search_param('tf_prefix')
        if prefix_param:
            prefix = rospy.get_param(prefix_param)
            return "%s/%s" % (prefix, frame_id)
        else:
            return frame_id
