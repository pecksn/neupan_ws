#!/usr/bin/env python
import rospy
import tf
from collections import deque
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
import serial # 需要安装 pyserial
import math

class RobotBaseDriver:
    def __init__(self):
        rospy.init_node('robot_base_driver')

        # 参数
        self.port = rospy.get_param('~port', '/dev/ttyUSB0')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.gps_frame = rospy.get_param('~gps_frame', 'gps_link')
        self.gps_cov_xy = rospy.get_param('~gps_cov_xy', 9.0)
        self.gps_cov_z = rospy.get_param('~gps_cov_z', 25.0)
        self.gps_jump_reject_m = rospy.get_param('~gps_jump_reject_m', 30.0)
        self.gps_max_speed_mps = rospy.get_param('~gps_max_speed_mps', 20.0)
        self.gps_filter_window = max(1, int(rospy.get_param('~gps_filter_window', 5)))
        self.gps_stationary_speed_mps = max(0.01, float(rospy.get_param('~gps_stationary_speed_mps', 0.15)))
        self.gps_stationary_jump_m = max(0.2, float(rospy.get_param('~gps_stationary_jump_m', 1.5)))
        self.gps_cov_stationary_scale = max(1.0, float(rospy.get_param('~gps_cov_stationary_scale', 6.0)))
        self.gps_ema_alpha = float(rospy.get_param('~gps_ema_alpha', 0.2))
        self.gps_ema_alpha = min(max(self.gps_ema_alpha, 0.01), 1.0)
        
        # 机器人运动学参数 (Ackermann)
        self.wheelbase = rospy.get_param('~wheelbase', 0.67) # 轴距 (m)
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.1275) # 轮半径 (m)

        # 初始化串口
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=0.1)
            rospy.loginfo(f"Connected to serial port {self.port}")
        except Exception as e:
            self.serial = None
            rospy.logerr(f"Failed to connect to serial port: {e}")

        # 订阅上层规划器的速度指令
        rospy.Subscriber('/neupan_cmd_vel', Twist, self.cmd_vel_callback)
        
        # 发布里程计和GPS
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.gps_pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        # 机器人状态
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = rospy.Time.now()
        
        # 里程计零点偏移,用于初始化
        self.offset_init = False
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.offset_th = 0.0

        self.last_gps_lat = None
        self.last_gps_lon = None
        self.last_gps_stamp = None
        self.last_pub_gps_lat = None
        self.last_pub_gps_lon = None
        self.gps_filter_buf = deque(maxlen=self.gps_filter_window)
        self.last_odom_x = None
        self.last_odom_y = None
        self.last_odom_stamp = None
        self.odom_speed_mps = 0.0

    def _parse_gps_lat_lon(self, parts):
        """
        支持两种常见格式：
        1) GPS,<lat>,<lon>[, ...]
        2) GPS,<lat>,N/S,<lon>,E/W[, ...]
        """
        if len(parts) < 3:
            raise ValueError('GPS fields not enough')

        # 格式2: GPS,39.9,N,116.4,E
        if len(parts) >= 5 and parts[2].upper() in ('N', 'S') and parts[4].upper() in ('E', 'W'):
            lat = float(parts[1])
            lon = float(parts[3])

            if parts[2].upper() == 'S':
                lat = -abs(lat)
            else:
                lat = abs(lat)

            if parts[4].upper() == 'W':
                lon = -abs(lon)
            else:
                lon = abs(lon)
        else:
            # 格式1: GPS,39.9,116.4 (后续附加字段忽略)
            lat = float(parts[1])
            lon = float(parts[2])

        if abs(lat) > 90.0 or abs(lon) > 180.0:
            raise ValueError(f'GPS lat/lon out of range: lat={lat}, lon={lon}')

        return lat, lon

    def _haversine_distance_m(self, lat1, lon1, lat2, lon2):
        r = 6378137.0
        p1 = math.radians(lat1)
        p2 = math.radians(lat2)
        dlat = p2 - p1
        dlon = math.radians(lon2 - lon1)
        a = math.sin(dlat / 2.0) ** 2 + math.cos(p1) * math.cos(p2) * (math.sin(dlon / 2.0) ** 2)
        c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
        return r * c

    def _smooth_lat_lon(self, lat, lon):
        self.gps_filter_buf.append((lat, lon))
        if len(self.gps_filter_buf) <= 1:
            return lat, lon

        lat_mean = sum(p[0] for p in self.gps_filter_buf) / float(len(self.gps_filter_buf))
        lon_mean = sum(p[1] for p in self.gps_filter_buf) / float(len(self.gps_filter_buf))
        return lat_mean, lon_mean

    def _ema_lat_lon(self, lat, lon):
        if self.last_pub_gps_lat is None or self.last_pub_gps_lon is None:
            return lat, lon

        alpha = self.gps_ema_alpha
        lat_ema = alpha * lat + (1.0 - alpha) * self.last_pub_gps_lat
        lon_ema = alpha * lon + (1.0 - alpha) * self.last_pub_gps_lon
        return lat_ema, lon_ema

    def cmd_vel_callback(self, msg):
        """
        接收 /neupan_cmd_vel，转换为下位机指令并通过串口发送
        协议格式: CMD,<油门>,<转向>
        油门 (Throttle): rad/s
        转向 (Steering): deg
        """
        v = msg.linear.x
        
        # 【修改】Neupan 直接输出的是转向角 (rad)
        steering_angle = msg.angular.z 

        # 【新增】电机死区处理：过滤掉过小的速度，防止电机异响和震荡
        dead_zone = 0.05 # 5cm/s
        if abs(v) < dead_zone:
             v = 0.0
             # 处于静止状态时，强制回正方向，避免原地左右摆头
             steering_angle = 0.0
        
        # 计算轮子转速 (rad/s)
        throttle = v / self.wheel_radius

        # 转换为角度
        steering_deg = math.degrees(steering_angle)

        # 【修复】增加转向角物理限幅 (例如 +/- 33度)，防止出现 90度的无效指令
        max_steering_deg = 25.0 
        if steering_deg > max_steering_deg:
            steering_deg = max_steering_deg
        elif steering_deg < -max_steering_deg:
            steering_deg = -max_steering_deg
        
        # 构造指令: CMD,<油门>,<转向>
        command = f"CMD,{throttle:.2f},{steering_deg:.2f}\n"
        
        if self.serial and self.serial.is_open:
            try:
                self.serial.write(command.encode())
                rospy.loginfo_throttle(1, f"Send to MCU: {command.strip()}")
            except Exception as e:
                rospy.logerr(f"Serial write failed: {e}")

    def read_from_mcu(self):
        """
        从下位机读取数据（里程计）
        格式: X Y Heading (ASCII, 空格或逗号分隔)
        示例: 1.234 0.567 180.5
        """
        if not self.serial or not self.serial.is_open:
            return

        try:
            # 读取所有等待的数据，只处理最新的一行，或者按行处理
            if self.serial.in_waiting:
                line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    return
                
                # 解析数据: X(m) Y(m) Heading(deg)
                # 兼容空格或逗号分隔
                # 示例数据: ODO,0.000,0.000,13.199
                if line.startswith("ODO"):
                    parts = line.replace(',', ' ').split()
                    # parts[0] 是 "ODO", parts[1] 是 X, parts[2] 是 Y, parts[3] 是 Heading
                    if len(parts) >= 4:
                        try:
                            raw_x = float(parts[1])
                            raw_y = float(parts[2])
                            raw_heading_deg = float(parts[3])
                            raw_th = math.radians(raw_heading_deg)

                            if not self.offset_init:
                                self.offset_x = raw_x
                                self.offset_y = raw_y
                                self.offset_th = raw_th
                                self.offset_init = True
                            
                            # 计算相对位移
                            dx = raw_x - self.offset_x
                            dy = raw_y - self.offset_y
                            
                            # 旋转到初始朝向坐标系 (Odom Frame X轴朝前)
                            # 逆时针旋转 -offset_th
                            c_th = math.cos(-self.offset_th)
                            s_th = math.sin(-self.offset_th)
                            
                            self.x = dx * c_th - dy * s_th
                            self.y = dx * s_th + dy * c_th
                            self.th = raw_th - self.offset_th
                            
                            # self.x = x_m
                            # self.y = y_m
                            # self.th = math.radians(heading_deg) # 转换为弧度

                            current_time = rospy.Time.now()

                            # 用里程计位置差分估计当前速度，用于静止时GPS抖动抑制
                            if self.last_odom_stamp is not None:
                                dt_odom = (current_time - self.last_odom_stamp).to_sec()
                                if dt_odom > 1e-3:
                                    ds = math.hypot(self.x - self.last_odom_x, self.y - self.last_odom_y)
                                    self.odom_speed_mps = ds / dt_odom
                            self.last_odom_x = self.x
                            self.last_odom_y = self.y
                            self.last_odom_stamp = current_time

                            # 发布 TF: odom -> base_link
                            quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
                            # 【修改】这里恢复直接发布TF，因为你的 launch 文件中没有启动 EKF
                            if rospy.get_param("~publish_tf", False):
                                self.tf_broadcaster.sendTransform(
                                (self.x, self.y, 0),
                                quat,
                                current_time,
                                self.base_frame,
                                self.odom_frame
                            )

                            # 发布 Odom 消息
                            odom = Odometry()
                            odom.header.stamp = current_time
                            odom.header.frame_id = self.odom_frame
                            odom.child_frame_id = self.base_frame
                            odom.pose.pose.position.x = self.x
                            odom.pose.pose.position.y = self.y
                            odom.pose.pose.orientation = Quaternion(*quat)
                            
                            # 由于下位机只发位置，这里速度设为0或通过差分计算(略)
                            odom.twist.twist.linear.x = 0.0
                            odom.twist.twist.angular.z = 0.0
                            
                            # 【新增】给定里程计的固定协方差，供 EKF 融合使用
                            odom.pose.covariance = [
                                0.05, 0, 0, 0, 0, 0,
                                0, 0.05, 0, 0, 0, 0,
                                0, 0, 99999, 0, 0, 0,
                                0, 0, 0, 99999, 0, 0,
                                0, 0, 0, 0, 99999, 0,
                                0, 0, 0, 0, 0, 0.05
                            ]
                            odom.twist.covariance = odom.pose.covariance
                            
                            self.odom_pub.publish(odom)
                            self.last_time = current_time
                            
                        except ValueError:
                            rospy.logwarn(f"Invalid data format from MCU: {line}")
                            
                # 【新增】解析下位机发上来的 GPS 数据
                elif line.startswith("GPS"):
                    parts = line.replace(',', ' ').split()
                    if len(parts) >= 3:
                        try:
                            lat, lon = self._parse_gps_lat_lon(parts)
                            gps_stamp = rospy.Time.now()

                            # 抑制明显不合理的 GPS 突跳，避免 map 系抖动
                            if self.last_gps_lat is not None and self.last_gps_stamp is not None:
                                dt = (gps_stamp - self.last_gps_stamp).to_sec()
                                if dt > 1e-3:
                                    jump_m = self._haversine_distance_m(self.last_gps_lat, self.last_gps_lon, lat, lon)
                                    speed_mps = jump_m / dt
                                    reject_hard_jump = jump_m > (2.0 * self.gps_jump_reject_m)
                                    reject_jump_with_speed = jump_m > self.gps_jump_reject_m and speed_mps > self.gps_max_speed_mps
                                    if reject_hard_jump or reject_jump_with_speed:
                                        rospy.logwarn_throttle(
                                            1.0,
                                            f"Reject GPS jump ({jump_m:.1f}m, {speed_mps:.1f}m/s): {line}"
                                        )
                                        return

                            # 车辆近似静止时，拒绝大幅GPS漂移，避免 map->odom 被持续拉扯
                            if self.last_pub_gps_lat is not None and self.odom_speed_mps < self.gps_stationary_speed_mps:
                                drift_m = self._haversine_distance_m(
                                    self.last_pub_gps_lat,
                                    self.last_pub_gps_lon,
                                    lat,
                                    lon,
                                )
                                if drift_m > self.gps_stationary_jump_m:
                                    rospy.logwarn_throttle(
                                        1.0,
                                        (
                                            f"Reject stationary GPS drift ({drift_m:.2f}m > "
                                            f"{self.gps_stationary_jump_m:.2f}m), odom speed={self.odom_speed_mps:.2f}m/s"
                                        ),
                                    )
                                    return

                            lat_s, lon_s = self._smooth_lat_lon(lat, lon)
                            lat_s, lon_s = self._ema_lat_lon(lat_s, lon_s)

                            # 静止时增大协方差，降低GPS在全局融合里的拉扯力度
                            cov_scale = self.gps_cov_stationary_scale if self.odom_speed_mps < self.gps_stationary_speed_mps else 1.0
                            cov_xy = self.gps_cov_xy * cov_scale
                            cov_z = self.gps_cov_z * cov_scale

                            gps_msg = NavSatFix()
                            gps_msg.header.stamp = gps_stamp
                            gps_msg.header.frame_id = self.gps_frame

                            gps_msg.latitude = lat_s
                            gps_msg.longitude = lon_s
                            gps_msg.altitude = 0.0  # 如果有高度也可以解析
                            
                            # 设置正常状态
                            gps_msg.status.status = 0
                            gps_msg.status.service = 1
                            
                            # GPS 协方差可参数化，便于室外调小/调大融合强度
                            gps_msg.position_covariance = [
                                cov_xy, 0.0, 0.0,
                                0.0, cov_xy, 0.0,
                                0.0, 0.0, cov_z
                            ]
                            gps_msg.position_covariance_type = 1 # 1: COVARIANCE_TYPE_APPROXIMATED
                            
                            self.gps_pub.publish(gps_msg)
                            self.last_gps_lat = lat
                            self.last_gps_lon = lon
                            self.last_gps_stamp = gps_stamp
                            self.last_pub_gps_lat = lat_s
                            self.last_pub_gps_lon = lon_s
                        except ValueError:
                            rospy.logwarn(f"Invalid GPS data from MCU: {line}")
                else:
                    # 忽略其他非 ODO 开头的数据，避免刷屏警告
                    pass

        except Exception as e:
            rospy.logwarn(f"Serial read error: {e}")

    def run(self):
        rate = rospy.Rate(20) # 20Hz
        while not rospy.is_shutdown():
            self.read_from_mcu()
            rate.sleep()

if __name__ == '__main__':
    try:
        driver = RobotBaseDriver()
        driver.run()
    except rospy.ROSInterruptException:
        pass
