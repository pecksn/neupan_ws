#!/usr/bin/env python
import rospy
import tf
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

        # 【修复】增加转向角物理限幅 (例如 +/- 30度)，防止出现 90度的无效指令
        max_steering_deg = 30.0 
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
                    # 假设格式如: GPS,39.12345,116.12345 (Latitude, Longitude)
                    if len(parts) >= 3:
                        try:
                            gps_msg = NavSatFix()
                            gps_msg.header.stamp = rospy.Time.now()
                            gps_msg.header.frame_id = "gps_link"
                            
                            # 通常 GPS 数据格式中第二个是纬度，第三个是经度
                            lat = float(parts[1])
                            lon = float(parts[2])
                            
                            # 【处理南纬/西经的加负号问题】
                            # 假设下位机发上来的格式是: GPS,纬度值,N/S,经度值,E/W
                            # 例如: GPS,39.9,N,116.4,E 或 GPS,39.9,S,116.4,W
                            # 如果下位机发的是带方向字母的格式，则进行智能判断：
                            if len(parts) >= 5:
                                lat_dir = parts[2].upper()
                                lon_raw = float(parts[3])
                                lon_dir = parts[4].upper()
                                if lat_dir == 'S': lat = -lat
                                if lon_dir == 'W': lon = -lon_raw
                                else: lon = lon_raw
                            else:
                                # 如果下位机没发字母，直接认为它自己在内部已经加了负号
                                pass
                                
                            gps_msg.latitude = lat
                            gps_msg.longitude = lon
                            gps_msg.altitude = 0.0  # 如果有高度也可以解析
                            
                            # 设置正常状态
                            gps_msg.status.status = 0
                            gps_msg.status.service = 1
                            
                            # 设置合理的协方差 (单位: m^2) 保证 Navsat 会信任它
                            gps_msg.position_covariance = [
                                2.0, 0.0, 0.0,
                                0.0, 2.0, 0.0,
                                0.0, 0.0, 5.0
                            ]
                            gps_msg.position_covariance_type = 1 # 1: COVARIANCE_TYPE_APPROXIMATED
                            
                            self.gps_pub.publish(gps_msg)
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
