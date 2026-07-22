#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
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
        self.enable_localization_safety_stop = rospy.get_param(
            '~enable_localization_safety_stop', False
        )
        self.localization_safety_topic = rospy.get_param(
            '~localization_safety_topic', '/localization_safe'
        )
        self.localization_safe = not self.enable_localization_safety_stop
        self.print_sent_command = rospy.get_param('~print_sent_command', False)

        # 初始化串口
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=0.1)
            rospy.loginfo(f"Connected to serial port {self.port}")
        except Exception as e:
            self.serial = None
            rospy.logerr(f"Failed to connect to serial port: {e}")

        if self.enable_localization_safety_stop:
            self.send_serial_command(0.0, 0.0)
            rospy.logwarn("Localization safety gate enabled; holding stop until localization is ready")

        # 订阅上层规划器的速度指令
        rospy.Subscriber('/neupan_cmd_vel', Twist, self.cmd_vel_callback)
        if self.enable_localization_safety_stop:
            rospy.Subscriber(
                self.localization_safety_topic, Bool, self.localization_safety_callback
            )
        
        # 发布里程计
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
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

        self.last_odom_x = None
        self.last_odom_y = None
        self.last_odom_stamp = None
        self.odom_speed_mps = 0.0

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

        if self.enable_localization_safety_stop and not self.localization_safe:
            v = 0.0
            steering_angle = 0.0
            rospy.logerr_throttle(
                1.0, "Localization unsafe: blocking planner command and holding stop"
            )

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
        self.send_serial_command(throttle, steering_deg)

    def localization_safety_callback(self, msg):
        was_safe = self.localization_safe
        self.localization_safe = bool(msg.data)
        if was_safe and not self.localization_safe:
            rospy.logerr("Localization became unsafe; sending immediate stop to MCU")
            self.send_serial_command(0.0, 0.0)
        elif not was_safe and self.localization_safe:
            rospy.loginfo("Localization recovered; planner commands are enabled")

    def send_serial_command(self, throttle, steering_deg):
        command = f"CMD,{throttle:.2f},{steering_deg:.2f}\n"
        if self.serial and self.serial.is_open:
            try:
                self.serial.write(command.encode())
                if self.print_sent_command:
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
