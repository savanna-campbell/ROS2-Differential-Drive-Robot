# !/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import WheelCmd
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry
import serial
import threading
import time
import math

class ArduinoBridge(Node):
    def __init__(self):
        # name node
        super().__init__("arduino_bridge")
        # set up serial
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

        time.sleep(2)
        self.ser.reset_input_buffer()

        self.ser.write(b'START_STREAM 100\n')
        
        # setup odometry variables
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        self.prev_time = None
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.wheel_radius = .03
        self.l = 0.105

        # set up subscriber
        self.subscriber_ = self.create_subscription(WheelCmd, "/wheel_cmd", self.wheel_callback, 10)
        
        # set up publisher
        self.imu_publisher_ = self.create_publisher(Imu, "/imu/data", 10)
        self.gps_publisher_ = self.create_publisher(NavSatFix, "/gps/fix", 10)
        self.odom_publisher_ = self.create_publisher(Odometry, "/wheel/odometry", 10)

        self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.ser_lock = threading.Lock()

        self.read_thread.start()

    def read_loop(self):
        ACCEL_SCALE = 16384.0
        GYRO_SCALE = 131.0
        DEG_TO_RAD = math.pi / 180.0
        G_TO_MS2 = 9.80665


        while rclpy.ok():
            with self.ser_lock:
                line = self.ser.readline().decode(errors="ignore").strip()
            if not line.startswith("STATE"):
                continue
            fields = line[len("STATE "):].split(",")
            if len(fields) != 12:
                continue
            #STATE time_ms left_count right_count ax ay az gx gy gz lat lon gps_fix
            try:
                time_ms, lt, rt, ax, ay, az, gx, gy, gz, lat, lon, fix = fields
                time_ms = int(time_ms)
                lt, rt = int(lt), int(rt)
                ax, ay, az = int(ax), int(ay), int(az)
                gx, gy, gz = int(gx), int(gy), int(gz)
                lat, lon = float(lat), float(lon)
                fix = int(fix)
            except ValueError:
                continue

            now = self.get_clock().now().to_msg()
            
            # imu setup
            imu_msg = Imu()
            imu_msg.header.stamp = now
            imu_msg.header.frame_id = "imu_link"

            imu_msg.angular_velocity.x = (gx / GYRO_SCALE) * DEG_TO_RAD
            imu_msg.angular_velocity.y = (gy / GYRO_SCALE) * DEG_TO_RAD
            imu_msg.angular_velocity.z = (gz / GYRO_SCALE) * DEG_TO_RAD

            imu_msg.linear_acceleration.x = (ax / ACCEL_SCALE) * G_TO_MS2
            imu_msg.linear_acceleration.y = (ay / ACCEL_SCALE) * G_TO_MS2
            imu_msg.linear_acceleration.z = (az / ACCEL_SCALE) * G_TO_MS2

            imu_msg.orientation_covariance[0] = -1.0

            self.imu_publisher_.publish(imu_msg)

            # gps setup
            gps_msg = NavSatFix()
            gps_msg.header.stamp = now
            gps_msg.header.frame_id = "gps_link"

            gps_msg.status.status = NavSatStatus.STATUS_FIX if fix else NavSatStatus.STATUS_NO_FIX
            gps_msg.status.service = NavSatStatus.SERVICE_GPS

            gps_msg.latitude = lat
            gps_msg.longitude = lon
            gps_msg.altitude = float('nan')

            self.gps_publisher_.publish(gps_msg)

            odom_msg = self.compute_odometry(lt, rt, time_ms)
            if odom_msg is not None:
                self.odom_publisher_.publish(odom_msg)
        
    def compute_odometry(self, lt, rt, time_ms):

        
        if self.prev_time is None:
            self.prev_time = time_ms
            self.prev_left_ticks = lt
            self.prev_right_ticks = rt
            return None

        # calculate omega_L and omega_R
        left_dist_change = (lt - self.prev_left_ticks) * ((2 * self.wheel_radius * math.pi) / 1080) 
        right_dist_change = (rt - self.prev_right_ticks) * ((2 * self.wheel_radius * math.pi) / 1080)

        dt = (time_ms - self.prev_time) / 1000.0

        left_vel = left_dist_change / dt
        right_vel = right_dist_change / dt

        # change in x, y, z

        change_x = math.cos(self.theta) * 1/2 * (left_vel + right_vel)
        change_y = math.sin(self.theta) * 1/2 * (left_vel + right_vel)
        change_theta = 1/(2 * self.l) * (right_vel - left_vel)

        self.x = self.x + dt * change_x
        self.y = self.y + dt * change_y
        self.theta = self.theta + dt * change_theta

        # update vars
        self.prev_left_ticks = lt
        self.prev_right_ticks = rt
        self.prev_time = time_ms

        # create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"


        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        odom_msg.twist.twist.linear.x = (left_vel + right_vel) / 2.0
        odom_msg.twist.twist.angular.z = change_theta

        odom_msg.pose.covariance[0] = 0.05   # x
        odom_msg.pose.covariance[7] = 0.05   # y
        odom_msg.pose.covariance[35] = 0.1   # yaw
        odom_msg.twist.covariance[0] = 0.05  # vx
        odom_msg.twist.covariance[35] = 0.1  # vyaw

        return odom_msg


    def wheel_callback(self, msg):
        MAX_PWM = 255

        left_pwm = max(-MAX_PWM, min(MAX_PWM, int(msg.left_pwm * MAX_PWM)))
        right_pwm = max(-MAX_PWM, min(MAX_PWM, int(msg.right_pwm * MAX_PWM)))

        command = f"SET_MOTORS {left_pwm} {right_pwm}\n"
        with self.ser_lock:
            self.ser.write(command.encode())
        self.get_logger().info(f"sent: {command.strip()}")

        
        
def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()