#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from my_robot_interfaces.msg import WheelCmd
import math, csv

class TrajLogger(Node):

    def __init__(self):
        super().__init__("trajectory_logger")

        self.declare_parameter('output_file', 'traj.csv')
        output_file = self.get_parameter('output_file').value

        self.r = 0.03    # MUST match arduino_bridge's wheel_radius exactly
        self.l = 0.105    # MUST match arduino_bridge's self.l exactly (currently 0.15 there)
        self.u_R = 0.0
        self.u_L = 0.0
        self.omega_R = 0.0
        self.omega_L = 0.0

        self.f = open(output_file, 'w', newline='')
        self.writer = csv.writer(self.f)
        self.writer.writerow(['time', 'x', 'y', 'theta', 'omega_R', 'omega_L', 'u_R', 'u_L'])

        self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, 50)
        self.create_subscription(Odometry, '/wheel/odometry', self.wheel_odom_cb, 50)
        self.create_subscription(WheelCmd, '/wheel_cmd', self.cmd_cb, 10)

        self.get_logger().info(f"Logging trajectory to {output_file}")

    def cmd_cb(self, msg):
        self.u_R = msg.right_pwm
        self.u_L = msg.left_pwm

    def wheel_odom_cb(self, msg):
        # raw wheel-derived velocity 
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        right_vel = v + self.l * w
        left_vel = v - self.l * w
        self.omega_R = right_vel / self.r
        self.omega_L = left_vel / self.r

    def odom_cb(self, msg):
        # EKF-fused pose 
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        theta = 2 * math.atan2(qz, qw)

        self.writer.writerow([t, x, y, theta, self.omega_R, self.omega_L, self.u_R, self.u_L])
        self.f.flush()


def main(args=None):
    rclpy.init(args=args)
    node = TrajLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.f.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()