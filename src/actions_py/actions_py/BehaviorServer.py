# !/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from my_robot_interfaces.action import Circle, TurnInPlace, StraightLine, FigureEight
from my_robot_interfaces.msg import WheelCmd
from .ControlLaw_class import ControlLaw, Parameters, Control, State
import time
import math
import subprocess
import signal

class BehaviorServer(Node):
    def __init__(self):
        super().__init__("behavior_server")

        self.cb_group = ReentrantCallbackGroup()

        self.circle_server_ = ActionServer(self, Circle, "circle", execute_callback=self.circle, callback_group=self.cb_group)
        self.turn_in_place_server_ = ActionServer(self, TurnInPlace, "turn_in_place", execute_callback=self.turnIP, callback_group=self.cb_group)
        self.line_server_ = ActionServer(self, StraightLine, "straight_line", execute_callback=self.line, callback_group=self.cb_group)
        self.figure_eight_server_ = ActionServer(self, FigureEight, "figure_eight", execute_callback=self.figure8, callback_group=self.cb_group)

        self.wheel_cmd_pub_ = self.create_publisher(WheelCmd, 'wheel_cmd', 10)
        
        self.latest_state_ = State(0,0,0,0,0)
        self.odom_sub_ = self.create_subscription(Odometry, "/odometry/filtered", self.odom_callback, 10,
                                                  callback_group=self.cb_group)


        self.uncalibrated_params = Parameters(
            r_R=0.03,      # 3 cm wheel radius — measure this directly, don't guess
            r_L=0.03,      # same — measure this directly
            l=0.105,        # 8 cm center-to-wheel distance — measure this directly
            kg_R=25.0,
            kg_L=25.0,
            ka=0.5,
            kf=0.15,
            kq=0.02,
            alpha=8.0,
            beta=8.0
            )
    
        self.logger_proc = None
        

    def odom_callback(self, msg:Odometry):
        q = msg.pose.pose.orientation
        theta = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self.latest_state_ = State(0.0, 0.0, theta, 0.0, 0.0)

    def start_logger(self, shape_name):
        baseline = self.wheel_cmd_pub_.get_subscription_count()
        filename = f"{shape_name}_{int(time.time())}.csv"
        self.logger_proc = subprocess.Popen([
            'ros2', 'run', 'my_robot_localization', 'trajectory_logger',
            '--ros-args', '-p', f'output_file:={filename}'
        ])
        self.get_logger().info(f"Logging to {filename}")

        # block until the new subscriber actually connects to /wheel_cmd
        timeout = 5.0
        start = time.time()
        while self.wheel_cmd_pub_.get_subscription_count() <= baseline and (time.time() - start) < timeout:
            time.sleep(0.1)

    def stop_logger(self):
        if self.logger_proc is not None:
            self.logger_proc.send_signal(signal.SIGINT)
            try:
                self.logger_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.logger_proc.kill()
            self.logger_proc = None

    def circle(self, goal_handle):
        
        controller = ControlLaw(goal_handle.request.velocity, goal_handle.request.radius, self.uncalibrated_params)
        empty_state = State(0,0,0,0,0)

        cmd = WheelCmd()

        self.start_logger("circle")
        start_time = self.get_clock().now()
        try:
            while (self.get_clock().now() - start_time).nanoseconds / 1e9  < goal_handle.request.duration:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    cmd.right_pwm = 0.0
                    cmd.left_pwm = 0.0
                    result = Circle.Result()
                    result.success = False
                    result.message = "Circle Cancelled"
                    return result

                control = controller.circle(empty_state)
                cmd.right_pwm = control.u_R
                cmd.left_pwm = control.u_L
                self.wheel_cmd_pub_.publish(cmd)
                time.sleep(0.05)

            cmd.right_pwm = 0.0
            cmd.left_pwm = 0.0
            self.wheel_cmd_pub_.publish(cmd)

            result = Circle.Result()
            result.message = ""
            result.success = True
            goal_handle.succeed()
            return result
        finally:
            self.stop_logger()
   
    def turnIP(self, goal_handle):
        pass

    def line(self, goal_handle):
        
        controller = ControlLaw(goal_handle.request.velocity, 0, self.uncalibrated_params)
        empty_state = State(0,0,0,0,0)

        cmd = WheelCmd()
        self.start_logger("line")
        start_time = self.get_clock().now()
        
        try:
            while (self.get_clock().now() - start_time).nanoseconds / 1e9  < goal_handle.request.duration:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    cmd.right_pwm = 0.0
                    cmd.left_pwm = 0.0
                    result = StraightLine.Result()
                    result.success = False
                    result.message = "Straight Line Cancelled"
                    return result

                control = controller.straight_line(empty_state)
                cmd.right_pwm = control.u_R
                cmd.left_pwm = control.u_L
                self.wheel_cmd_pub_.publish(cmd)
                time.sleep(0.05)

            cmd.right_pwm = 0.0
            cmd.left_pwm = 0.0
            self.wheel_cmd_pub_.publish(cmd)

            result = StraightLine.Result()
            result.message = ""
            result.success = True
            goal_handle.succeed()
            return result
        finally:
            self.stop_logger()

    def figure8(self, goal_handle):
        
        controller = ControlLaw(goal_handle.request.velocity, goal_handle.request.radius, self.uncalibrated_params)
        

        cmd = WheelCmd()

        self.start_logger("figure_eight")
        start_time = self.get_clock().now()
        try:
            while (self.get_clock().now() - start_time).nanoseconds / 1e9  < goal_handle.request.duration:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    cmd.right_pwm = 0.0
                    cmd.left_pwm = 0.0
                    result = FigureEight.Result()
                    result.success = False
                    result.message = "Figure8 Cancelled"
                    return result

                control = controller.figure_8(self.latest_state_)
                cmd.right_pwm = control.u_R
                cmd.left_pwm = control.u_L
                self.wheel_cmd_pub_.publish(cmd)
                time.sleep(0.05)

            cmd.right_pwm = 0.0
            cmd.left_pwm = 0.0
            self.wheel_cmd_pub_.publish(cmd)

            result = FigureEight.Result()
            result.message = ""
            result.success = True
            goal_handle.succeed()
            return result
        finally:
            self.stop_logger()

def main(args=None):
    rclpy.init(args=args)
    node = BehaviorServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()

if __name__ == "__main__":
    main()