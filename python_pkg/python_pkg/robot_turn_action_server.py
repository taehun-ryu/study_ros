#!/usr/bin/env/ python3
#
# Copyright 2021 Seoul Business Agency Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import time
import numpy as np

from study_msgs.action import RobotTurn
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from python_pkg.image_capture import ImageSubscriber

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

direction_dict = {0: 0.0 , 1: (-1 * math.pi / 2), 2: math.pi, 3: math.pi / 2}
direction_str_dict = {0: 'North', 1: 'East', 2: 'South', 3: 'West'}

# RobotTurn.action structure

#     int32[] turning_sequence
#     ---
#     bool success
#     ---
#     string feedback_msg

#    https://gist.github.com/salmagro/2e698ad4fbf9dae40244769c5ab74434
def euler_from_quaternion(quaternion):
    """
    Return Converted euler roll, pitch, yaw from quaternion (w in last place).

    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class RobotTurnActionServer(Node):

    def __init__(self):
        super().__init__('robot_turn_action_server')

        self.yaw = 0.0
        self.forward_distance = 0.0

        self.twist_msg = Twist()
        self.loop_rate = self.create_rate(5, self.get_clock())

        self.odom_sub = self.create_subscription(
            Odometry, 'diffbot/odom', self.odom_sub_cb, 10
        )

        self.cmd_vel_pub = self.create_publisher(Twist, 'diffbot/cmd_vel', 10)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_callback)

        self._action_server = ActionServer(
            self, RobotTurn, 'diffbot/robot_turn_action', self.execute_callback
        )
        self.get_logger().info('=== Robot Turn Action Server Started ====')

    def odom_sub_cb(self, data):
        orientation = data.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion(orientation)
        # self.get_logger().info(f'yaw : {self.yaw}')

    def publish_callback(self):
        self.cmd_vel_pub.publish(self.twist_msg)

    def turn_robot(self, euler_angle):
        self.get_logger().info(f'Robot Turns to {euler_angle}')

        turn_offset = 100

        while abs(turn_offset) > 0.087:
            turn_offset = 0.7 * (euler_angle - self.yaw)
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = turn_offset
            self.cmd_vel_pub.publish(self.twist_msg)

        self.stop_robot()

    def go_during_second(self, second):
        self.twist_msg.linear.x = 0.5
        self.twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist_msg)

        time.sleep(second)
        self.stop_robot()

    def stop_robot(self):
        self.twist_msg.linear.x = 0.0
        self.twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist_msg)

        time.sleep(1)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback = RobotTurn.Feedback()
        feedback.feedback_msg = ''

        for _, val in enumerate(goal_handle.request.turning_sequence):
            self.get_logger().info(f'Current Cmd: {val}')

            feedback.feedback_msg = f'Look {direction_str_dict[val]}'

            self.turn_robot(direction_dict[val])
            self.go_during_second(2)

            goal_handle.publish_feedback(feedback)

        image_sub_node = ImageSubscriber()
        rclpy.spin_once(image_sub_node)
        center_pixel = image_sub_node.center_pixel

        print(sum(center_pixel), center_pixel[1])
        # 195 63

        # 주황색 벽을 만나면 성공
        if sum(center_pixel) < 220 and center_pixel[1] > 40:
            goal_handle.succeed()
            self.get_logger().warn('==== Succeed ====')
            result = RobotTurn.Result()
            result.success = True
        else:
            goal_handle.abort()
            self.get_logger().error('==== Fail ====')
            result = RobotTurn.Result()
            result.success = False

        return result


def main(args=None):
    rclpy.init(args=args)

    # Referenced from robotpilot/ros2-seminar-examples
    # https://github.com/robotpilot/ros2-seminar-examples/blob/main/topic_service_action_rclpy_example/topic_service_action_rclpy_example/calculator/main.py
    try:
        robot_turn_action_server = RobotTurnActionServer()
        executor = MultiThreadedExecutor()
        executor.add_node(robot_turn_action_server)
        try:
            executor.spin()
        except KeyboardInterrupt:
            robot_turn_action_server.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            executor.shutdown()
            robot_turn_action_server._action_server.destroy()
            robot_turn_action_server.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()