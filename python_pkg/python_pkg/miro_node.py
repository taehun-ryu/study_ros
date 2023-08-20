# !/usr/bin/env/ python3

import cv2
import numpy as np
import time
import math

from cv_bridge import (
    CvBridge,
    CvBridgeError,
) 

from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from python_pkg.image_capture import ImageSubscriber

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

DIRECTION_DICT = {0: 0.0 , 1: (-1 * math.pi / 2), 2: math.pi, 3: math.pi / 2}

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

"""
# Request
string robot_service
---
# Response
bool success
"""

"""
# Request
bool do_available
---
# Response
bool success
"""

class Miro(Node):

    def __init__(self):
        super().__init__('miro_node')
        # 현재 로봇의 상태
        self.yaw = 0.0
        self.turn_count = 3

        self.turn_flag = False
        self.turn_value = 0
        self.action_success = True

        self.done = True
        self.center_pixel = None

        self.twist_msg = Twist()
        # 오도메트리 sub
        self.odom_sub = self.create_subscription(
            Odometry, 'diffbot/odom', self.odom_sub_callback, 10
        )
        # 라이더 sub
        queue_size = 10
        self.subscriber = self.create_subscription(
            LaserScan, 'diffbot/scan', self.subscribeCallBack, queue_size
        )
        # 로봇 mv pub
        self.cmd_vel_pub = self.create_publisher(Twist, 'diffbot/cmd_vel', 10)
        self.timer_period = 0.0001  # seconds
        self.timer = self.create_timer(self.timer_period, self.publish_callback)
        self.get_logger().info(f'====== start ======')
        
        self.subscription = self.create_subscription(
            Image,
            "diffbot/camera_sensor/image_raw",
            self.imageSubscribeCallBack,
            10,
        )
        self.subscription
        self.cv_bridge = CvBridge()

    def odom_sub_callback(self, data):
        orientation = data.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion(orientation)
        # self.get_logger().info(f'yaw : {self.yaw}')


    def imageSubscribeCallBack(self, data):
        try:
            current_frame = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")
            ImageSubscriber.image_ = current_frame
        except CvBridgeError as e:
            self.get_logger().info(e)

        # cv2.imshow("camera", current_frame)
        # cv2.waitKey(1)

        self.center_pixel = current_frame[400, 400]

    def publish_callback(self):
        self.twist_msg.linear.x = 0.5
        self.twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist_msg)

    def turn_callback(self, target):
        if self.turn_flag:
            self.turn_value = target
            self.get_logger().info(f'start Robot yaw is {self.yaw}')
            self.get_logger().info(f'Robot Turns to {self.turn_value}')
            self.turn_flag = False
         
        turn_offset = 0.7 * (self.turn_value - self.yaw)
        self.twist_msg.linear.x = 0.0
        self.twist_msg.angular.z = turn_offset
        self.cmd_vel_pub.publish(self.twist_msg)

        if abs(turn_offset) <= 0.087:
            self.stop_robot()
            self.timer.cancel()
            self.get_logger().warn(f'Robot current count {self.turn_count}')
            self.timer = self.create_timer(self.timer_period, self.publish_callback)
            self.go_during_second(4)
            self.get_logger().info(f'finish Robot yaw is {self.yaw}')
            self.done = True

    def stop_robot(self):
        self.twist_msg.linear.x = 0.0
        self.twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist_msg)

        time.sleep(1)

    def go_during_second(self, second):
        self.twist_msg.linear.x = 0.5
        self.twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist_msg)

        time.sleep(second)
        self.stop_robot()
    #  ===============================================================================================
    def check_right_turn(self):
        self.turn_count = (self.turn_count + 1) % 4
        return self.turn_count

    def check_left_turn(self):
        self.turn_count = (self.turn_count - 1 + 4) % 4
        return self.turn_count

    def check_back_turn(self):
        self.turn_count = (self.turn_count + 2) % 4
        return self.turn_count
    #  ===============================================================================================

    def create_callback(self, target, callback):
        self.turn_flag = True
        self.get_logger().info(f'====== call robot {target} ======')
        self.timer = self.create_timer(self.timer_period, callback)
        self.done = False


    def subscribeCallBack(self, msg):
        self.right_laser_sensor = msg.ranges[0]
        self.left_laser_sensor = msg.ranges[719]
        self.front_laser_sensor = msg.ranges[360]

        if not self.done:
            return

        if self.front_laser_sensor < 1.2:
            
            self.stop_robot()
            self.timer.cancel()
    
            print(sum(self.center_pixel), self.center_pixel[1])

            if sum(self.center_pixel) < 200 and self.center_pixel[1] > 70:
                self.get_logger().info(f'====== finish ======')
                self.destroy_node()
           

            if self.right_laser_sensor < 2.0 and self.left_laser_sensor > 2.0:
                value = DIRECTION_DICT[self.check_left_turn()]
                self.create_callback('turn left', lambda: self.turn_callback(value))

            elif self.right_laser_sensor < 2.0 and self.left_laser_sensor < 2.0:
                value = DIRECTION_DICT[self.check_back_turn()]
                self.create_callback('turn back', lambda: self.turn_callback(value))


            elif self.right_laser_sensor > 2.0:
                value = DIRECTION_DICT[self.check_right_turn()]
                self.create_callback('turn right', lambda: self.turn_callback(value))


def main(args=None):
    rclpy.init(args=args)
    try:
        executor = MultiThreadedExecutor()
        Miro_node = Miro()
        executor.add_node(Miro_node)
        try:
            executor.spin()
        except KeyboardInterrupt:
            Miro_node.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            executor.shutdown()
            Miro_node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
