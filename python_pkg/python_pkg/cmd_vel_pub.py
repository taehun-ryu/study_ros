# !/usr/bin/env/ python3
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
import os

class CmdVelPublisher(Node):

    def __init__(self):
        super().__init__('cmd_vel_pub_node')
        ROBOT_MODEL = os.environ['ROBOT_MODEL']
        self.publisher = self.create_publisher(
            Twist, ROBOT_MODEL+'/cmd_vel', 10
        )  # queue size
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publishCallback)
        self.get_logger().info(
            'DriveForward node Started, move forward during 5 seconds \n'
        )

    def publishCallback(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.5
        twist_msg.angular.z = 1.0
        self.publisher.publish(twist_msg)

    def stopRobot(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.publisher.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)

    cmd_vel_publisher = CmdVelPublisher()
    start_time = cmd_vel_publisher.get_clock().now().to_msg().sec
    clock_now = start_time
    time_delta = 0

    while (clock_now - start_time) < 5:
        rclpy.spin_once(cmd_vel_publisher)
        clock_now = cmd_vel_publisher.get_clock().now().to_msg().sec

        time_delta = clock_now - start_time
        cmd_vel_publisher.get_logger().info(f'{time_delta} seconds passed')

    cmd_vel_publisher.stopRobot()

    cmd_vel_publisher.get_logger().info('\n==== Stop Publishing ====')
    cmd_vel_publisher.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()