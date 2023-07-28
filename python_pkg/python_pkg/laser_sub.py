# !/usr/bin/env/ python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import os

class LaserSubscriber(Node):

    def __init__(self):
        super().__init__('laser_sub_node')
        ROBOT_MODEL = os.environ['ROBOT_MODEL']
        queue_size = 10
        self.subscriber = self.create_subscription(
            LaserScan, ROBOT_MODEL+'/scan', self.subscribeCallBack, queue_size
        )
        self.subscriber  # prevent unused variable warning

    def subscribeCallBack(self, msg):
        self.get_logger().info(f'Raw Laser Data : {msg.ranges}')

def main(args=None):
    rclpy.init(args=args)

    laser_subscriber = LaserSubscriber()

    rclpy.spin(laser_subscriber)

    laser_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()