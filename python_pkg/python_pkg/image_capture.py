# !/usr/bin/env/ python3
import os
import cv2
import rclpy

from rclpy.node import Node
# 둘 사이의 변환을 담당하는 cv_bridge입니다.
from cv_bridge import (
    CvBridge,
    CvBridgeError,
)  # Package to convert between ROS and OpenCV Images

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from study_msgs.srv import ImageCapture

# bool button_press
# ---
# bool success

class ImageSubscriber(Node):
    image_ = None

    def __init__(self):
        super().__init__("image_capture")
        ROBOT_MODEL = os.environ['ROBOT_MODEL']
        self.subscription = self.create_subscription(
            Image,
            ROBOT_MODEL+"/camera_sensor/image_raw",
            self.listener_callback,
            10,
        )
        self.subscription
        self.srv = self.create_service(
            ImageCapture, 'capture_image', self.captureImageCallBack
        )
        # ROS2 <=> OpenCV를 해주는 cv_bridge
        self.cv_bridge = CvBridge()
        self.get_logger().info('==== Image Capture Server Started, Waiting for Request ====')

    def captureImageCallBack(self, request, response):
        src= ImageSubscriber.image_
        if request.edge:
            self.get_logger().info('A image is saved naming image.jpg using Canny Edge Detection')
            gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
            canny = cv2.Canny(gray,50,200)
            cv2.imwrite("/home/ryu/study_ws/src/py_pkg/image.jpg",canny)
            response.success = True
        else:
            self.get_logger().info('A image is saved naming image.jpg using Original Image')
            cv2.imwrite("/home/ryu/study_ws/src/py_pkg/image.jpg",src)
            response.success = True
        self.get_logger().info('Servie Process Done...')

        return response

    def listener_callback(self, data):
        try:
            current_frame = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")
            ImageSubscriber.image_ = current_frame
        except CvBridgeError as e:
            self.get_logger().info(e)
        cv2.imshow("camera", current_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
