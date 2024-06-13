#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.video_writer = None
        self.frame_width = 800
        self.frame_height = 800
        self.fps = 30
        self.video_file = 'output_video.avi'
        self.create_video_writer()
    def create_video_writer(self):
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.video_writer = cv2.VideoWriter(self.video_file, fourcc, self.fps, (self.frame_width, self.frame_height))
    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if self.video_writer is None:
                self.get_logger().info('Video writer not initialized.')
                return
            self.video_writer.write(cv_image)
            self.get_logger().info('Frame written to video.')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
    def destroy_node(self):
        if self.video_writer:
            self.video_writer.release()
        super().destroy_node()
def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        image_subscriber.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        image_subscriber.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()