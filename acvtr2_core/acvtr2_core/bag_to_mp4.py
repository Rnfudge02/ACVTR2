#!/usr/bin/python3

#Copyright [2025] [Robert Fudge]
#SPDX-FileCopyrightText: © 2025 Robert Fudge <rnfudge@mun.ca>
#SPDX-License-Identifier: {Apache-2.0}

#Import libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

#Node for converting from ROS2 images to a .mp4 file using OpenCV
class ImageToVideoConverter(Node):
    #Class constructor
    def __init__(self):
        super().__init__('image_to_video_converter')
        
        #Declare node parameters
        self.declare_parameter('topic', '/camera/camera/color/image_raw')
        self.declare_parameter('output', 'output.mp4')
        self.declare_parameter('fps', 30)
        self.declare_parameter('compressed', False)

        #Get the required parameters
        topic_name = self.get_parameter('topic').value
        output_file = self.get_parameter('output').value
        self.fps = self.get_parameter('fps').value
        self.compressed = self.get_parameter('compressed').valuepassthrough

        if not topic_name:
            self.get_logger().error('Must specify input topic!')
            raise ValueError('Input topic not specified')

        # Initialize CV bridge
        self.bridge = CvBridge()
        self.video_writer = None
        self.frame_size = None

        # Create appropriate subscriber
        if self.compressed:
            self.subscription = self.create_subscription(
                CompressedImage,
                topic_name,
                self.compressed_callback,
                10)
        else:
            self.subscription = self.create_subscription(
                Image,
                topic_name,
                self.image_callback,
                10)

        self.get_logger().info(f'Listening to {topic_name}, saving to {output_file}')

    def image_callback(self, msg):
        self.get_logger().info(f'Received image: encoding = {msg.encoding}, width = {msg.width}, height = {msg.height}')
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
            self.get_logger().info(f'Converted image shape: {cv_image.shape}')
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {str(e)}')
            return

        self.process_frame(cv_image)

    def compressed_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is None:
                raise ValueError('Failed to decompress image')

        except Exception as e:
            self.get_logger().error(f'Compressed image error: {str(e)}')
            return

        self.process_frame(cv_image)

    def process_frame(self, frame):
        if self.video_writer is None:
            self.initialize_writer(frame)

        self.video_writer.write(frame)

    def initialize_writer(self, frame):
        height, width = frame.shape[:2]
        self.frame_size = (width, height)
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.video_writer = cv2.VideoWriter(
            self.get_parameter('output').value,
            fourcc,
            self.fps,
            self.frame_size
        )
        self.get_logger().info(f'Initialized video writer with {self.frame_size} @ {self.fps} FPS')

    def destroy_node(self):
        if self.video_writer is not None:
            self.video_writer.release()
            self.get_logger().info('Video writer released')
        super().destroy_node()

def main(args = None):
    rclpy.init(args = args)
    try:
        converter = ImageToVideoConverter()
        rclpy.spin(converter)

    except Exception as e:
        converter.get_logger().error(f'Error: {str(e)}')

    finally:
        converter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()