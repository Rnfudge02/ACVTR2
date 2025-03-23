import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class VideoStreamNode(Node):
    def __init__(self):
        super().__init__('video_stream_node')
        self.video_path = "/path/to/your/video.mp4"  # Replace with your video path
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(self.video_path)
        self.timer_ = self.create_timer(1/30, self.publish_frame) # Publish at 30 FPS

    def publish_frame(self):
        ret, frame = self.cap.read()
        if ret:
            try:
                image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.publisher_.publish(image_msg)
            except Exception as e:
                self.get_logger().error("Error publishing image: %s" % str(e))
        else:
            self.get_logger().info("End of video reached")
            self.cap.release() # Release the video capture object
            self.destroy_node() # Stop the node when video ends
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = VideoStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()