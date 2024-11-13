from sensor_msgs.msg import Image
from rclpy.node import Node
import rclpy
import cv2
from cv_bridge import CvBridge, CvBridgeError

class ImagePublishNode(Node):
    def __init__(self, name):
        super().__init__(name)

        # Create a publisher to publish images to the 'image_raw' topic.
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.get_logger().info('Image publishing node started...')

        # Initialize the camera capture (0 indicates the default camera)
        # self.cap = cv2.VideoCapture(4)
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error("Failed to open the camera.")
            return

        self.timer_ = self.create_timer(1 / 50, self.timer_callback_)  # Publish every 0.1 seconds (10 FPS)
        self.cv_bridge = CvBridge()

    def timer_callback_(self):
        # Capture frame from the camera
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().error("Failed to capture image from camera.")
            return

        try:
            # Convert the image to an ROS image message.
            image_message = self.cv_bridge.cv2_to_imgmsg(frame, "bgr8")
            # Publish the image message.
            self.publisher_.publish(image_message)
            # self.get_logger().info('Published image from camera')

        except CvBridgeError as e:
            self.get_logger().error(f'Failed to convert image: {str(e)}')
            return

    def destroy_node(self):
        # Release the camera when the node is destroyed
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    image_publish_node = ImagePublishNode('image_publish_node')

    rclpy.spin(image_publish_node)
    image_publish_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



