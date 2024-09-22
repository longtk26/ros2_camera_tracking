import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import torchvision
import numpy as np
from torchvision.models.detection import ssdlite320_mobilenet_v3_large, SSDLite320_MobileNet_V3_Large_Weights, ssd300_vgg16
from .data_object import COCO_INSTANCE_CATEGORY_NAMES
from .multi_tracking import get_multi_tracker, tracking, add_tracker


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.publishers_ = self.create_publisher(Image, 'detection_image', 10)
        self.subscribers_ = self.create_subscription(Image, "image_raw", self.listener_callback, 10)
        
        # Load the model with the correct weights
        weights = SSDLite320_MobileNet_V3_Large_Weights.DEFAULT
        self.model = ssdlite320_mobilenet_v3_large(weights=weights)  # Instantiate the model
        self.model.eval()  # Set the model to evaluation mode
        
        self.cv_bridge = CvBridge()

        self.coco_labels = {k: v for k, v in enumerate(COCO_INSTANCE_CATEGORY_NAMES)}

        # Init boxes for tracking
        self.boxes = []
        self.old_num_boxes = 0
        self.multi_tracker = None
        self.result_tracking = None
        self.get_logger().info("labels: %s" % self.coco_labels)
        self.get_logger().info('Object Detection Node has been started')

    def listener_callback(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            transform = torchvision.transforms.Compose([torchvision.transforms.ToTensor()])
            image_tensor = transform(cv_image)
            outputs = self.model([image_tensor])[0]

            if len(self.boxes) == 0:
                self.get_logger().info("Init boxes........")
                self.filter_bboxes_tracking(outputs, cv_image)

            # Tracking
            if self.multi_tracker is None and len(self.boxes) > 0:
                self.get_logger().info('Get multi-tracker..........')
                self.multi_tracker = get_multi_tracker(self.boxes, cv_image)
                publish_image = self.cv_bridge.cv2_to_imgmsg(cv_image, "bgr8")
                self.publishers_.publish(publish_image)
            elif len(self.boxes) > 0:
                frame_update, tracked_boxes = tracking(self.multi_tracker, cv_image)

                # If tracking fails (tracked_boxes is None), re-detect objects
                if tracked_boxes is None:
                    self.get_logger().info('Tracking failed, triggering re-detection...')
                    self.boxes = []  # Reset boxes
                    self.filter_bboxes_tracking(outputs, cv_image)
                    self.multi_tracker = None  # Reset tracker
                else:
                    # Check if the size of the tracking box is larger than the detection box
                    for i, (tracked_box, detection_box) in enumerate(zip(tracked_boxes, outputs['boxes'])):
                        tracked_width = tracked_box[2]  # Width of the tracked box
                        tracked_height = tracked_box[3]  # Height of the tracked box

                        detection_x1, detection_y1, detection_x2, detection_y2 = detection_box.int().tolist()
                        detection_width = detection_x2 - detection_x1
                        detection_height = detection_y2 - detection_y1

                        # If tracking box is significantly larger than detection box, re-detect
                        if tracked_width > detection_width * 1.5 or tracked_height > detection_height * 1.5:
                            self.get_logger().info('Tracking bbox too large, triggering re-detection...')
                            self.boxes = []  # Reset boxes
                            self.filter_bboxes_tracking(outputs, cv_image)
                            self.multi_tracker = None  # Reset tracker
                            break

                # If frame_update is not False, publish the updated frame
                if frame_update is not False:
                    handled_image = self.cv_bridge.cv2_to_imgmsg(frame_update, "bgr8")
                    self.publishers_.publish(handled_image)

        except CvBridgeError as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return

        
    
    def filter_bboxes_tracking(self, outputs, cv_image):
        for _, (box, score, label) in enumerate(zip(outputs['boxes'], outputs['scores'], outputs['labels'])):
            if score >= 0.5:
                x1, y1, x2, y2 = box.int().tolist()

                # if label.item() not in [0, 1, 2, 3]:  # Only allow certain labels
                #     continue
                
                new_box = (x1, y1, x2 - x1, y2 - y1)
                
                self.boxes.append(new_box)
              
                   
                    
def main(args=None):
    rclpy.init(args=args)
    object_detection_node = ObjectDetectionNode()
    rclpy.spin(object_detection_node)
    
    object_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


def listener_callback(self, msg):
    try:
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")

        # Detect new objects and filter them for tracking
        self.filter_bboxes_tracking(cv_image)

        # Tracking init
        if self.multi_tracker is None and len(self.boxes) > 0:
            self.get_logger().info('Initializing multi-tracker..........')
            self.multi_tracker = get_multi_tracker(self.boxes, cv_image)
            handled_image = self.cv_bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.publishers_.publish(handled_image)
        elif self.multi_tracker is not None:
            self.result_tracking = tracking(self.multi_tracker, cv_image)

        # Object tracking failure
        if self.result_tracking is False:
            self.get_logger().info('Tracking failed - re-detect and initialize tracking again...')
            self.boxes = []
            self.result_tracking = None
            self.multi_tracker = None

            # Re-detect and reinitialize tracking
            self.filter_bboxes_tracking(cv_image)
            if len(self.boxes) > 0:
                self.multi_tracker = get_multi_tracker(self.boxes, cv_image)
                handled_image = self.cv_bridge.cv2_to_imgmsg(cv_image, "bgr8")
                self.publishers_.publish(handled_image)

        # Update frame to rviz
        if self.result_tracking is not None:
            if isinstance(self.result_tracking, np.ndarray):
                if len(self.result_tracking.shape) == 2 or len(self.result_tracking.shape) == 3:
                    handled_image = self.cv_bridge.cv2_to_imgmsg(self.result_tracking, "bgr8")
                    self.publishers_.publish(handled_image)
                else:
                    self.get_logger().error(f'Invalid image shape: {self.result_tracking.shape}')
            else:
                self.get_logger().error('result_tracking is not a NumPy array')
        else:
            self.get_logger().error('result_tracking is None')

    except CvBridgeError as e:
        self.get_logger().error('Failed to convert image: %s' % str(e))
        return


