import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import torchvision
import numpy as np
from torchvision.models.detection import fasterrcnn_resnet50_fpn, FasterRCNN_ResNet50_FPN_Weights, ssdlite320_mobilenet_v3_large, SSDLite320_MobileNet_V3_Large_Weights
from .data_object import COCO_INSTANCE_CATEGORY_NAMES
from .multi_tracking import get_multi_tracker, tracking


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.publishers_ = self.create_publisher(CompressedImage, 'detection_image', 10)
        self.subscribers_ = self.create_subscription(Image, "image_raw", self.listener_callback, 10)
        
        # Load the model with the correct weights
        weights = SSDLite320_MobileNet_V3_Large_Weights.DEFAULT
        self.model = ssdlite320_mobilenet_v3_large(weights=weights)  # Instantiate the model
        self.model.eval()  # Set the model to evaluation mode
        
        self.cv_bridge = CvBridge()

        self.coco_labels = {k: v for k, v in enumerate(COCO_INSTANCE_CATEGORY_NAMES)}
        # Init calculate
        self.first_ratio = 0
        self.first_focal = 0
        self.pixel_to_meter_ratio = 0
        self.focal_length = 0

        # Init boxes for tracking
        self.boxes = []
        self.multi_tracker = None
        self.result_tracking = None
        self.old_boxes = []
        self.get_logger().info("labels: %s" % self.coco_labels)
        self.get_logger().info('Object Detection Node has been started')

    def listener_callback(self, msg): 
        try: 
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8") 
            self.cv_image = cv_image 
 
            if self.multi_tracker: 
                self.tracking_objects() 
                return 
 
            if self.multi_tracker is None and len(self.boxes) == 0: 
                self.detect_img(cv_image) 
                self.init_tracker() 
                return 
             
 
        except CvBridgeError as e: 
            self.get_logger().error('Failed to convert image: %s' % str(e)) 
            return 
 
    def detect_img(self, cv_image): 
        self.get_logger().info("Detecting image................")
        transform = torchvision.transforms.Compose([torchvision.transforms.ToTensor()]) 
        image_tensor = transform(cv_image) 
        outputs = self.model([image_tensor])[0] 
        self.outputs = outputs 
 
        # return outputs, cv_image 
        for _, (box, score, label) in enumerate(zip(outputs['boxes'], outputs['scores'], outputs['labels'])): 
            if score >= 0.5: 
                x1, y1, x2, y2 = box.int().tolist() 
 
                if label.item() not in [0, 1]:  # Only allow certain labels 
                    continue 
                 
                new_box = (x1, y1, x2 - x1, y2 - y1) 
                 
                self.boxes.append(new_box) 
 
    def tracking_objects(self): 
        frame_update, tracked_boxes = tracking(self, self.multi_tracker, self.cv_image) 
 
        # If tracking fails (tracked_boxes is None), re-detect objects 
        if tracked_boxes is None: 
            self.get_logger().info('Tracking failed, triggering re-detection...') 
            self.boxes = []  # Reset boxes 
            self.multi_tracker = None
            return 
 
        # If frame_update is not False, publish the updated frame 
        if frame_update is not False: 
            # self.get_logger().info('Tracking..........') 
            # handled_image = self.cv_bridge.cv2_to_imgmsg(frame_update, "bgr8") 
            handled_image = self.cv_bridge.cv2_to_compressed_imgmsg(frame_update)

            self.publishers_.publish(handled_image) 
     
    def init_tracker(self): 
        if self.multi_tracker is None and len(self.boxes) > 0: 
            self.get_logger().info('Get multi-tracker..........') 
            self.multi_tracker = get_multi_tracker(self.boxes, self.cv_image) 
            # publish_image = self.cv_bridge.cv2_to_imgmsg(self.cv_image, "bgr8") 
            # self.publishers_.publish(publish_image)
                   
                    
def main(args=None):
    rclpy.init(args=args)
    object_detection_node = ObjectDetectionNode()
    rclpy.spin(object_detection_node)
    
    object_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
