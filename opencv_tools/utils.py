import numpy as np
import cv2

class Utils:
    def __init__(self) :
        pass

    def calculate_velocity(self, old_position, new_position, fps=0.02, current_frame=None, node_detect=None):
        x_old, y_old, w_old, h_old = old_position
        x_new, y_new, w_new, h_new = new_position
        old_center = (x_old + w_old/2, y_old + h_old/2)
        new_center =  (x_new + w_new/2, y_new + h_new/2)

        distance = np.sqrt((old_center[0] - new_center[0])**2 + (old_center[1] - new_center[1])**2)

        velocity = distance / fps

        real_world_height = 1.63
        velocity_meter = 0

        if h_new != 0:
            if node_detect.first_ratio == 0:
                node_detect.pixel_to_meter_ratio = 0.0002645833
                node_detect.get_logger().info(f"RATIO FIRST................{node_detect.pixel_to_meter_ratio}")
                node_detect.first_ratio = 1

            velocity_meter = velocity * node_detect.pixel_to_meter_ratio
            print(f"Velocity: {velocity_meter:.2f} m/s")
         
        if current_frame is not None:
            velocity_text = f"Velocity: {velocity_meter:.2f} m/s"
            
            position = (50, 50)  
            font = cv2.FONT_HERSHEY_SIMPLEX  
            font_scale = 1  
            color = (0, 0, 255) 
            thickness = 2  

            cv2.putText(current_frame, velocity_text, position, font, font_scale, color, thickness)
        return velocity_meter
    
    def calculate_distance(self, position, current_frame, node_detect):
        x, y, w, h = position
        real_high  = 1.63
        measured_distance = 0.9 #meter 
        if node_detect.first_focal == 0:
            node_detect.focal_length = self.focal_length(high_in_image=h, real_high=real_high, measured_distance=measured_distance)
            node_detect.get_logger().info(f"FOCAL LENGTH FIRST................{node_detect.focal_length}")
            node_detect.first_focal = 1
        
        distance = self.distance_finder(focal_length=node_detect.focal_length, real_high_person=real_high, high_person_in_frame=h)

        if current_frame is not None:
            distance_text = f"Distance: {distance:.2f} m"
            
            position = (50, 100)  # Position on the frame to display the text
            font = cv2.FONT_HERSHEY_SIMPLEX  # Font type
            font_scale = 1  # Font scale (size)
            color = (255, 0, 0)  # Green color for the text
            thickness = 2  # Thickness of the text

        cv2.putText(current_frame, distance_text, position, font, font_scale, color, thickness)

        return distance

    def focal_length(self, high_in_image, real_high, measured_distance):
        focal_length = (high_in_image * measured_distance) / real_high
        return focal_length

    def distance_finder(self, focal_length, real_high_person, high_person_in_frame):
        distance = (real_high_person * focal_length)/high_person_in_frame

        return distance
    
    def calculate_angle(self, position, current_frame, node_detect):
        # Get the width and height of the current frame
        current_height, current_width, _ = current_frame.shape
        # node_detect.get_logger().info(f"Width of picture: {current_width}")
        
        # Camera parameters
        horizontal_fov = 80  # in degrees
        
        # Calculate angle per pixel for the current width
        angle_per_pixel = horizontal_fov / current_width

        # Bounding box center
        x, y, w, h = position
        person_center_x = x + w / 2

        # Calculate the pixel offset from the center of the image
        offset_x = person_center_x - (current_width / 2)

        # Calculate the angle
        angle = offset_x * angle_per_pixel

        # Display angle on the frame if provided
        if current_frame is not None:
            angle_text = f"Angle: {angle:.2f} degree"
            position_text = (50, 150)
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 1
            color = (0, 255, 0)
            thickness = 2

            cv2.putText(current_frame, angle_text, position_text, font, font_scale, color, thickness)

        return angle
