import numpy as np
import cv2

class Utils:
    def __init__(self) :
        pass

    def calculate_velocity(self, old_position, new_position, fps=0.02, current_frame=None):
        x_old, y_old, w_old, h_old = old_position
        x_new, y_new, w_new, h_new = new_position
        old_center = (x_old + w_old/2, y_old + h_old/2)
        new_center =  (x_new + w_new/2, y_new + h_new/2)

        distance = np.sqrt((old_center[0] - new_center[0])**2 + (old_center[1] - new_center[1])**2)

        velocity = distance / fps

        real_world_height = 1.63

        if h_new != 0:
            pixel_to_meter_ratio = real_world_height / h_new
            velocity_meter = velocity * pixel_to_meter_ratio
            print(f"Velocity: {velocity_meter:.2f} m/s")
         
        if current_frame is not None:
            velocity_text = f"Velocity: {velocity_meter:.2f} m/s"
            
            position = (50, 50)  
            font = cv2.FONT_HERSHEY_SIMPLEX  
            font_scale = 1  
            color = (0, 0, 255) 
            thickness = 2  

            cv2.putText(current_frame, velocity_text, position, font, font_scale, color, thickness)

    def calculateDistance(self):
        pass