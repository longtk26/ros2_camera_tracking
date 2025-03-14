import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import serial
import ast
import math
import sys
from .utils import Utils

utils = Utils()

class StandleyNode(Node):
    def __init__(self):
        super().__init__("standley_node")
        # Initialize variables
        self.EARTH_RADIUS = 6371000
        self.START_STANDLEY_ALGORITHM = False
        self.ref_lat = 10.882168240297924   
        self.ref_lon = 106.80561693651214
        self.angle_imu = 0.0
        self.current = 0.0
        # ROS2 subscription
        self.subscription_ui = self.create_subscription(
            String,
            "ui_control",
            self.ui_callback,
            10
        )
        self.subscription_serial = self.create_subscription(
            String,
            "gps_topic",
            self.gps_callback,
            10
        )
        self.subscription_stm32 = self.create_subscription(
            String,
            "stm32_topic",
            self.stm32_callback,
            10
        )

        # ROS2 publisher
        self.publisher_standley_algorithm = self.create_publisher(
            String,
            "standley_output",
            10
        )

        self.get_logger().info("Standley node has been started.")
        

    def ui_callback(self, msg):
        node_received, data_received = self.__get_node_and_data_from_msg(msg.data)
        self.handle_ui_callback(node_received, data_received)

    def handle_ui_callback(self, node_received, data_received):
        if node_received != "[standley_node]":
            return
        self.get_logger().info("===[Standley Node] received data from UI===")
        self.list_gps_data_on_gui = utils.convert_gps_data(data=data_received, context=self)

        # Convert GPS data to x, y coordinates
        self.x_y_coordinates = []
        for gps_data in self.list_gps_data_on_gui:
            x, y = self.__convert_lat_lon_to_xy(lat=gps_data[0], lon=gps_data[1])
            self.x_y_coordinates.append([x, y])
        self.get_logger().info(f"X, Y coordinates: {self.x_y_coordinates}")
        self.START_STANDLEY_ALGORITHM = True
        
    def gps_callback(self, msg):
        """
        Callback function for GPS data.
        """
        self.status_gps = msg.data.split(":")[0]
        self.lat_current = msg.data.split(":")[1]
        self.lon_current = msg.data.split(":")[2]
        self.handle_gps_callback()

    def handle_gps_callback(self):
        """
        Handle GPS data.
        """
        # Check if GPS is initialized
        if self.status_gps != "running":
            self.ref_lat = float(self.lat_current)
            self.ref_lon = float(self.lon_current)
            self.get_logger().info(f"Initalized GPS ref: {self.ref_lat}, {self.ref_lon}")
            self.current = 0
            self.__publish_msg(type_msg="stm32", data="E")  # Start the robot
            return

        # Convert current GPS data to x, y coordinates
        self.x_current, self.y_current = self.__convert_lat_lon_to_xy(lat=float(self.lat_current), lon=float(self.lon_current))
        try:
            if self.START_STANDLEY_ALGORITHM:
                delta, distance_to_goal, min_distance, heading_ref, theta_d = self.__run_standley_algorithm()
                angle_imu_rad = math.radians(float(self.angle_imu))
                self.get_logger().info(f"Delta: {delta}, Distance to goal: {distance_to_goal}, Min distance: {min_distance}, IMU: {angle_imu_rad}, X_Curr: {self.x_current}, Y_Curr: {self.y_current}")
                self.__publish_msg(type_msg="ui", data=f"{delta}:{distance_to_goal}:{min_distance}:{angle_imu_rad}:{heading_ref}:{theta_d}")
        except Exception as e:
            self.get_logger().error(f"Error in handle gps callback: {e}")
    def stm32_callback(self, msg):
        """
        Callback function for STM32 data.
        """
        self.angle_imu, is_valid = self.__check_msg_stm32_is_valid_and_return_data(msg.data)
        if is_valid:
            # self.get_logger().info(f"Angle IMU:::::: {self.angle_imu}")
            pass
        
    def __run_standley_algorithm(self):
        """
        Run Stanley algorithm to calculate the steering angle.
        """
        try:
            if not self.x_y_coordinates or not hasattr(self, 'x_current') or not hasattr(self, 'y_current'):
                return

            # Step 1: Check if the robot has reached the final goal
            goal_x, goal_y = self.x_y_coordinates[-1]  # Last point on the trajectory
            distance_to_goal = math.sqrt((goal_x - self.x_current) ** 2 + (goal_y - self.y_current) ** 2)
            if distance_to_goal < 0.05:
                self.__publish_msg(type_msg="stm32", data="E")  # Stop the robot           
                self.get_logger().info("Goal reached. Stopping robot.")
                return

            # Step 2: Find the reference point (closest point on the path)
            # min_distance = float('inf')
            # closest_point = None
            # for i in range(len(self.x_y_coordinates) - 10):
            #     for j in range(i, i + 10):  # Search next 10 points
            #         x_ref, y_ref = self.x_y_coordinates[j]
            #         distance = math.sqrt((x_ref - self.x_current) ** 2 + (y_ref - self.y_current) ** 2)
            #         if abs(distance) < abs(min_distance):
            #             min_distance = distance
            #             closest_point = (x_ref, y_ref, j)
            
            # if closest_point is None:
            #     return
            # x_ref, y_ref, j = closest_point
            self.current, min_distance = self.find_closest_in_window(self.x_current, self.y_current, self.x_y_coordinates, self.current, 10)
            j = self.current
            self.get_logger().info(f"Closest point [{j}]: {self.x_y_coordinates[j]}")
            
            # Step 3: Compute crosstrack error e(t)
            e_t = min_distance
            
            # Step 4: Compute heading error theta_e
            heading_ref = abs(math.atan2(self.x_y_coordinates[j+1][1] - self.x_y_coordinates[j][1]
                                     , self.x_y_coordinates[j+1][0] - self.x_y_coordinates[j][0]))
            heading_robot = math.radians(float(self.angle_imu))
            theta_e = heading_ref - heading_robot
            
            
            # Step 5: Compute control angle delta
            k = 0.5  # Gain parameter for crosstrack error
            ksoft = 0.01  # Small positive constant to avoid instability at low speed
            v = 0.5  # Assume velocity is 0.5 m/s (adjust if real velocity is available)
            theta_d = math.atan2(k * e_t, ksoft + v)
            
            # Compute final steering angle
            # theta_position = math.atan2(self.y_current, self.x_current)
            if heading_robot < heading_ref:
                delta = theta_e - theta_d
            else:
                delta = theta_e + theta_d

            # Publish result
            self.__publish_msg(type_msg="stm32", data=delta)
            self.get_logger().info(f"Published steering angle: {delta} radians")

            return delta, distance_to_goal, min_distance, heading_ref, theta_d
        except Exception as e:
            self.get_logger().error(f"Error in standley algorithm: {e}")
   
    def find_closest_in_window(self, x_current, y_current, x_y_coordinates, cur_index, window_size):
        if not x_y_coordinates:
            return None  # Trả về None nếu danh sách trống

        num_reference_points = len(x_y_coordinates)

        # Xác định phạm vi cửa sổ tìm kiếm
        start = max(0, cur_index - window_size)
        end = min(num_reference_points - 1, cur_index + window_size)

        min_distance = float('inf')
        best_index = start

        for i in range(start, end + 1):
            x_ref, y_ref = x_y_coordinates[i]
            distance = math.sqrt((x_ref - x_current) ** 2 + (y_ref - y_current) ** 2)

            if distance < min_distance:
                min_distance = distance
                best_index = i

        return best_index, min_distance

    def __convert_lat_lon_to_xy(self, lat, lon):
        """
        Convert latitude and longitude to x and y coordinates.
        """
        try:
            if lat == self.ref_lat and lon == self.ref_lon:
                return 0.0, 0.0

            lat_rad = math.radians(lat)
            lon_rad = math.radians(lon)
            ref_lat_rad = math.radians(self.ref_lat)
            ref_lon_rad = math.radians(self.ref_lon)

            sin_lat = math.sin(lat_rad)
            cos_lat = math.cos(lat_rad)
            cos_d_lon = math.cos(lon_rad - ref_lon_rad)

            ref_sin_lat = math.sin(ref_lat_rad)
            ref_cos_lat = math.cos(ref_lat_rad)

            c = math.acos(ref_sin_lat * sin_lat + ref_cos_lat * cos_lat * cos_d_lon)
            k = 1.0 if abs(c) < sys.float_info.epsilon else c / math.sin(c)

            x = k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) * self.EARTH_RADIUS
            y = k * cos_lat * math.sin(lon_rad - ref_lon_rad) * self.EARTH_RADIUS

            return x, y
        except Exception as e:
            self.get_logger().error(f"Error convert lat lon to xy: {e}")
            return 0.0, 0.0

    def __get_node_and_data_from_msg(self, msg):
        """
        Get node and data from the message.
        """
        node_received = msg.split(" ")[0]
        data_received = msg.split(" ")[1]
        return node_received, data_received

    def __check_msg_stm32_is_valid_and_return_data(self, msg):
        """
        Get node and data from the message.
        stm32_msg: s:4:<angle>:e
        Description:
            s: start
            4: Node received is Standley node
            <angle>: angle value
            e: end
        """
        try:
            node_received = msg.split(":")[1]
            if node_received != "4":
                return self.angle_imu, False
            
            # self.get_logger().info(f"STM32 msg received in standley: {msg}")
            data_received = msg.split(":")[2]
            return data_received, True
        except Exception as e:
            self.get_logger().error(f"Error check msg stm32: {e}")
            return False
    

    def __publish_msg(self, type_msg="stm32", data=""):
        """
        Publish message to the topic.
        """
        store = {
            "stm32": f"s:2:{data}:e",
            "ui": f"s:5:{data}:e"   # Node 4 is Standley node on UI: s:4:delta:distance_to_goal:min_distance:e
        }
        msg = String()
        msg.data = store[type_msg]
        self.publisher_standley_algorithm.publish(msg)
        # self.get_logger().info(f"Published message from standley::::::: {msg.data}")

    def destroy_node(self):
        """
        Cleanup resources on shutdown.
        """
        self.get_logger().info("Shutting down node...")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    standley_node = StandleyNode()

    try:
        rclpy.spin(standley_node)
    except KeyboardInterrupt:
        standley_node.get_logger().info("Keyboard interrupt detected.")
    finally:
        standley_node.destroy_node()
        rclpy.shutdown()
