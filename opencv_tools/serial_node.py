import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import serial
import ast

class SerialNode(Node):
    def __init__(self):
        super().__init__("serial_node")
        # Signal to start following specs
        self.SIGNAL_FOLLOW_SPECS = False
        self.SIGNAL_GPS = False
        self.SIGNAL_INIT_GPS = False

        # Serial port configuration
        self.serial_port = "/dev/ttyUSB0"  # Update this to your STM32's port
        self.baud_rate = 19200
        self.serial_connection = serial.Serial(self.serial_port, self.baud_rate, timeout=0.001
                                               , parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)

        # Serial gps configuration
        self.serial_gps_port = "/dev/ttyACM0"  # Update this to your GPS's port
        self.baud_rate_gps = 38400
        self.serial_gps_conn = serial.Serial(
            self.serial_gps_port,
            self.baud_rate_gps,
            timeout=1,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        # self.serial_gps_conn = None

        # Specs for STM32
        self.specs = {
            "speed": 0,
            "angle": 0
        }
        self.gps_data = ""

        # ROS2 subscription
        self.subscription = self.create_subscription(
            String,
            "ui_control",
            self.listener_callback,
            10
        )
        self.subscription_follow_specs_ = self.create_subscription(Twist, "follow_specs", self.handle_follow_specs, 10)
        self.publishers_ = self.create_publisher(String, "stm32_topic", 10)


        # ROS2 timers
        self.timer_ = self.create_timer(0.01, self.send_follow_specs)
        self.timer_receive_STM32_ = self.create_timer(0.01, self.read_from_stm32)
        self.timer_read_gps_ = self.create_timer(0.01, self.read_gps_data)
        self.get_logger().info("Serial node has been started.")
        

    def listener_callback(self, msg):
        """
        Callback to handle messages from the ROS2 topic and send to STM32.
        """
        self.get_logger().info(f"===SIGNAL FROM UI CONTROL=== {msg.data}")
        node_received = msg.data.split(" ")[0]
        self.get_logger().info(f"===NODE RECEIVED=== {node_received}")
        try:
            if msg.data == "start_follow":
                self.SIGNAL_FOLLOW_SPECS = True
                self.SIGNAL_GPS = False
                self.SIGNAL_INIT_GPS = False
            elif msg.data == "stop_follow":
                self.SIGNAL_FOLLOW_SPECS = False
                self.SIGNAL_GPS = False
                self.SIGNAL_INIT_GPS = False
                self.specs["speed"] = 0
                self.specs["angle"] = 0
            elif node_received == "[serial]":
                self.SIGNAL_FOLLOW_SPECS = False
                self.gps_data = msg.data.split(" ")[1]
                if not(self.gps_data == "find-me"):
                    self.SIGNAL_GPS = True
                    self.SIGNAL_INIT_GPS = False
                    self.send_gps_data()
                else:
                    self.SIGNAL_GPS = False
                    self.SIGNAL_INIT_GPS = True
                    self.send_init_gps_lat_long()
        except Exception as e:
            self.get_logger().error(f"Error sending to STM32: {e}")

    def handle_follow_specs(self, msg):
        """
        Callback to handle follow specs messages and send to STM32.
        """
        # self.get_logger().info(f"Sending follow specs to STM32: {msg}")
        try:
            # Send the message to STM32 via serial
            self.specs["speed"] = f"{msg.linear.x:.3f}"
            self.specs["angle"] = f"{msg.angular.z:.3f}"
        except Exception as e:
            self.get_logger().error(f"Error sending follow specs to STM32: {e}")

    def read_from_stm32(self):
        """
        Continuously read from STM32 and handle the data.
        """
        try:
            if self.serial_connection.in_waiting > 0:
                data = self.serial_connection.readline().decode("utf-8").strip()
                self.get_logger().info(f"Received from STM32::::: {data}")
                # Publish the received data to another ROS topic
                msg = String()
                msg.data = data
                self.publishers_.publish(msg)
                # Process the received data or publish it to another ROS topic if needed
        except Exception as e:
            self.get_logger().error(f"Error reading from STM32: {e}")
    
    def read_gps_data(self):
        """
        Read GPS data from the GPS module.
        Data format from GPS module: $GNRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
        """
        try:
            if self.serial_gps_conn.in_waiting > 0 and self.SIGNAL_GPS:
                data = self.serial_gps_conn.readline().decode("utf-8").strip()
                self.get_logger().info(f"Received GPS data::::: {data}")
                # format the data to send to GUI
                formatted_data = data.split(",")
                frame_ = f"s:2:2:{formatted_data[3]}:{formatted_data[5]}:e"
                self.get_logger().info(f"Sending GPS data to GUI:::: {frame_}")

                msg = String()
                msg.data = frame_
                self.publishers_.publish(msg)
                # Process the received data or publish it to another ROS topic if needed
        except Exception as e:
            self.get_logger().error(f"Error reading GPS data: {e}")

    def send_follow_specs(self):
        """
        Send follow specs to STM32 in follow mode.
        """
        try:
            if self.SIGNAL_FOLLOW_SPECS:
                frame_ = f"s:1:2:{self.specs['angle']}:{self.specs['speed']}:e\n"
                self.get_logger().info(f"Sending specs to stm32 {frame_}")
                self.serial_connection.write((frame_).encode("utf-8"))
        except Exception as e:
            self.get_logger().error(f"Error sending follow specs to STM32: {e}")

    def send_gps_data(self):
        """
        Send GPS data to STM32.
        """
        try:
            # Send GPS data to STM32
            if self.SIGNAL_GPS:
                frame_ = self._convert_gps_data(self.gps_data)
                for index, data in enumerate(frame_):
                    frame_ = f"s:2:1:{data[0]}:{data[1]}:e"
                    self.get_logger().info(f"Sending GPS data to STM32:::: {frame_}--{index+1}")
                    self.serial_connection.write((frame_).encode("utf-8"))
        except Exception as e:
            self.get_logger().error(f"Error sending GPS data to STM32: {e}")

    def _convert_gps_data(self, data):
        """
        Convert GPS data from string format to a list of tuples.
        data: "len-[[lat, long], [lat, long], ...]"
        """
        try:
            data = data.split("-", 1)[1] 
            formatted_data = ast.literal_eval(data)  
            
            result = [[formatted_data[i], formatted_data[i+1]] for i in range(0, len(formatted_data), 2)]
            self.get_logger().info(f"Len of GPS data:::: {len(result)}")
            return result
        except Exception as e:
            self.get_logger().error(f"Error converting GPS data::: {e}")
            return None

    def send_init_gps_lat_long(self):
        """
        Send initial GPS data to GUI.
        """
        try:
            if self.serial_gps_conn.in_waiting > 0 and self.SIGNAL_INIT_GPS:
                data = self.serial_gps_conn.readline().decode("utf-8").strip()
                self.get_logger().info(f"Received GPS data::::: {data}")
                # format the data to send to GUI
                formatted_data = data.split(",")
                if not(formatted_data[0] == "$GNRMC"):
                    self.get_logger().warn(f"Invalid GPS data::: {data}")
                    return
                frame_ = f"s:3:2:{formatted_data[3]}:{formatted_data[5]}:e"
                self.get_logger().info(f"Sending GPS data to GUI:::: {frame_}")

                msg = String()
                msg.data = frame_
                self.publishers_.publish(msg)
            # if self.SIGNAL_INIT_GPS:
            #     frame_ = "s:3:2:1058.61276:10640.44881:e"
            #     self.get_logger().info(f"Sending init GPS data to GUI: {frame_}")

            #     msg = String()
            #     msg.data = frame_
            #     self.publishers_.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error sending init GPS data to GUI: {e}")

    def destroy_node(self):
        """
        Cleanup resources on shutdown.
        """
        self.get_logger().info("Shutting down node...")
        self.serial_connection.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    serial_comm_node = SerialNode()

    try:
        rclpy.spin(serial_comm_node)
    except KeyboardInterrupt:
        serial_comm_node.get_logger().info("Keyboard interrupt detected.")
    finally:
        serial_comm_node.destroy_node()
        rclpy.shutdown()
