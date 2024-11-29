import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import serial
import threading
import time

class SerialNode(Node):
    def __init__(self):
        super().__init__("serial_node")

        # Serial port configuration
        self.serial_port = "/dev/ttyUSB0"  # Update this to your STM32's port
        self.baud_rate = 19200
        self.serial_connection = serial.Serial(self.serial_port, self.baud_rate, timeout=1
                                               , parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)

        # ROS2 subscription
        self.subscription = self.create_subscription(
            String,
            "stm32_topic",
            self.listener_callback,
            10
        )
        self.subscription_follow_specs_ = self.create_subscription(Twist, "follow_specs", self.handle_follow_specs, 10)
        
        # Thread for continuous reading from STM32
        self.read_thread = threading.Thread(target=self.read_from_stm32, daemon=True)
        self.read_thread.start()

    def listener_callback(self, msg):
        """
        Callback to handle messages from the ROS2 topic and send to STM32.
        """
        self.get_logger().info(f"Sending to STM32: {msg.data}")
        try:
            # Send the message to STM32 via serial
            self.serial_connection.write((msg.data + "\n").encode("utf-8"))
        except Exception as e:
            self.get_logger().error(f"Error sending to STM32: {e}")

    def handle_follow_specs(self, msg):
        """
        Callback to handle follow specs messages and send to STM32.
        """
        # self.get_logger().info(f"Sending follow specs to STM32: {msg}")
        try:
            # Send the message to STM32 via serial
            self.serial_connection.write((f"{msg.linear.x},{msg.angular.z}\n").encode("utf-8"))
            time.sleep(1)
        except Exception as e:
            self.get_logger().error(f"Error sending follow specs to STM32: {e}")

    def read_from_stm32(self):
        """
        Continuously read from STM32 and handle the data.
        """
        self.get_logger().info("Starting STM32 read loop...")
        while rclpy.ok():
            try:
                if self.serial_connection.in_waiting > 0:
                    data = self.serial_connection.readline()
                    self.get_logger().info(f"Received from STM32::::: {data}")
                    # Process the received data or publish it to another ROS topic if needed
            except Exception as e:
                self.get_logger().error(f"Error reading from STM32: {e}")

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
