import rclpy
from rclpy.node import Node
import serial
import threading
from flask import Flask, request, jsonify
from flask_cors import CORS 

class SerialServiceNode(Node):
    def __init__(self):
        super().__init__('http_node')
        self.serial_lock = threading.Lock()
        # Initialize the serial connection
        self.serial_port = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=1)
        
        # Initialize Flask app
        self.app = Flask(__name__)
        CORS(self.app)
        self.app.add_url_rule('/send_data', 'send_data', self.handle_http_request, methods=['POST'])
        
        # Start the Flask server in a separate thread
        self.flask_thread = threading.Thread(target=self.start_flask_server)
        self.flask_thread.start()
        
        self.get_logger().info('Serial Service Node with Flask HTTP server is running')

    def start_flask_server(self):
        # Run Flask server on localhost and port 5000
        self.app.run(host='0.0.0.0', port=5000, threaded=True)

    def handle_http_request(self):
        try:
            # Parse JSON from HTTP POST request
            data = request.get_json()
            serial_data = data.get('serial_data', None)
            
            if serial_data is not None:
                # Send data through serial port
                self.send_serial_data(serial_data)
                
                # Return success response
                response = {'status': 'success', 'message': 'Data sent via serial'}
                return jsonify(response), 200
            else:
                response = {'status': 'error', 'message': 'No serial_data provided'}
                return jsonify(response), 400
        except Exception as e:
            self.get_logger().error(f'Failed to handle HTTP request: {e}')
            response = {'status': 'error', 'message': 'Failed to handle request'}
            return jsonify(response), 500

    def send_serial_data(self, data):
        try:
            with self.serial_lock:
                self.serial_port.write(data.encode('utf-8'))
                self.serial_port.flush()  # Ensure data is sent
                self.get_logger().info(f'Sent data over serial: {data}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial communication error: {e}')

    def destroy_node(self):
        # Close the serial port and stop the Flask server on shutdown
        if self.serial_port.is_open:
            self.serial_port.close()
        self.get_logger().info('Serial port closed')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialServiceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
