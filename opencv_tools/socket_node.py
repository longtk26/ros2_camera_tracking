import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import base64
import asyncio
import websockets
import threading

class SocketNode(Node):
    def __init__(self):
        super().__init__('socket_node')
        self.subscribers_ = self.create_subscription(Image, "detection_image", self.listener_callback, 10)
        self.cv_bridge = CvBridge()
        
        self.websocket_clients = set()
        
        # Start the WebSocket server in a separate thread
        self.websocket_thread = threading.Thread(target=self.start_websocket_server)
        self.websocket_thread.start()
        self.get_logger().info('Socket Node has been started')

    def start_websocket_server(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.websocket_main())

    async def websocket_main(self):
        # Start the WebSocket server
        self.websocket_server = await websockets.serve(self.websocket_handler, "0.0.0.0", 8765)
        await self.websocket_server.wait_closed()

    async def websocket_handler(self, websocket):
        self.websocket_clients.add(websocket)
        try:
            async for _ in websocket:
                pass  # Keeping connection open
        finally:
            self.websocket_clients.remove(websocket)

    def listener_callback(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            _, buffer = cv2.imencode('.jpg', cv_image)
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')
            # self.get_logger().info(f'Image converted to text: {jpg_as_text[:50]}...')

            # Use asyncio.run_coroutine_threadsafe for cross-thread calls
            asyncio.run_coroutine_threadsafe(self.broadcast_to_clients(jpg_as_text), self.loop)

        except CvBridgeError as e:
            self.get_logger().error(f'Error converting image: {e}')

    async def broadcast_to_clients(self, message):
        if self.websocket_clients:
            await asyncio.gather(*[client.send(message) for client in self.websocket_clients])

    def destroy_node(self):
        # Close the WebSocket server
        self.loop.run_until_complete(self.websocket_server.wait_closed())
        self.get_logger().info('WebSocket server stopped')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    socket_node = SocketNode()
    rclpy.spin(socket_node)
    
    socket_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
