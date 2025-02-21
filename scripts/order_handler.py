#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class OrderHandler(Node):
    def __init__(self):
        super().__init__('order_handler')

        # Publisher to send navigation requests
        self.nav_pub = self.create_publisher(String, '/navigate_to', 10)
        
        # Subscriber to listen for new orders
        self.order_sub = self.create_subscription(String, '/order', self.order_callback, 10)
        
        self.get_logger().info("Order Handler Initialized!")

        # Define waypoints (X, Y) as string messages
        self.locations = {
            'home': '0.0 0.0',
            'kitchen': '-0.5 -1.5',
            'table_1': '6.0 -1.0',
            'table_2': '6.0 -6.00',
            'table_3': '6.0 2.45'
        }

    def order_callback(self, msg):
        """Receives an order and sends a navigation request."""
        order = msg.data.strip()
        if order in self.locations:
            self.get_logger().info(f"Received order for {order}. Sending navigation request.")
            nav_msg = String()
            nav_msg.data = self.locations[order]
            self.nav_pub.publish(nav_msg)
        else:
            self.get_logger().warn(f"Invalid order received: {order}")

def main(args=None):
    rclpy.init(args=args)
    node = OrderHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
