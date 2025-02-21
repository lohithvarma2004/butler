#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class ExceptionHandler(Node):
    def __init__(self):
        super().__init__('exception_handler')

        # Subscribers
        self.order_sub = self.create_subscription(String, '/order', self.order_callback, 10)
        self.cancel_sub = self.create_subscription(String, '/cancel', self.cancel_callback, 10)

        # Publisher to send navigation requests
        self.nav_pub = self.create_publisher(String, '/navigate_to', 10)

        # Track active order and time
        self.active_order = None
        self.order_start_time = None

        # Timer to check for timeouts every second
        self.create_timer(1.0, self.check_timeout)

    def order_callback(self, msg):
        """Handles a new order request."""
        if self.active_order:
            self.get_logger().warn(f"Already processing order {self.active_order}. Ignoring new order.")
            return  # Ignore new orders while one is active

        self.active_order = msg.data
        self.order_start_time = time.time()
        self.get_logger().info(f"Order {self.active_order} started!")

    def cancel_callback(self, msg):
        """Handles order cancellation."""
        cancel_order = msg.data
        if self.active_order == cancel_order:
            self.get_logger().info(f"Order {cancel_order} canceled! Returning home.")
            self.return_home()

    def check_timeout(self):
        """Checks if the active order has exceeded the allowed time limit."""
        if self.active_order and time.time() - self.order_start_time > 180:  # 15-second timeout
            self.get_logger().warn(f"Order {self.active_order} timed out! Returning home.")
            self.return_home()

    def return_home(self):
        """Sends the robot back to the home position."""
        nav_msg = String()
        nav_msg.data = "0.0 0.0"  # Home coordinates
        self.nav_pub.publish(nav_msg)
        self.active_order = None  # Reset active order

def main(args=None):
    rclpy.init(args=args)
    node = ExceptionHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
