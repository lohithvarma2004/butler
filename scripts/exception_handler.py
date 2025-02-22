#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class ExceptionHandler(Node):
    def __init__(self):
        super().__init__('exception_handler')

        # Subscribers to handle orders and cancellations
        self.order_sub = self.create_subscription(String, '/order', self.order_callback, 10)
        self.cancel_sub = self.create_subscription(String, '/cancel', self.cancel_callback, 10)

        # Publisher to reset robot movement
        self.reset_pub = self.create_publisher(String, '/reset_robot', 10)

        # Active order tracking
        self.active_order = None
        self.order_start_time = None
        self.timeout_duration = 15  # Timeout after 15 seconds
        self.create_timer(1.0, self.check_timeout)  # Check timeout every second

    def order_callback(self, msg):
        """Handles new orders and starts tracking timeout."""
        if self.active_order:
            self.get_logger().warn(f"Already processing order {self.active_order}. Ignoring new order {msg.data}.")
            return
        
        self.active_order = msg.data.strip()
        self.order_start_time = time.time()
        self.get_logger().info(f"Order {self.active_order} started!")

    def cancel_callback(self, msg):
        """Handles order cancellation by stopping the robot and clearing order."""
        cancel_order = msg.data.strip()
        if self.active_order == cancel_order:
            self.get_logger().info(f"Order {cancel_order} canceled! Stopping robot and returning home.")
            self.active_order = None
            self.reset_robot()
        else:
            self.get_logger().warn(f"Received cancel request for {cancel_order}, but no active order matches.")

    def check_timeout(self):
        """Monitors if the robot exceeds its allowed time for an order."""
        if self.active_order and time.time() - self.order_start_time > self.timeout_duration:
            self.get_logger().warn(f"Order {self.active_order} timed out! Returning home.")
            self.active_order = None
            self.reset_robot()

    def reset_robot(self):
        """Stops the robot and resets its state."""
        reset_msg = String()
        reset_msg.data = "reset"
        self.reset_pub.publish(reset_msg)
        self.get_logger().info("Robot reset signal sent.")

def main(args=None):
    rclpy.init(args=args)
    node = ExceptionHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
