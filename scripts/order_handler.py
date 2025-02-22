#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from collections import deque
import time

class OrderHandler(Node):
    def __init__(self):
        super().__init__('order_handler')

        # Publishers & Subscribers
        self.nav_pub = self.create_publisher(String, '/navigate_to', 10)
        self.order_sub = self.create_subscription(String, '/order', self.order_callback, 10)
        self.confirm_sub = self.create_subscription(String, '/confirmation_topic', self.confirmation_callback, 10)
        self.cancel_sub = self.create_subscription(String, '/cancel', self.cancel_callback, 10)
        
        # Task State
        self.task_queue = deque()
        self.current_task = None
        self.waiting_for_confirmation = False
        self.timeout_timer = None
        self.timeout_duration = 10  # 10 seconds timeout
        self.start_time = None

        # Locations (x, y)
        self.locations = {
            'home': (0.0, 0.0),
            'kitchen': (-0.5, -1.5),
            'table_1': (6.0, -1.0),
            'table_2': (6.0, -6.0),
            'table_3': (6.0, 2.45)
        }

    def order_callback(self, msg):
        """Handles incoming order requests."""
        table = msg.data.strip().lower()
        if table in self.locations and table not in self.task_queue:
            self.task_queue.append(table)
            self.get_logger().info(f"Received order for {table}")
            if not self.current_task:
                self.process_next_order()
        else:
            self.get_logger().warn(f"Invalid or duplicate order: {table}")

    def process_next_order(self):
        """Handles the next order in queue."""
        if not self.task_queue:
            self.get_logger().info("No pending orders.")
            return

        self.current_task = self.task_queue.popleft()
        self.get_logger().info(f"Processing order: {self.current_task}")
        
        # Move to kitchen first
        self.publish_navigation_goal('kitchen')
        self.wait_for_confirmation()

    def confirmation_callback(self, msg):
        """Handles confirmation messages from kitchen or tables."""
        if not self.waiting_for_confirmation:
            return
        
        location = msg.data.strip().lower()
        if location == 'kitchen' and self.current_task:
            self.get_logger().info("Kitchen confirmation received. Moving to table.")
            self.publish_navigation_goal(self.current_task)
            self.wait_for_confirmation()
        elif location == self.current_task:
            self.get_logger().info(f"Delivery confirmed at {self.current_task}. Returning home.")
            self.current_task = None
            self.publish_navigation_goal('home')
            self.process_next_order()

    def cancel_callback(self, msg):
        """Handles order cancellations."""
        canceled_task = msg.data.strip().lower()
        if canceled_task in self.task_queue:
            self.task_queue.remove(canceled_task)
            self.get_logger().info(f"Canceled task: {canceled_task}")
        elif self.current_task == canceled_task:
            self.get_logger().info(f"Current task {canceled_task} canceled. Returning home.")
            self.current_task = None
            self.publish_navigation_goal('home')
            self.process_next_order()

    def wait_for_confirmation(self):
        """Starts timeout timer and waits for confirmation."""
        self.waiting_for_confirmation = True
        self.start_time = time.time()

        if not self.timeout_timer:
            self.timeout_timer = self.create_timer(1.0, self.check_timeout)

    def check_timeout(self):
        """Checks if waiting for confirmation has timed out."""
        if not self.waiting_for_confirmation:
            return
        
        elapsed_time = time.time() - self.start_time
        if elapsed_time > self.timeout_duration:
            self.get_logger().warn(f"Timeout at {self.current_task or 'kitchen'}. Returning home.")
            self.waiting_for_confirmation = False
            self.current_task = None
            self.publish_navigation_goal('home')
            self.process_next_order()

    def publish_navigation_goal(self, location):
        """Publishes navigation goal to move the robot."""
        if location in self.locations:
            goal_msg = String()
            goal_msg.data = f"{self.locations[location][0]} {self.locations[location][1]}"
            self.nav_pub.publish(goal_msg)
            self.get_logger().info(f"Navigating to {location}: {self.locations[location]}")


def main(args=None):
    rclpy.init(args=args)
    node = OrderHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
