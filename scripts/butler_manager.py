#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math
from collections import deque

class ButlerManager(Node):
    def __init__(self):
        super().__init__('butler_manager')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.order_sub = self.create_subscription(String, '/order', self.order_callback, 10)
        self.confirm_sub = self.create_subscription(String, '/confirmation_topic', self.confirmation_callback, 10)
        self.cancel_sub = self.create_subscription(String, '/cancel', self.cancel_callback, 10)

        # State machine states:
        # 'idle'           : waiting for new order (not at home)
        # 'at_home'        : robot is home and waiting (this prevents oscillation)
        # 'to_kitchen'     : moving to kitchen for food pickup
        # 'waiting_kitchen': arrived at kitchen, waiting for confirmation
        # 'to_table'       : moving to table for delivery
        # 'waiting_table'  : arrived at table, waiting for confirmation
        # 'to_home'        : returning home after delivery
        self.state = 'at_home'
        self.task_queue = deque()
        self.current_task = None

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.goal_x = None
        self.goal_y = None
        self.moving_to_goal = False

        self.locations = {
            'home': (0.0, 0.0),
            'kitchen': (-0.5, -1.5),
            'table_1': (6.0, -1.0),
            'table_2': (6.0, -6.0),
            'table_3': (6.0, 2.45)
        }
        self.goal_threshold = 0.4

        self.create_timer(0.1, self.control_loop)

    def scan_callback(self, msg):
        if msg.ranges:
            min_distance = min(msg.ranges)
        else:
            min_distance = float('inf')
        self.obstacle_detected = min_distance < 0.5
        if self.obstacle_detected:
            self.get_logger().warn("🚧 Obstacle detected! Stopping.")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def order_callback(self, msg):
        table = msg.data.strip().lower()
        if table in self.locations:
            if table == self.current_task or table in self.task_queue:
                self.get_logger().warn(f"⚠️ Duplicate order: {table}")
                return
            self.task_queue.append(table)
            self.get_logger().info(f"📦 Received order for {table}")
            # Accept new orders when idle or at home.
            if self.state in ['idle', 'at_home']:
                self.process_next_order()
        else:
            self.get_logger().warn(f"⚠️ Invalid order: {table}")

    def process_next_order(self):
        if not self.task_queue:
            self.get_logger().info("✅ No pending orders.")
            self.state = 'at_home'
            return
        self.current_task = self.task_queue.popleft()
        self.get_logger().info(f"🚀 Processing order: {self.current_task}")
        # For each order, start by picking up food at the kitchen.
        self.move_to_location('kitchen')
        self.state = 'to_kitchen'

    def confirmation_callback(self, msg):
        location = msg.data.strip().lower()
        if self.state == 'waiting_kitchen' and location == 'kitchen':
            self.get_logger().info("✅ Kitchen confirmation received. Food loaded.")
            self.move_to_location(self.current_task)
            self.state = 'to_table'
        elif self.state == 'waiting_table' and location == self.current_task:
            self.get_logger().info(f"✅ Delivery confirmed at {self.current_task}.")
            self.move_to_location('home')
            self.state = 'to_home'

    def cancel_callback(self, msg):
        canceled_task = msg.data.strip().lower()
        if canceled_task in self.task_queue:
            self.task_queue.remove(canceled_task)
            self.get_logger().info(f"❌ Canceled task: {canceled_task}")
        elif canceled_task == self.current_task:
            self.get_logger().info(f"❌ Current task {canceled_task} canceled.")
            self.move_to_location('home')
            self.state = 'to_home'

    def move_to_location(self, location):
        if location in self.locations:
            self.goal_x, self.goal_y = self.locations[location]
            self.moving_to_goal = True
            self.get_logger().info(f"🚗 Moving to {location}: {self.goal_x}, {self.goal_y}")

    def control_loop(self):
        # When idle or at home, publish zero velocities.
        if self.state in ['idle', 'at_home'] or not self.moving_to_goal or self.goal_x is None or self.goal_y is None:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            return

        if getattr(self, 'obstacle_detected', False):
            self.avoid_obstacle()
            return

        distance = math.sqrt((self.goal_x - self.current_x)**2 + (self.goal_y - self.current_y)**2)
        if distance < self.goal_threshold:
            self.stop_robot()
            return

        angle_to_goal = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
        angle_diff = angle_to_goal - self.current_yaw

        twist = Twist()
        if abs(angle_diff) > 0.2:
            twist.angular.z = 0.3 if angle_diff > 0 else -0.3
        else:
            twist.linear.x = 0.2
        self.cmd_pub.publish(twist)

    def avoid_obstacle(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.get_logger().warn("🚧 Obstacle detected! Robot stopping.")

    def stop_robot(self):
        # Stop movement and reset the moving flag to prevent oscillation.
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.moving_to_goal = False

        if self.state == 'to_kitchen':
            self.state = 'waiting_kitchen'
            self.get_logger().info("⏳ Arrived at kitchen. Waiting for confirmation...")
        elif self.state == 'to_table':
            self.state = 'waiting_table'
            self.get_logger().info("⏳ Arrived at table. Waiting for confirmation...")
        elif self.state == 'to_home':
            self.get_logger().info("✅ Returned home. Robot is at home and ready for new orders.")
            # Reset state variables so new orders can be processed immediately.
            self.current_task = None
            self.state = 'at_home'
            self.goal_x = None
            self.goal_y = None
            # If pending orders exist, process the next order.
            if self.task_queue:
                self.process_next_order()

def main(args=None):
    rclpy.init(args=args)
    node = ButlerManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
