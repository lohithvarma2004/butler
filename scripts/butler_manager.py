#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math
from collections import deque

def normalize_angle(angle):
    """Normalize angle to be within [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

class ButlerManager(Node):
    def __init__(self):
        super().__init__('butler_manager')

        # Publishers and subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.order_sub = self.create_subscription(String, '/order', self.order_callback, 10)
        self.confirm_sub = self.create_subscription(String, '/confirmation_topic', self.confirmation_callback, 10)
        self.cancel_sub = self.create_subscription(String, '/cancel', self.cancel_callback, 10)

        # States:
        # "at_home"        : waiting at home for orders
        # "to_kitchen"     : moving to kitchen
        # "waiting_kitchen": arrived at kitchen, waiting for confirmation (or timeout)
        # "to_table"       : moving to table for delivery
        # "waiting_table"  : arrived at table, waiting for confirmation (or timeout)
        # "to_home"        : returning home
        self.state = 'at_home'
        self.task_queue = deque()   # holds table orders (e.g. "table_1")
        self.current_task = None

        # Robot pose and goal
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.goal_x = None
        self.goal_y = None
        self.moving_to_goal = False

        # Known locations (x, y)
        self.locations = {
            'home': (0.0, 0.0),
            'kitchen': (-0.5, -1.5),
            'table_1': (6.0, -1.0),
            'table_2': (6.0, -6.0),
            'table_3': (6.0, 2.45)
        }
        self.goal_threshold = 0.5  # distance threshold for "arrival"

        # Timeout for waiting states (in seconds); increased to 20 seconds for testing
        self.waiting_timeout = 20.0  
        self.waiting_start_time = None

        self.create_timer(0.1, self.control_loop)

    # --- Callbacks ---
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
            if self.state == 'at_home' and not self.moving_to_goal:
                self.process_next_order()
        else:
            self.get_logger().warn(f"⚠️ Invalid order: {table}")

    def confirmation_callback(self, msg):
        location = msg.data.strip().lower()
        self.get_logger().info(f"Received confirmation: {location} in state {self.state}")
        if self.state == 'waiting_kitchen' and location == 'kitchen':
            self.get_logger().info("✅ Kitchen confirmation received. Food loaded.")
            if self.current_task:
                self.move_to_location(self.current_task)
                self.state = 'to_table'
            else:
                self.get_logger().warn("No current task after kitchen confirmation.")
        elif self.state == 'waiting_table' and location == self.current_task:
            self.get_logger().info(f"✅ Delivery confirmed at {self.current_task}.")
            if self.task_queue:
                self.current_task = self.task_queue.popleft()
                self.move_to_location(self.current_task)
                self.state = 'to_table'
            else:
                self.move_to_location('home')
                self.state = 'to_home'

    def cancel_callback(self, msg):
        canceled_task = msg.data.strip().lower()
        if canceled_task in self.task_queue:
            self.task_queue.remove(canceled_task)
            self.get_logger().info(f"❌ Canceled task: {canceled_task}")
        elif canceled_task == self.current_task:
            self.get_logger().info(f"❌ Current task {canceled_task} canceled. Returning home.")
            self.move_to_location('home')
            self.state = 'to_home'

    # --- State transitions ---
    def process_next_order(self):
        if not self.task_queue:
            self.get_logger().info("✅ No pending orders.")
            self.state = 'at_home'
            return
        self.current_task = self.task_queue.popleft()
        self.get_logger().info(f"🚀 Processing order: {self.current_task}")
        self.move_to_location('kitchen')
        self.state = 'to_kitchen'

    def move_to_location(self, location):
        if location in self.locations:
            self.goal_x, self.goal_y = self.locations[location]
            self.moving_to_goal = True
            self.waiting_start_time = None
            self.get_logger().info(f"🚗 Moving to {location}: {self.goal_x}, {self.goal_y}")

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.moving_to_goal = False
        now = self.get_clock().now()
        if self.state == 'to_kitchen':
            self.state = 'waiting_kitchen'
            self.waiting_start_time = now
            self.get_logger().info("⏳ Arrived at kitchen. Waiting for confirmation or timeout...")
        elif self.state == 'to_table':
            self.state = 'waiting_table'
            self.waiting_start_time = now
            self.get_logger().info("⏳ Arrived at table. Waiting for confirmation or timeout...")
        elif self.state == 'to_home':
            self.state = 'at_home'
            self.get_logger().info("✅ Returned home. Ready for new orders.")
            if self.task_queue:
                self.process_next_order()

    def handle_timeout(self, waiting_state):
        if waiting_state == 'waiting_kitchen':
            self.get_logger().info("⏰ Timeout waiting at kitchen. Returning home.")
            self.move_to_location('home')
            self.state = 'to_home'
        elif waiting_state == 'waiting_table':
            self.get_logger().info("⏰ Timeout waiting at table.")
            if self.task_queue:
                self.get_logger().info("Skipping current table and moving to next order.")
                self.current_task = self.task_queue.popleft()
                self.move_to_location(self.current_task)
                self.state = 'to_table'
            else:
                self.get_logger().info("No more table orders. Returning to kitchen then home.")
                self.move_to_location('kitchen')
                self.state = 'to_kitchen'

    # --- Control loop ---
    def control_loop(self):
        now = self.get_clock().now()
        # Check for timeout in waiting states
        if self.state in ['waiting_kitchen', 'waiting_table']:
            if self.waiting_start_time is not None:
                elapsed = (now - self.waiting_start_time).nanoseconds / 1e9
                if elapsed > self.waiting_timeout:
                    self.handle_timeout(self.state)
                    return

        if self.state == 'at_home' and self.task_queue and not self.moving_to_goal:
            self.get_logger().info("📢 Pending order found at home. Initiating next order.")
            self.process_next_order()
            return

        if self.state == 'at_home' or not self.moving_to_goal or self.goal_x is None or self.goal_y is None:
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
        angle_diff = normalize_angle(angle_to_goal - self.current_yaw)

        twist = Twist()
        if abs(angle_diff) > 0.2:
            twist.angular.z = 0.3 if angle_diff > 0 else -0.3
        else:
            twist.linear.x = 0.2
        self.cmd_pub.publish(twist)
        self.get_logger().debug(f"Distance: {distance:.2f}, Angle diff: {angle_diff:.2f}")

    def avoid_obstacle(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.get_logger().warn("🚧 Obstacle detected! Robot stopping.")

def main(args=None):
    rclpy.init(args=args)
    node = ButlerManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
