#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math
import numpy as np

class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('simple_navigator')

        # ROS2 Publishers & Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.nav_sub = self.create_subscription(String, '/navigate_to', self.nav_callback, 10)

        # Variables for navigation
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.goal_x = None
        self.goal_y = None
        self.obstacle_detected = False
        self.moving_to_goal = False
        self.safe_distance = 0.6  # Distance to maintain from obstacles

        # Timer for continuous control (runs at 10 Hz)
        self.create_timer(0.1, self.control_loop)

    def scan_callback(self, msg):
        """Process LiDAR scan data to detect obstacles and determine avoidance direction."""
        ranges = np.array(msg.ranges)
        ranges[ranges == 0] = float('inf')  # Ignore invalid readings

        min_distance = np.min(ranges)  # Closest object distance
        min_index = np.argmin(ranges)  # Angle index of closest object

        if min_distance < self.safe_distance:
            self.obstacle_detected = True
            self.avoidance_direction = 'left' if min_index > len(ranges) // 2 else 'right'
            self.get_logger().warn(f"Obstacle detected {min_distance:.2f}m away! Turning {self.avoidance_direction}.")
        else:
            self.obstacle_detected = False

    def odom_callback(self, msg):
        """Update the robot's position and orientation from odometry."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Extract yaw (rotation) from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def nav_callback(self, msg):
        """Receives navigation request from OrderHandler."""
        x, y = map(float, msg.data.split())
        self.move_to(x, y)

    def move_to(self, x, y):
        """Set a goal for the robot to navigate to."""
        self.goal_x = x
        self.goal_y = y
        self.moving_to_goal = True
        self.get_logger().info(f"Moving to ({x}, {y})")

    def control_loop(self):
        """Main navigation logic executed repeatedly by a ROS2 timer."""
        if not self.moving_to_goal or self.goal_x is None or self.goal_y is None:
            return  # Do nothing if no goal is set

        if self.obstacle_detected:
            self.avoid_obstacle()
            return

        distance = math.sqrt((self.goal_x - self.current_x) ** 2 + (self.goal_y - self.current_y) ** 2)

        if distance < 0.3:  # Stop if close enough to the goal
            self.stop_robot()
            return

        angle_to_goal = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
        angle_diff = angle_to_goal - self.current_yaw

        twist = Twist()
        if abs(angle_diff) > 0.2:  # Rotate towards the goal first
            twist.angular.z = 0.3 if angle_diff > 0 else -0.3
        else:
            twist.linear.x = 0.2  # Move forward when aligned
        self.cmd_pub.publish(twist)

    def avoid_obstacle(self):
        """Improved obstacle avoidance: turn in the best direction instead of blindly reversing."""
        self.get_logger().warn(f"Avoiding obstacle by turning {self.avoidance_direction}.")

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.5 if self.avoidance_direction == 'left' else -0.5
        self.cmd_pub.publish(twist)

    def stop_robot(self):
        """Stops the robot completely."""
        self.send_velocity(0.0, 0.0)
        self.moving_to_goal = False
        self.get_logger().info("Reached destination!")

    def send_velocity(self, linear_x, angular_z):
        """Publish velocity commands."""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
