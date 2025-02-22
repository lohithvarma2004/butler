#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math

class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('simple_navigator')

        # ROS2 Publishers & Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.nav_sub = self.create_subscription(PoseStamped, '/goal_pose', self.nav_callback, 10)

        # State variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.goal_x = None
        self.goal_y = None
        self.moving_to_goal = False
        self.obstacle_detected = False
        self.safety_distance = 0.5  # Stop 50 cm before reaching the goal

        self.create_timer(0.1, self.control_loop)  # Control loop at 10 Hz

    def scan_callback(self, msg):
        """Process LiDAR data to detect obstacles."""
        min_distance = min(msg.ranges) if msg.ranges else float('inf')

        if min_distance < self.safety_distance:  
            self.obstacle_detected = True
            self.get_logger().warn("Obstacle detected! Adjusting path.")
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
        """Receive navigation requests from `/goal_pose` topic."""
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.moving_to_goal = True
        self.get_logger().info(f"Moving to ({self.goal_x}, {self.goal_y})")

    def control_loop(self):
        """Navigation logic executed continuously."""
        if not self.moving_to_goal or self.goal_x is None or self.goal_y is None:
            return  

        if self.obstacle_detected:
            self.avoid_obstacle()
            return

        distance = math.sqrt((self.goal_x - self.current_x) ** 2 + (self.goal_y - self.current_y) ** 2)

        if distance < self.safety_distance:  
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
        """Obstacle avoidance behavior."""
        self.get_logger().warn("Obstacle detected! Adjusting movement.")

        twist = Twist()
        twist.linear.x = -0.1  
        self.cmd_pub.publish(twist)
        rclpy.sleep(1.0)  

        twist.linear.x = 0.0
        twist.angular.z = 0.5  
        self.cmd_pub.publish(twist)

    def stop_robot(self):
        """Stops the robot completely."""
        self.send_velocity(0.0, 0.0)
        self.moving_to_goal = False
        self.get_logger().info("Reached destination safely!")

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
