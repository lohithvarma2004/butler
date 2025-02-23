# Butler Manager ROS2 Package

## Overview
The **Butler Manager** package provides a complete setup for simulating a service robot in ROS2 and Gazebo. The robot, referred to as the “butler,” navigates from its home position to the kitchen to pick up food orders, then proceeds to designated tables for delivery, and finally returns home. It manages multiple orders, handles timeouts and cancellations, and uses odometry and laser data for navigation.

## Package Contents

1. **State Machine Node (butler_manager.py)**  
   - Manages the robot’s behavior via states:
     - at_home / idle: Waiting for orders.
     - to_kitchen: Moving to the kitchen.
     - waiting_kitchen: At kitchen, awaiting confirmation (or timeout).
     - to_table: Moving to the requested table.
     - waiting_table: At table, awaiting confirmation (or timeout).
     - to_home: Returning to home.
   - Listens on:
     - /order for new orders (e.g., "table_1").
     - /confirmation_topic for confirmations (e.g., "kitchen", "table_1").
     - /cancel for cancellations.
   - Publishes velocity commands on /cmd_vel, stops if obstacles are too close (via /scan data).
   - Maintains a queue for multiple orders, each requiring a trip to the kitchen first.

2. **Robot Model Files**
   - robot.xacro: Defines the physical structure of the differential-drive robot. Key points:
     - Body, wheels, caster, and a LiDAR sensor link.
     - Inertial properties are calculated using macros (inertia_wheel, inertia_caster, etc.).
     - A laser_joint attaches the LiDAR sensor to the robot’s main body.
     - References robot.gazebo for Gazebo-specific configurations.
   - robot.gazebo: Inserts Gazebo plugins and material/friction parameters:
     - <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so"> for differential-drive control.
     - <sensor name="laser" type="ray"> plugin for publishing laser scans (/scan).
     - Material and friction properties for each link (body, wheels, caster).
     - Corrected references for joints (e.g., wheel1_joint, wheel2_joint).

3. **Launch File (launch/your_launch_file.py)**
   - Loads Gazebo with the specified world (hotel.world).
   - Spawns the robot using spawn_entity.py.
   - Runs robot_state_publisher to broadcast TF transforms for the robot’s links.
   - Points to robot.xacro to generate the robot description at runtime.

4. **World File (hotel.world)**
   - Defines the simulation environment, including the kitchen and table positions.
   - You can adjust <pose> tags for each table to ensure the robot has enough clearance.

## Installation

1. Clone the Repository:
   cd ~/ros2_ws/src  
   git clone https://github.com/lohithvarma2004/butler.git

2. Build the Package:
   cd ~/ros2_ws  
   colcon build --packages-select butler

3. Source the Workspace:
   source ~/ros2_ws/install/setup.bash

## Usage

1. Launch Gazebo with the World File:
   ros2 launch butler <launch_file>.py
   (This starts Gazebo, spawns the robot, and runs robot_state_publisher.)

2. Run the Butler Manager Node:
   ros2 run butler butler_manager.py

3. Send Orders:
   ros2 topic pub --once /order std_msgs/msg/String "data: 'table_1'"
   (The robot travels home → kitchen → table → home for each order.)

4. Confirmations (If Required):
   ros2 topic pub --once /confirmation_topic std_msgs/msg/String "data: 'kitchen'"
   ros2 topic pub --once /confirmation_topic std_msgs/msg/String "data: 'table_1'"

5. Cancel Orders:
   ros2 topic pub --once /cancel std_msgs/msg/String "data: 'table_1'"

## Robot & Plugins

- Differential Drive Plugin:
  - Defined in robot.gazebo with <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">.
  - Subscribes to /cmd_vel internally to drive the wheels.
  - Publishes odometry on /odom.

- Laser Sensor Plugin:
  - <sensor name="laser" type="ray"> in robot.gazebo.
  - Publishes sensor_msgs/LaserScan messages on /scan.
  - Used by butler_manager.py to detect obstacles and stop if too close.

- Xacro Macros:
  - inertia_wheel, inertia_caster, and inertial_cylinder macros calculate mass/inertia.
  - Invoked for each wheel and the caster link in robot.xacro.

## Configuration & Customization

- Locations: In butler_manager.py, self.locations maps "kitchen", "table_1", etc. to (x, y) coordinates. Adjust these to match hotel.world.
- Timeout & Thresholds:
  - self.waiting_timeout sets how long the robot waits for confirmation.
  - self.goal_threshold sets how close the robot must get before considering it “arrived.”
- World File: Update <pose> in hotel.world for each table if collisions occur or if you want a different layout.

## Troubleshooting

- Robot Not Moving: Check if /cmd_vel is being published, and verify no warnings about missing plugins or transforms.
- Collisions: Tables may be too close. Adjust poses or add advanced obstacle avoidance.
- State Machine Issues: Ensure orders are valid, no duplicates, and confirmations are sent before timeout.

## License
MIT

## Author
Lohith
