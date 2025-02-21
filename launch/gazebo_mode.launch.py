import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # This name has to match the robot name in the Xacro file
    robotXacroName = 'differential_drive_robot'

    namePackage = 'butler'

    modeFileRelativePath = 'model/robot.xacro'

    worldFileRelativePath = 'model/hotel.world'

    pathModelFile = os.path.join(get_package_share_directory(namePackage), modeFileRelativePath)

    # This is the absolute path to the world model
    pathWorldFile = os.path.join(get_package_share_directory(namePackage), worldFileRelativePath)

    robotDescription = xacro.process_file(pathModelFile).toxml()

    # This is the launch file from the gazebo_ros package
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    )
    
    # This is the launch description
    gazeboLaunch = IncludeLaunchDescription(
        gazebo_rosPackageLaunch, launch_arguments={'world': pathWorldFile}.items()
    )

    # Here, we create a gazebo_ros Node
    spawnModelNode = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', robotXacroName], output='screen'
    )
    
    # Robot State Publisher Node
    nodeRobotStatePublisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output='screen',
        parameters=[{'robot_description': robotDescription, 'use_sim_time': True}]
    )   

    # Here we create an empty launch description object
    launchDescriptionObject = LaunchDescription()

    # We add gazeboLaunch
    launchDescriptionObject.add_action(gazeboLaunch)

    # We add two nodes
    launchDescriptionObject.add_action(spawnModelNode)
    launchDescriptionObject.add_action(nodeRobotStatePublisher)
    
    return launchDescriptionObject