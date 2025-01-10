import os

# Import necessary ROS 2 packages and modules
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
from launch.event_handlers import OnProcessExit
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Find the package share directory for the 'robot_urdf' package
    test_robot_description_share = FindPackageShare(package='robot_urdf').find('robot_urdf')
    # Define the default path to the robot's URDF file
    default_model_path = os.path.join(test_robot_description_share, 'urdf/robot4.xacro')
    # Define the path to the RViz configuration file
    rviz_config_path = os.path.join(test_robot_description_share, 'config/rviz.rviz')
    # Define the path to the Gazebo world file
    default_world_path = os.path.join(test_robot_description_share, 'worlds/world_markers.world')

    # Node to publish the robot state
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    
    # Node to publish the joint states of the robot
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    # Node to spawn the robot entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', 'my_test_robot', '-topic', '/robot_description', '-x', '0.025', '-y', '0.015'],
        output='screen'
    )
    
    # Define the path to the camera configuration file and check its existence
    camera_config_path = os.path.join(get_package_share_directory('robot_urdf'), 'config', 'config.yaml')
    if not os.path.exists(camera_config_path):
        raise FileNotFoundError(f"Parameter file {camera_config_path} does not exist")

    # Node to spawn the joint velocity controller and joint state broadcaster
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_velocity_controller', 'joint_state_broadcaster'],
        output='screen'
    )

    # Node to run the ArUco marker detection
    aruco_node = Node(
        package='ros2_aruco',
        executable='aruco_node',
        name='aruco_node',
        output='screen'
    )
    
    # Custom NodeB for camera rotation
    NodeB = Node(
        package='robot_urdf',
        executable='NodeB.py',
        name='NodeB',
        output='screen'
    )

    # Return the launch description including all nodes and processes
    return LaunchDescription([
        # Declare the launch argument for the robot model
        DeclareLaunchArgument(
            name='model', 
            default_value=default_model_path,
            description='Absolute path to robot urdf file'
        ),
        spawn_entity,
        # Execute Gazebo with the specified world file and factory plugin
        ExecuteProcess(
            cmd=['gazebo', '--verbose', default_world_path, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        # Execute RViz with the specified configuration file
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_path],
            output='screen'
        ),
        controller_spawner,
        robot_state_publisher_node,
        joint_state_publisher_node,
        aruco_node,
        NodeB
    ])
