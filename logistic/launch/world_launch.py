from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('logistic')

    # Specify world file path
    world_file_path = os.path.join(pkg_share, 'worlds', 'model.sdf')

    # Verify that the world file exists
    if not os.path.exists(world_file_path):
        raise FileNotFoundError(f"World file not found at {world_file_path}")

    # Specify the bridge YAML file path
    bridge_yaml_path = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')

    # Verify that the bridge YAML file exists
    if not os.path.exists(bridge_yaml_path):
        raise FileNotFoundError(f"Gazebo bridge config file not found at {bridge_yaml_path}")

    # Launch Gazebo with the specified world
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', world_file_path],
        output='screen'
    )

    # Launch the ROS-Gazebo bridge directly
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_yaml_path}',],
        output='screen'
    )

    # Create and return launch description
    return LaunchDescription([
        gazebo,
        ros_gz_bridge
    ])
