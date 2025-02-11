import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Get package path
    pkg_path = get_package_share_directory('dog_example_pkg')
    
    # Define paths
    rviz_config_path = os.path.join(pkg_path, 'config', 'config.rviz')  # Doesn't matter that much
    xacro_file_path = os.path.join(pkg_path,'description','dog.urdf.xacro')
    
    robot_description_config = xacro.process_file(xacro_file_path)
    params = {'robot_description': robot_description_config.toxml()}

    # Robot State Publisher (publishes transforms for the robot model)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # # Joint State Publisher (publishes joint states)
    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     parameters=[params]
    # )

    # # Joint State Publisher GUI
    # joint_state_publisher_gui_node = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui",
    #     name="joint_state_publisher_gui",
    # )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]  # Loads RViz config file
    )

    return LaunchDescription([
        robot_state_publisher_node,
        # joint_state_publisher_node,
        # joint_state_publisher_gui_node,
        rviz_node
    ])
