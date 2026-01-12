import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    pkg_name = 'cocaleaf'

    pkg_share = get_package_share_directory(pkg_name)
    
    # XACRO
    xacro_file = os.path.join(pkg_share, 'urdf', 'brazo3.urdf.xacro')
    
    # configuración de RViz
    rviz_config_path = os.path.join(pkg_share, 'config', '/home/fabricio/Desktop/simulacion_robot.rviz')
    robot_description = Command(['xacro ', xacro_file])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        ),
        Node(
            package=pkg_name, 
            executable='visio',
            name='vision_node',
            output='screen'
        ),
        Node(
            package=pkg_name, 
            executable='cal',
            name='cinematica_node',
            output='screen'
        ),
        Node(
            package=pkg_name, 
            executable='CPU',
            name='cerebro_node',
            output='screen'
        ),

        Node(
            package=pkg_name, 
            executable='simu',
            name='simulacion_node',
            output='screen'
        )
    ])