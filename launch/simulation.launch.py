from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():

    descr_path = get_package_share_path('dexhand_description')
    default_model_path = descr_path / 'urdf/dexhand-left.xacro'

    dexhand_manus_path = get_package_share_path('dexhand_manus')
    rviz_path = dexhand_manus_path / 'rviz/simulation.rviz'


    urdf = ParameterValue(Command(['xacro ', str(default_model_path)]),
                                     value_type=str)

    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf}]),
        Node(
            package='manus_ros2',
            executable='manus_ros2',
            name='manus_ros2',
            output='screen'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', str(rviz_path)]),
    ])