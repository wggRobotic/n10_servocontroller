import os
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    package_path = FindPackageShare('n10_servo_controller')
    parameter_file = PathJoinSubstitution([
        package_path,
        'parameter',
        'n10_servo_controller_pca.yaml'
    ])

    n10_servocontroller = Node(
        package='n10_servo_controller',
        executable='n10_servo_controller',
        name='servo_controller_pca',
        parameters=[parameter_file],
        namespace="n10",
        output='screen'
    )

    return LaunchDescription([
        n10_servocontroller
    ])