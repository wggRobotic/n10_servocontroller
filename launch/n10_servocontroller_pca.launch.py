import os
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    package_path = FindPackageShare('n10_servocontroller')
    parameter_file = PathJoinSubstitution([
        package_path,
        'parameter',
        'n10_servocontroller_pca.yaml'
    ])

    n10_servocontroller = Node(
        package='n10_servocontroller',
        executable='n10_servocontroller',
        name='servocontroller',
        parameters=[parameter_file],
        namespace="n10",
        output='screen'
    )

    return LaunchDescription([
        n10_servocontroller
    ])