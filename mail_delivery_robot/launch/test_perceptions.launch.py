from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
            Node(package='mail_delivery_robot',
            namespace='tests',
            executable='perceptions_test',
            name='perceptions_test',
            output='log',
            remappings=[('/tests/preceptions', '/control/preceptions')]),
            Node(package='mail_delivery_robot',
            namespace='control',
            executable='robot_driver',
            name='robot_driver',
            output='log',
            remappings=[('/control/actions', '/tests/actions')]
            )
        ])