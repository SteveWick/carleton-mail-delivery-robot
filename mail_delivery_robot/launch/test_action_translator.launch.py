from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
            Node(package='mail_delivery_robot',
            namespace='tests',
            executable='action_translator_test',
            name='action_translator_test',
            output='log',
            remappings=[('/tests/actions', '/control/actions')]),
            Node(package='mail_delivery_robot',
            namespace='control',
            executable='action_translator',
            name='action_translator',
            remappings=[('/control/cmd_vel', '/tests/cmd_vel')]
            )
        ])



