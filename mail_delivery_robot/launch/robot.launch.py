from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    robot_model = DeclareLaunchArgument(
        "robot_model", default_value=TextSubstitution(text="CREATE_1"))

    return LaunchDescription([
        robot_model,
        Node(package='create_driver',
             executable='create_driver',
             name='create_driver',
             output="screen",
             parameters=[{
                 "robot_model": LaunchConfiguration('robot_model')
             }],
            remappings=[
             ('/cmd_vel', '/control/cmd_vel')]
             ),
        Node(package='mail_delivery_robot',
            namespace='preceptions',
            executable='IRSensor',
            name='IRSensor',
            output="log",
            remappings=[('/preceptions/preceptions', '/control/preceptions')]
            ),
        Node(package='mail_delivery_robot',
            namespace='preceptions',
            executable='beacon_reader',
            name='beacon_reader',
            remappings=[('/preceptions/beacons', '/navigation/beacons')]
            ),
        Node(package='mail_delivery_robot',
            namespace='control',
            executable='action_translator',
            name='action_translator',
            ),
        Node(package='mail_delivery_robot',
            namespace='control',
            executable='robot_driver',
            name='robot_driver',
            remappings=[
             ('/control/navigationMap', '/navigation/navigationMap'),
             ('/control/bumpEvent', '/preceptions/bumpEvent')]
            ),
        Node(package='mail_delivery_robot',
            namespace='navigation',
            executable='captain',
            name='captain'),
        Node(package='mail_delivery_robot',
            namespace='preceptions',
            executable='bumper_sensor',
            name='bumper_sensor',
            remappings=[('/preceptions/bumper', '/bumper')]),
            
    ])
