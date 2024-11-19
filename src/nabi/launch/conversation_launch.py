from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nabi',
            executable='recorder',
            name='sim'
        ),
        Node(
            package='nabi',
            executable='writer',
            name='sim'
        ),
        Node(
            package='nabi',
            executable='parser',
            name='sim'
        ),
        Node(
            package='nabi',
            executable='talker',
            name='sim'
        )
    ])