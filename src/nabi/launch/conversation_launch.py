from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nabi',
            executable='recorder',
            name='recorder',
            parameters=[
                {'threshold': 5000},
            ]
        ),
        Node(
            package='nabi',
            executable='writer',
            name='writer'
        ),
        Node(
            package='nabi',
            executable='parser',
            name='parser'
        ),
        Node(
            package='nabi',
            executable='talker',
            name='talker'
        )
    ])