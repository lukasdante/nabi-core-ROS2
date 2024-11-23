from launch import LaunchDescription
from launch_ros.actions import Node

axes    = ['x', 'y', 'z', 'a', 'b', 'c']
can_ids = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06]

def generate_launch_description():
    print("Generating launch description for joints")
    return LaunchDescription([
        Node(
            namespace=f'{axis}',
            package='nabi',
            executable='joint',
            name='joint',
            parameters=[
                {'can_id': can_id}
            ]
        ) for axis, can_id in zip(axes, can_ids)
    ])
