from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    print("Generating launch description for joints")
    return LaunchDescription([
        Node(
            namespace='x_axis',
            package='nabi',
            executable='joint',
            name='jointx',
        ),
        Node(
            namespace=f'y_axis',
            package='nabi',
            executable='joint',
            name=f'jointy',
        ),
        # Node(
        #     namespace=f'z_axis',
        #     package='nabi',
        #     executable='joint',
        #     name=f'joint',
        # ),
        # Node(
        #     namespace=f'a_axis',
        #     package='nabi',
        #     executable='joint',
        #     name=f'joint',
        # ),
        # Node(
        #     namespace=f'b_axis',
        #     package='nabi',
        #     executable='joint',
        #     name=f'joint',
        # ),
        # Node(
        #     namespace=f'c_axis',
        #     package='nabi',
        #     executable='joint',
        #     name=f'joint',
        # ),
    ])
