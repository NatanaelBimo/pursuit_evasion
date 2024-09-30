from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),

        Node(
            package='pursuit_evasion_v2',
            executable='worldset',
            name='turtle_spawn',
            output='screen'
        ),
        Node(
            package='pursuit_evasion_v2',
            executable='evader',
            name='turtle_evader',
            output='screen'
        ),
        
        # Launch the pursuer node
        Node(
            package='pursuit_evasion_v2',
            executable='pursuer',
            name='turtle_pursuer',
            output='screen'
        ),

    ])
