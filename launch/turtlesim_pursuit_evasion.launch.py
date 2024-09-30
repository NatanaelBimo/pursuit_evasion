from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='turtlesim',
#             executable='turtlesim_node',
#             name='sim'
#         ),
#         Node(
#             package='pursuit_evasion',
#             executable='turtle_tf2_broadcaster',
#             name='broadcaster1',
#             parameters=[
#                 {'turtlename': 'turtle1'}
#             ]
#         ),
#         DeclareLaunchArgument(
#             'target_turtle', default_value='turtle1',
#             description='Target turtle name.'
#         ),
#         Node(
#             package='pursuit_evasion',
#             executable='turtle_tf2_broadcaster',
#             name='broadcaster2',
#             parameters=[
#                 {'turtlename': 'turtlePursuer'}
#             ]
#         ),
#         Node(
#             package='pursuit_evasion',
#             executable='pursuer',
#             name='listener',
#             parameters=[
#                 {'target_turtle': LaunchConfiguration('target_turtle')}
#             ]
#         ),
#     ])

def generate_launch_description():
    return LaunchDescription([
        # Launch turtlesim node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        
        # TF Broadcaster for turtle1 (evader)
        Node(
            package='pursuit_evasion',
            executable='turtle_tf2_broadcaster',
            name='broadcaster1',
            parameters=[
                {'turtlename': 'turtle1'}  # 'turtle1' will be the evader
            ]
        ),
        
        # TF Broadcaster for turtlePursuer (pursuer)
        Node(
            package='pursuit_evasion',
            executable='turtle_tf2_broadcaster',
            name='broadcaster2',
            parameters=[
                {'turtlename': 'turtlePursuer'}
            ]
        ),
        
        # Pursuer node
        DeclareLaunchArgument(
            'target_turtle', default_value='turtle1',
            description='Target turtle name.'
        ),
        Node(
            package='pursuit_evasion',
            executable='pursuer',
            name='pursuer',
            parameters=[
                {'target_turtle': LaunchConfiguration('target_turtle')}
            ]
        ),
        
        # Evader node for turtle1
        Node(
            package='pursuit_evasion',
            executable='evader',
            name='evader',
            parameters=[
                {'pursuer_turtle': 'turtlePursuer'}
            ]
        )
    ])

