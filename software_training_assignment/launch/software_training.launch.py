from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='',
            executable='turtlesim_node',
            name='turtle1'
        ),
#        Node(
#            package='software_training_assignment',
#            namespace='',
#            executable='turtle_request',
#            name='neutralize'
#        ),
        Node(
            package='software_training_assignment',
            namespace='',
            executable='cmd_vel_publisher',
            name='turtle_move_in_circle'
        ),
        Node(
            package='software_training_assignment',
            namespace='',
            executable='turtle_spawn',
            name='turtle_spawn'
        ),
        Node(
            package='software_training_assignment',
            namespace='',
            executable='turtle_service',
            name='turtle_service'
        ),
        Node(
            package='software_training_assignment',
            namespace='',
            executable='turtle_pub',
            name='turtle_pub'
        ),
        Node(
            package='software_training_assignment',
            namespace='',
            executable='moving_turtle_action_server',
            name='turtle_action_server'
        )
        ])
