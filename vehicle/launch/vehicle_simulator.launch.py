from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    topic_name_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='vehicle_speed',
        description='Topic name to publish vehicle speed'
    )
    
    speed_limit_arg = DeclareLaunchArgument(
        'speed_limit',
        default_value='50.0',
        description='Maximum speed limit in km/h'
    )
    
    initial_rate_arg = DeclareLaunchArgument(
        'initial_rate',
        default_value='1.0',
        description='Initial publishing rate in Hz'
    )

    # Node configuration
    vehicle_simulator_node = Node(
        package='vehicle',
        executable='vehicle_simulator',
        name='vehicle_simulator',
        parameters=[{
            'topic_name': LaunchConfiguration('topic_name'),
            'speed_limit': LaunchConfiguration('speed_limit'),
            'initial_rate': LaunchConfiguration('initial_rate'),
        }]
    )

    return LaunchDescription([
        topic_name_arg,
        speed_limit_arg,
        initial_rate_arg,
        vehicle_simulator_node,
    ])

