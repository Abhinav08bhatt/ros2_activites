from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    pkg_name = 'amr_controller'

    profile_argument = DeclareLaunchArgument(
        'profile',
        default_value='default',
        description='config profile to load : ( default , break , fast_nav , long_nav , negative_nav )'
    )

    selected_config = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'config',
        [LaunchConfiguration('profile'),TextSubstitution(text= ".yaml")]
        ]
    )

    amr_node = Node(
        package="amr_controller",
        executable="amr",
        name="amr_node",
        output="screen",
        parameters=[selected_config] 
    )

    supervisor_node = Node(
        package="amr_controller",
        executable="supervisor",
        name="supervisor_node",
        output="screen",
        parameters=[selected_config] 
    )

    return LaunchDescription(
        [
            profile_argument,
            amr_node,
            supervisor_node
        ]
    )