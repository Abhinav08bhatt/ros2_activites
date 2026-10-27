# for YAML config file
import os
from ament_index_python.packages import get_package_share_directory

# for launch file
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

# for substations
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    pkg_share = get_package_share_directory('amr_controller')
    amr_params_file = os.path.join(pkg_share,'config','amr_params.yaml')

    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value = "NORMAL",
        description="NORMAL / BREAK(for breaking 1.5 meter from target)"
    )

    amr_node = Node(
        package= 'amr_controller',
        executable= 'amr',
        name= 'amr_node',
        output= 'screen',
        parameters=[amr_params_file]
    )

    missions = [
        {
            'name':'normal mission',
            'goal_x':3.0,
            'goal_y':3.0,
            'mode':'NORMAL',
        },
        {
            'name':'break mission',
            'goal_x':5.0,
            'goal_y':3.6,
            'mode':'BREAK',
        }
    ]

    supervisor_node = []
    for mission in missions:
        node = Node(
            package= 'amr_controller',
            executable= 'supervisor',
            name= 'supervisor_node',
            output= 'screen',  
            parameters= [
                {
                    'goal_x' : mission["goal_x"],
                    'goal_y' : mission["goal_y"],
                    'mode' : mission["mode"]
                }
            ] 
        )
        supervisor_node.append(node)

    return LaunchDescription(
        [
            mode_arg,
            amr_node,
            *supervisor_node
        ]
    )