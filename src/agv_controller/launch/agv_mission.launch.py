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
    
    pkg_share = get_package_share_directory('agv_controller')
    agv_params_file = os.path.join(pkg_share, 'config', 'agv_params.yaml')

    cargo_weight_arg = DeclareLaunchArgument(
        'mission_cargo_weight',
        default_value = '15.0',
        description = 'Default cargo weight for mission'
    )
    mission_cargo = LaunchConfiguration('mission_cargo_weight')

    agv_node = Node(
        package = 'agv_controller',
        executable = 'agv',
        name = 'agv_node',
        output = 'screen',
        parameters = [agv_params_file]
    )

    missions = [
        {'name':'dispatcher_alpha',
         'x':3.0,
         'y':2.0,
         'cargo':10.0
        },
        {'name':'dispatcher_beta',
         'x':-2.0,
         'y':-3.0,
         'cargo':25.0
        },
        {'name':'dispatcher_hazard',
         'x':5.2,
         'y':4.8,
         'cargo':5.0
        },
        {'name':'dispatcher_heavy',
         'x':1.0,
         'y':4.0,
         'cargo':mission_cargo
        }
    ]

    dispatcher_node = []
    for mission in missions:
        node = Node(
            package = 'agv_controller',
            executable = 'dispatcher',
            name = mission["name"],
            output = 'screen',
            parameters = [{
                    'goal_x' : mission["x"],
                    'goal_y' : mission["y"],
                    'cargo_weight' : mission["cargo"]
            }]
        )
        dispatcher_node.append(node)
    
    return LaunchDescription(
        [
            cargo_weight_arg,
            agv_node,
            *dispatcher_node
        ]
    )