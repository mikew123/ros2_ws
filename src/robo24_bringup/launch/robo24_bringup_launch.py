
# MRW 1/25/2024 Added robot_localization

import launch
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    efk_config = os.path.join(
        get_package_share_directory('robo24_localization'),
        'config',
        'efk_config.yaml'
        )
 
    return launch.LaunchDescription([
        
        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            name='joy_xbox'
        ),

        launch_ros.actions.Node(
            package='robo24_wheel_interface',
            executable='robo24_teleop_node',
            name='robo24_teleop'
        ),

        launch_ros.actions.Node(
            package='robo24_wheel_interface',
            executable='robo24_wheel_controller_node',
            name='robo24_wheel_controller'
        ),

        launch_ros.actions.Node(
            package='robo24_wheel_interface',
            executable='robo24_diynav_node',
            name='robo24_diynav'
        ),

        # TODO: remove when integrated into robo24_teleop
        launch_ros.actions.Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist',
            parameters=[{
                "enable_button": 9,
                "axis_linear.x": 1,
                "axis_angular.yaw": 0,
            }]
        ),
   
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='efk_odom',
            parameters=[efk_config]
        )
  ])
