
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
            package='robo24_stuff',
            executable='robo24_can_xy_node',
            name='robo24_can_xy'
        ),

        launch_ros.actions.Node(
            package='robo24_stuff',
            executable='sensor_serial_node',
            name='sensor_serial'
        ),

        launch_ros.actions.Node(
            package='robo24_stuff',
            executable='openmv_serial_node',
            name='openmv_serial'
        ),

        launch_ros.actions.Node(
            package='robo24_stuff',
            executable='robo24_teleop_node',
            name='robo24_teleop'
        ),

        launch_ros.actions.Node(
            package='robo24_stuff',
            executable='robo24_wheel_controller_node',
            name='robo24_wheel_controller'
        ),

        launch_ros.actions.Node(
            package='robo24_stuff',
            executable='robo24_diynav_node',
            name='robo24_diynav'
        ),

        launch_ros.actions.Node(
            package='robo24_stuff',
            executable='robo24_diyslam_node',
            name='robo24_diyslam'
        ),

        launch_ros.actions.Node(
            package='robo24_stuff',
            executable='robo24_imu_serial_node',
            name='robo24_imu_serial'
        ),

        launch_ros.actions.Node(
            package='robo24_stuff',
            executable='robo24_watch_serial_node',
            name='robo24_watch_serial'
        ),

        # Ros2 system stuff
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
            package='joy',
            executable='joy_node',
            name='joy_xbox'
        ),
  
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='efk_odom',
            parameters=[efk_config]
        )
        
    ])
