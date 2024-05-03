import launch
import launch_ros
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('robo24_localization'),
        'config',
        'efk_config.yaml'
        )
    
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='efk_odom',
            parameters=[config]
        )

    ])
