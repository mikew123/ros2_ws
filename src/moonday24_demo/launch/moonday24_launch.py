
import launch
import launch_ros.actions

def generate_launch_description():
 
    return launch.LaunchDescription([

        launch_ros.actions.Node(
            package='robo24_stuff',
            executable='sensor_serial_node',
            name='sensor_serial'
        ),

        launch_ros.actions.Node(
            package='moonday24_demo',
            executable='moonday24_pcd_node',
            name='moonday24_pcd'
        )

    ])
