#!/usr/bin/env python3

import sys
import serial
import math
import time
import numpy as np

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Point, Quaternion, Vector3

from sensor_msgs.msg import Joy

from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import Pose, PoseStamped

class Robo24NavSimpleNode(Node):

    
    def __init__(self):
        super().__init__('robo24_nav_simple_node')
        nav = BasicNavigator()

        ip = PoseStamped()
        ip.header.frame_id = 'map'
        ip.header.stamp = nav.get_clock().now().to_msg()
        ip.pose.position.x = 3.45
        ip.pose.position.y = 2.15
        ip.pose.orientation.z = 1.0
        ip.pose.orientation.w = 0.0
        nav.setInitialPose(ip)

        # Activate navigation, if not autostarted. This should be called after setInitialPose()
        # or this will initialize at the origin of the map and update the costmap with bogus readings.
        # If autostart, you should `waitUntilNav2Active()` instead.
        nav.lifecycleStartup()
        #nav.waitUntilNav2Active() # if autostarted, else use lifecycleStartup()

        # If desired, you can change or load the map as well
        # navigator.changeMap('/path/to/map.yaml')

        # You may use the navigator to clear or obtain costmaps
        # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
        # global_costmap = navigator.getGlobalCostmap()
        # local_costmap = navigator.getLocalCostmap()

        self.get_logger().info("Robo24 Nav2 Simple Started")


        # Go to our demos first goal pose
        gp = PoseStamped()
        gp.header.frame_id = 'map'
        gp.header.stamp = nav.get_clock().now().to_msg()
        gp.pose.position.x = -2.0
        gp.pose.position.y = -0.5
        gp.pose.orientation.w = 1.0

        # sanity check a valid path exists
        # path = navigator.getPath(initial_pose, gp)

        nav.goToPose(gp)

        i = 0
        while not nav.isTaskComplete():
            ################################################
            #
            # Implement some code here for your application!
            #
            ################################################

            # Do something with the feedback
            i = i + 1
            feedback = nav.getFeedback()
            if feedback and i % 5 == 0:
                print(
                    'Estimated time of arrival: '
                    + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                        / 1e9
                    )
                    + ' seconds.'
                )

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    nav.cancelTask()

                # Some navigation request change to demo preemption
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                    gp.pose.position.x = -3.0
                    nav.goToPose(gp)

        # Do something depending on the return code
        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

        nav.lifecycleShutdown()

        exit(0)


def main(args=None):
    rclpy.init(args=args)

    node = Robo24NavSimpleNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

# This code is needed to run .py file directly
if __name__ == '__main__':
    main()
