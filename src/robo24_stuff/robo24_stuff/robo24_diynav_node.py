#!/usr/bin/env python3

import rclpy
import sys
import serial
import math
import time
import numpy as np
import random

from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_ros import Duration

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped

from sensor_msgs.msg import Joy
from std_msgs.msg import String, Header

from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

#from robo24_interfaces.srv import Claw
from datetime import datetime, timedelta

from rclpy.parameter import Parameter
import os

import json
from copy import copy, deepcopy

#import sensor_msgs.msg as sensor_msgs
#import std_msgs.msg as std_msgs

class Robo24DiynavNode(Node):
    # parameters?
#    name = "robo24_diynav_node"


    pi = 3.14159265

    # waypoint simple 2D pose [x,y,theta]
    gotoWaypoints = 0
    gotoWaypoints_last = 0

    gotoCan = 0
    gotoCan_last = 0

    gotoQtWaypoints = 0
    gotoQtWaypoints_last = 0

    goto4CornerWaypoints = 0
    goto4CornerWaypoints_last = 0

    nav_ctrl = {"mode":"none",  # none, 6-can, 4-corner, Quick-trip, Waypoints
                "arena":"home", # home, dprg
                "state": "done" # init, running, paused, done
                }
    nav_ctrl_last = deepcopy(nav_ctrl)

    XYLatched = False
    calc_waypoint_hz = 20

    state = 0 
    state_last = 0
    wpstate = 0

    navTimerStart = 0

    tf_nanosec_last = 0
    tf_timeout_ns = 1000000000

    wptf_nanosec_last = 0

    canMaxDist = 3.0 # meters when searching for the can
    canMinDist = 0.425 # meters stop when driving towards can

    ft2m:float = 0.3048 # feet per meter

    # 6 can waypoints - home arena
    can6_startWaypoint = [0.0,0.0,0.0] # center 18" from bottom of arena
    can6_goalAlignWaypoint = [5.5*ft2m,0.0,0.0] # in front of goal entrance
    can6_goalEntryWaypoint = [6.5*ft2m,0.0,0.0] # middle of goal entrance
    can6_goalDropWaypoint  = [8.0*ft2m,0.0,0.0] # inside goal area
    can6_leftScanWaypoint  = [6.75/2*ft2m, 1.75*ft2m, 0.0]
    can6_rightScanWaypoint = [6.75/2*ft2m, -1.75*ft2m, 0.0]
    can6_waypoints = [can6_leftScanWaypoint, can6_goalAlignWaypoint, can6_rightScanWaypoint, can6_startWaypoint]

    # 4 corner waypoints - home arena
    cor4_Waypoint0 = [0.0,0.0,0.0] # starting location (location it ends)
    cor4_Waypoint1 = [6*ft2m,0.0,0.0] 
    cor4_Waypoint2 = [6*ft2m,-5.5*ft2m,0.0] 
    cor4_Waypoint3 = [0.0,-5.5*ft2m,0.0] 
    cor4_waypoints = ["cor4_Waypoint1", "cor4_Waypoint2", "cor4_Waypoint3", "cor4_Waypoint0"]

    # quick trip waypoints - home arena
    qt_startWaypoint = [0.0, 0.0, 0.0]
    qt_turnWaypoint = [8.0*ft2m, 0.0, 0.0]
    qt_waypoints = ["qt_turnWaypoint", "qt_startWaypoint"]

    # 1M x 1M square route
    #waypoints = [[1.0, 0.0, 0.0],[1.0,1.0,pi/2],[0.0, 1.0, pi],[0.0,0.0,-pi/2]]
    # CCW rotation 
    #waypoints = [[0.0, 0.0, 0.0],[0.0,0.0,0.75*pi],[0.0,0.0,1.25*pi],[0.0, 0.0, 0.0]]
    # forward straight 5 Meters 
    #waypoints = [[5.0, 0.0, 0.0]]
    # 6 can arena 7x7.5 at home

    # initial waypoint
    waypoint_num = 0

    # TOF 8x8x3 data of interest to pull in the last 400mm before
    # grasping can
 
    tof8CanHor8x2:list[list[int]] = []

    tof8obstacle:list[int] = [-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1]


    scale_rotation_rate = 0.15 #0.1
    scale_forward_speed = 0.3 #0.2

    obstacleOffL = -1
    obstacleOffR = -1 
    obstacleMax = 400 # mm
    obstacleScale = 20.0
    obstacleZoff = 3 # offset from angle 0, 0 when not using TOF for can driving, otherwise maybe 3

    canZcnt = 0
    canZcross = 0
    az_last = 0

    tof8x8x3Ready = False

    navRunMode = "paused"

    def __init__(self):
        super().__init__('robo24_diynav_node')


        self.declare_parameter('6can_arena', "home")
        param_6can_arena = self.get_parameter('6can_arena').value
        self.get_logger().info(f"{param_6can_arena=}")

        self.tf_OK_time_last = self.get_clock().now()
        #self.tf_OK_time_last = rclpy.time.Time()

        self.wpstate3StartTime = self.get_clock().now()
        self.wpstate0StartTime = self.get_clock().now()
        self.findCanStartTime = self.get_clock().now()

        self.joy_subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tof8x8x3_subscription = self.create_subscription(String, 'tof8x8x3_msg', self.tof8x8x3_callback, 10)
        self.robo24_modes_json_subscription = self.create_subscription(String, 'robo24_modes', self.robo24_modes_callback, 10)
        self.robo24_json_subscription = self.create_subscription(String, 'robo24_json', self.robo24_json_callback, 10)

        self.robo24_modes_publisher = self.create_publisher(String, 'robo24_modes',10)
        self.robo24_json_publisher = self.create_publisher(String, 'robo24_json',10)

        # Calc new movement 10 times per second
        self.goto_timer = self.create_timer(1.0/self.calc_waypoint_hz, self.on_goto_timer)
        #self.goto_timer = self.create_timer(1.0/self.calc_waypoint_hz, self.on_goto_timer, callback_group=cb_group)

        self.tf_buffer = Buffer(Duration(seconds=0,nanoseconds=0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Transform frame broadcasters
        self.tf_static_broadcasterOdom = StaticTransformBroadcaster(self)

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.tf_broadcaster = TransformBroadcaster(self)

        # create world, map and odom static frames with no offset
        # map to odom becomes dynamic for transform moving robot relative to odom
        # when nav stack is used map to odom will also be dynamic to correct odom drift
        self.tf_static_broadcasterMap = StaticTransformBroadcaster(self)
        self.make_static_tf(self.tf_static_broadcasterMap,"world", "map", [0.0,0.0,0.0])

        #self.tf_static_broadcasterOdom = StaticTransformBroadcaster(self)
        #self.make_static_tf(self.tf_static_broadcasterOdom, "map", "odom", [0.0,0.0,0.0])

        #This TF is published dynamic in EFK filter node or in wheel_controller
        #self.make_static_tf("odom", "base_link", [0.0,0.0,0.0])

        # set fixed waypoints frames relative to map transform
        for n in range(len(self.can6_waypoints)) :
            wp = "waypoint" + str(n)
            self.tf_static_broadcaster = StaticTransformBroadcaster(self)
            self.make_static_tf(self.tf_static_broadcaster,"map", wp, self.can6_waypoints[n])
            #time.sleep(1.0)

        # Goal waypoint to bring CAN to after grabbing it
        self.tf_static_broadcasterGoalDrop = StaticTransformBroadcaster(self)
        self.make_static_tf(self.tf_static_broadcasterGoalDrop, "map", "goalDropWaypoint", self.can6_goalDropWaypoint)

        # Goal waypoint to bring CAN to after grabbing it
        self.tf_static_broadcasterGoalEntry = StaticTransformBroadcaster(self)
        self.make_static_tf(self.tf_static_broadcasterGoalEntry, "map", "goalEntryWaypoint", self.can6_goalEntryWaypoint)

        # Waypoint to exit goal area after dropping off CAN
        self.tf_static_broadcasterGoalAlign = StaticTransformBroadcaster(self)
        self.make_static_tf(self.tf_static_broadcasterGoalAlign, "map", "goalAlignWaypoint", self.can6_goalAlignWaypoint)

        self.tf_static_broadcasterQtStartWp = StaticTransformBroadcaster(self)
        self.make_static_tf(self.tf_static_broadcasterQtStartWp, "map", "qt_startWaypoint", self.qt_startWaypoint)

        self.tf_static_broadcasterQtTurnWp = StaticTransformBroadcaster(self)
        self.make_static_tf(self.tf_static_broadcasterQtTurnWp, "map", "qt_turnWaypoint", self.qt_turnWaypoint)

        self.tf_static_broadcaster4corWp0 = StaticTransformBroadcaster(self)
        self.make_static_tf(self.tf_static_broadcaster4corWp0, "map", "cor4_Waypoint0", self.cor4_Waypoint0)

        self.tf_static_broadcaster4corWp1 = StaticTransformBroadcaster(self)
        self.make_static_tf(self.tf_static_broadcaster4corWp1, "map", "cor4_Waypoint1", self.cor4_Waypoint1)

        self.tf_static_broadcaster4corWp2 = StaticTransformBroadcaster(self)
        self.make_static_tf(self.tf_static_broadcaster4corWp2, "map", "cor4_Waypoint2", self.cor4_Waypoint2)

        self.tf_static_broadcaster4corWp3 = StaticTransformBroadcaster(self)
        self.make_static_tf(self.tf_static_broadcaster4corWp3, "map", "cor4_Waypoint3", self.cor4_Waypoint3)

        self.navTimerStart = time.time()

        self.get_logger().info("Robo24 DIY Navigation Started")
    #end init

    def process_nav_cmd(self, cmd) :
        msg_str = None
        if "state" in cmd :
            state = cmd["state"]
            if state=="toggle" :
                if self.navRunMode=="running" :
                    self.navRunMode = "paused"
                if self.navRunMode=="init" :
                    self.navRunMode = "running"
                if self.navRunMode=="paused" :
                    self.navRunMode = "running"
                    
                msg_json = {"nav_stat": {"state": self.navRunMode}}
                msg_str = json.dumps(msg_json)
                    
        if msg_str != None :
            self.robo24_modes_data_publish(msg_str)

    def robo24_json_callback(self, msg:String) -> None :
        self.get_logger().info(f"diynav robo24_json_callback {msg=}")
        try :
            packet = json.loads(data)
        except Exception as ex:
            self.get_logger().error(f"watch serial watch_json_callback exception {ex}")
            return

        if "nav_cmd" in packet :
            nav_cmd = packet["nav_cmd"]
            process_nav_cmd(nav_cmd)

    def robo24_modes_callback(self, msg:String) ->None :
        self.get_logger().info(f"diynav robo24_modes_callback {msg=}")

        data = msg.data
        packet = None

        try :
            packet = json.loads(data)

        except Exception as ex:
            self.get_logger().error(f"watch serial watch_json_callback exception {ex}")

        msg_str:str = None
        # send nav run state to nav publish
        if packet!=None :
            if "run_state" in packet :
                run_state = packet["run_state"]
                if run_state == "toggle" :
                    pass
                if self.navRunMode == "running" :
                    self.navRunMode = "paused"
                    msg_str = "{\"state\": \"paused\"}"
                if self.navRunMode == "paused" :
                    self.navRunMode = "running"
                    msg_str = "{\"state\": \"running\"}"

        if msg_str != None :
            self.robo24_modes_data_publish(msg_str)


        ############# Detect obstacle using TOF8x8x3 sensors ###############
    def calcObstacleAvoidance(self, obstacleZoff:int) -> None :
        """
            calcs self.obstacleOffL and R
        """
        
        # select first obstical scanning middle to side
        #g = 75.0/67 # 0 to 75 deg has cos range of 1 to 1/4
        g = 67.0/67 # 0 to 67 deg has cos range of 1 to 1/3
        o = 0.2 # max obstacle angle velocity to try to avoid it

        # detect obstacle on left side
        self.obstacleOffL = 0.0
        iL=-1
        for n in range(obstacleZoff, 12):
            r = math.radians((45.0/8) * (n+0.5) * g)
            k = math.cos(r)
            i = 11-n
            if self.tof8obstacle[i]>=0 and self.tof8obstacle[i]<=k*self.obstacleMax :
                self.obstacleOffL = k*o
                iL=i
                break
        
        # detect obstacle on right side
        self.obstacleOffR = 0.0
        iR=-1
        for n in range(obstacleZoff, 12):
            r = math.radians((45.0/8) * (n+0.5) * g)
            k = math.cos(r)
            i = 12+n
            if self.tof8obstacle[i]>=0 and self.tof8obstacle[i]<=k*self.obstacleMax:
                self.obstacleOffR = k*o
                iR=i
                break

        if(iL!=-1 or iR!=-1) :
            self.get_logger().info(f"{self.tof8obstacle = } [{iL}]{self.obstacleOffL = } [{iR}]{self.obstacleOffR = }")

    def robo24_modes_data_publish(self, data:str) -> None :
        msg = String()
        msg.data = data
        self.robo24_modes_publisher.publish(msg)

    def robo24_json_data_publish(self, data:str) -> None :
        msg = String()
        msg.data = data
        self.robo24_json_publisher.publish(msg)

    def diy_slam_enable(self, slam_enable:bool=True) ->None:
        """
            Enable or disable diy slam 
        """
        if slam_enable :
            self.robo24_modes_data_publish("SLAM ON")
        else :
            self.robo24_modes_data_publish("SLAM OFF")


    def on_goto_timer(self) :

        waypoint_distance = 0
        waypoint_theta = 0
        theta_err = 0
        state:int = self.state                           
        retVal:int = 0

        # flag that transform failed - used to find can
        tf_OK = False
        TF_Timeout = False

        #now = self.get_clock().now()
        now = rclpy.time.Time() # Gets time=0 (I think simulation time)
        
        # # Create waypoint id string
        # if self.gotoWaypoints == 1: # Y button on controller
        #     waypoint = "waypoint" + str(self.waypoint_num)
        # elif self.gotoCan == 1: # X button on controller
        #     waypoint = "can"
        # elif self.gotoQtWaypoints == 1: # B button 1 on controller
        #     waypoint = self.qt_waypoints[self.waypoint_num]
        # elif self.goto4CornerWaypoints == 1: # A button 0 on controller
        #     waypoint = self.cor4_waypoints[self.waypoint_num]

        nav_ctrl_mode = self.nav_ctrl["mode"]
        nav_ctrl_mode_last = self.nav_ctrl_last["mode"]

        if   nav_ctrl_mode == "Waypoints" :  waypoint = "waypoint" + str(self.waypoint_num)
        elif nav_ctrl_mode == "6-can" :      waypoint = "can"
        elif nav_ctrl_mode == "Quick-trip" : waypoint = self.qt_waypoints[self.waypoint_num]
        elif nav_ctrl_mode == "4-corner" :   waypoint = self.cor4_waypoints[self.waypoint_num]

        # disable diy slam - takes a few seconds
        if nav_ctrl_mode!="none" and nav_ctrl_mode!=nav_ctrl_mode_last :
            self.robo24_modes_data_publish(f"Started {nav_ctrl_mode=}")
            if nav_ctrl_mode=="6-can" or nav_ctrl_mode=="none" : self.diy_slam_enable(True)
            else : self.diy_slam_enable(False)

        # # disable diy slam - takes a few seconds
        # if (self.gotoQtWaypoints==1 and self.gotoQtWaypoints_last==0) :
        #     self.diy_slam_enable(False)
        #     self.robo24_modes_data_publish("mode QTrip started")

        # if (self.goto4CornerWaypoints==1 and self.goto4CornerWaypoints_last==0) :
        #     self.diy_slam_enable(False)
        #     self.robo24_modes_data_publish("mode 4Corner started")
            
        # if (self.gotoCan==1 and self.gotoCan_last==0) :
        #     self.diy_slam_enable(True)
        #     self.robo24_modes_data_publish("mode 6Can started")

        # if (self.gotoWaypoints==1 and self.gotoWaypoints_last==0) :
        #     self.diy_slam_enable(True)
        #     self.robo24_modes_data_publish("mode Waypoints started")

        # autonomous drive robot to waypoint or can
        # create /cmd_vel message 
        msg = Twist()
        
        # if     self.gotoWaypoints==1     \
        #     or self.gotoCan==1           \
        #     or self.gotoQtWaypoints==1   \
        #     or self.goto4CornerWaypoints==1 :
        if nav_ctrl_mode!=None :

            ############## GOTO WAYPOINTS STATES ############
            # if self.gotoWaypoints==1 :
            if nav_ctrl_mode=="Waypoints" :
                retVal = self.gotoWaypointStates(now, waypoint, msg)
                if retVal != 0 : 
                    # start goto next Waypoint 
                    self.waypoint_num += 1
                    if self.waypoint_num >= len(self.can6_waypoints) :
                        self.waypoint_num = 0
                    self.get_logger().info(f'Arrived at {waypoint}, next num = {self.waypoint_num} '.encode())
                
            ############### GOTO Quick Trip WAYPOINTS #################
            # elif self.gotoQtWaypoints==1:
            elif nav_ctrl_mode=="Quick-trip" :
                retVal = self.gotoWaypointStates(now, waypoint, msg)
                if retVal != 0 :
                    # start goto next Waypoint - only 2 waypoints 0,1 stop at #1
                    if self.waypoint_num<1 :
                        self.waypoint_num += 1
                    self.get_logger().info(f'Arrived at QT {waypoint}, next num = {self.waypoint_num} '.encode())
                
            ############### GOTO 4 Corner WAYPOINTS #################
            #elif self.goto4CornerWaypoints==1:
            elif nav_ctrl_mode=="4-corner" :

                match self.state :

                  case 0:
                    # goto waypoint
                    retVal = self.gotoWaypointStates(now, waypoint, msg)
                    if retVal != 0 :
                        # start goto next Waypoint - 4 waypoints 0,1,2,3 stop at #3 and rotate to begin angle
                        if self.waypoint_num<3 :
                            self.waypoint_num += 1
                        else :
                            self.state = 1
                        self.get_logger().info(f'Arrived at 4 CORNER {waypoint}, next num = {self.waypoint_num} '.encode())
                        self.cor4Angle = -(math.pi/2)
                        self.cor4AngleTime = 0.0

                  case 1 :
                    # rotate to starting angle pose
                    dt = 1.0/self.calc_waypoint_hz # time interval sec
                    av = 0.5 #rad/sec
                    dr = av*dt # rad/interval
                    self.cor4Angle += dr #new angle
                    msg.angular.z = -av / (2*math.pi) # NOTE: units are wrong!!!!
                    if self.cor4Angle >= 0.0 :
                        self.state = 2

                  case 2 :
                    # end
                    msg.angular.z = 0.0


            ############### GOTO CAN STATES ###################
            # elif self.gotoCan==1:
            elif nav_ctrl_mode=="6-can" :

                # process TOF from L->R first sequential valid distances used
                # 3 states 0=no valid yet, 1=valid current, 2=past valid
                hvalid:int = int(self.canMinDist*1.5*1000)
                hstate:int = 0
                hdist:int = 0
                hrot:int = 0
                l:int = 0
                r:int = 0
                ln:int = 0
                rn:int = 0
                
                
                # process center 8 sensors in rows 2,3 from bottom
                hmin = 1000 # arbitrary large
                for i in range(0,8) :
                    h2 = self.tof8CanHor8x2[i]
                    hd=0
                    hn=0
                    # process data from each row as 1 value average
                    if h2[0]>0 and h2[0]<hvalid :
                        hd+=h2[0]
                        hn+=1
                    if h2[1]>0 and h2[1]<hvalid :
                        hd+=h2[1]
                        hn+=1
                    if hn>0 :
                        hd = hd/hn
                    match hstate :
                        case 0: # find 1st valid distance (pair)
                            if hd>0 and hd<hvalid :
                                if hd<hmin : 
                                    hmin = hd
                                if i<=3 : 
                                    l+=hd
                                    ln+=1
                                else : 
                                    r+=hd
                                    rn+=1
                            elif (ln+rn)>0 : # invalid dist after detecting valids
                                hstate = 1
                        # end state 0
                        case 1 : # stop processsing current set of detects, resume if new min
                            if hd>0 and hd<hvalid :
                                if hd<hmin : # new object start
                                    hmin = hd
                                    # reset vars for new obj dist min detect
                                    hstate = 0
                                    l=0
                                    ln=0
                                    r=0
                                    rn=0
                                    if i<=3 : 
                                        l+=hd
                                        ln+=1
                                    else : 
                                        r+=hd
                                        rn+=1
                        # end state 1
                    # end states
                                            
                ld=0
                rd=0
                if ln > 0 :
                    ld += l/ln
                if rn > 0 :
                    rd += r/rn
                
                hdist = int(ld + rd)
                if ld>0 and rd>0 :
                    hdist = int(hdist/2)

                hrot = -int(ld - rd)

                # self.get_logger().info(f"{hvalid = } {ld = } {ln = } {rd = } {rn = } {hdist = } {hrot = } {self.tof8CanHor8x2 = }")


                # determine if there is a close object using 6 middle horiz TOF
                # The outer sensors are not reliable and have a lot of false positive
                closeCanDet = False
                for i in range (9, 15) : pass
                #                    if self.tof8CanHor[i-8]>0 and self.tof8CanHor[i-8]<500 :
                #                        closeCanDet = True

                ##############################################


                # reset can scan time out when started
                # if self.gotoCan_last!=self.gotoCan :
                if nav_ctrl_mode!=nav_ctrl_mode_last :
                    self.wpstate0StartTime = self.get_clock().now()
                    self.findCanStartTime = self.get_clock().now()

                ########## CAN STATES ##########
                if (self.get_clock().now() - self.wpstate0StartTime) >= rclpy.time.Duration(seconds=45.0) :
                    self.findCanStartTime = self.get_clock().now()
                    # timeout finding can  goto a new waypoint
                    l:int = len(self.can6_waypoints)
                    n:int = random.randint(0,l-1)
                    self.newWaypoint = "waypoint" + str(n)
                    self.state = 1
                    self.get_logger().info(f'Timeout finding can goto new {self.newWaypoint=}')
            
                match self.state:
                  case 0:
                    state = self.state
                    retVal = self.gotoWaypointStates(now, "can", msg, 0, True)
                    if retVal != 0 :
                        if retVal == 100 : 
                            # timeout scanning  goto a new waypoint
                            l:int = len(self.can6_waypoints)
                            n:int = random.randint(0,l-1)
                            self.newWaypoint = "waypoint" + str(n)
                            self.state = 1
                            self.get_logger().info(f'[{state}->{self.state}, {waypoint} {self.newWaypoint=}]')
                        else :
                            # self.wpstate = 0
                            self.state = 3
                            self.get_logger().info(f'[{state}->{self.state}, {waypoint}]')

                  case 1:
                    state = self.state
                    retVal = self.gotoWaypointStates(now, self.newWaypoint, msg, 0, True)
                    if retVal != 0 :
                        self.state = 0
                        self.findCanStartTime = self.get_clock().now()
                        self.get_logger().info(f'[{state}->{self.state}, {waypoint}]')

                # removed state 2
                
                # Use TOF horizontal sensors to center can
                # timer rate can be faster than TOF data is ready
                  case 3:
                    state = self.state
                    if hdist==0 :
                        # TOF sensors have no data
                        self.state = 0
                    else :
                        if hdist>600 : 
                            self.state = 0 # too far, start searching again
                        else :
                            if hrot > 100 :
                                msg.angular.z = -0.05
                            elif hrot > 0 :
                                msg.angular.z = -0.01
                            elif hrot < -100 :
                                msg.angular.z = +0.05
                            elif hrot < 0 :
                                msg.angular.z = +0.01
                            
                            if msg.angular.z==0 : self.canZcnt += 1
                            else : self.canZcnt = 0

                            if self.az_last > 0 and msg.angular.z < 0 : self.canZcross += 1
                            if self.az_last < 0 and msg.angular.z > 0 : self.canZcross += 1
                            self.az_last = msg.angular.z

                            # after crossing zero multiple times or staying on zero for a while go to can
                            if self.canZcross > 2 or self.canZcnt > 6 :
                                self.state = 4
                                self.get_logger().info(f'[{state}->{self.state}, {waypoint}]')

                    # self.get_logger().info(f'[{state}->{self.state}, {waypoint}] {hdist = } {hrot = } Zct{self.canZcnt} Zcr{self.canZcross} fv{msg.linear.x} av{msg.angular.z}')
                                           
                # continue to can using TOF 8x8x3 Center sensors to go to within a few mm
                  case 4:
                    state = self.state
                    if hdist==0 :
                        # TOF sensors have no data
                        self.state = 0
                    else :
                        if hdist>600 : 
                            self.state = 0 # too far, start searching again
                        elif hdist<=130 : 
                            self.state = 5 # Can is close enough to grab
                        else :
                            # foward and rotational speed is variable a bit
                            if hrot > 100 :
                                msg.angular.z = -0.05
                            elif hrot > 0 :
                                msg.angular.z = -0.01
                            if hrot < -100 :
                                msg.angular.z = +0.05
                            elif hrot < 0 :
                                msg.angular.z = +0.01

                            if hdist > 200 :
                                msg.linear.x = 0.1
                            elif hdist > 130 :
                                msg.linear.x = 0.05
                            else : # ???? blocked logic ???
                                self.state = 4
                                # stop all movement
                                msg.angular.z = 0.0
                                msg.linear.x = 0.0
                                self.get_logger().info(f'[{state}->{self.state}, {waypoint}]')

                    # self.get_logger().info(f'[{state}->{self.state}, {waypoint}] d{hdist} r{hrot} fv{msg.linear.x} av{msg.angular.z}')

                # grab can
                  case 5:
                    state = self.state
                    resp = "bad hdist"
                    grabMax = self.obstacleMax*1.5 #600
                    # NOTE: timer rate can be faster than TOF data is ready
                    if hdist<=0 or hdist>grabMax : 
                        self.state = 0
                    else :
                        resp = self.clawCmd(100, 1000)
                        self.state = 6
                        self.findCanStartTime = self.get_clock().now()
                        self.get_logger().info(f'[{state}->{self.state}, {waypoint}]')

                    # self.get_logger().info(f'[{state}->{self.state}, {waypoint}], {hdist = } {resp = }')
                
                # Bring can to Goal Entrance Waypoint before entering
                # This aligns the robot to the entrance before entering
                  case 6:
                    state = self.state
                    retVal = self.gotoWaypointStates(now, "goalAlignWaypoint", msg, 4, True)
                    if retVal != 0 :
                        # self.wpstate = 0
                        self.state = 7
                        self.get_logger().info(f'[{state}->{self.state}, {waypoint}]')

                # Bring can to Goal Drop off location
                  case 7:
                    state = self.state
                    retVal = self.gotoWaypointStates(now, "goalDropWaypoint", msg, 0, False)
                    if retVal != 0 :
                        # self.wpstate = 0
                        self.state = 8
                        self.get_logger().info(f'[{state}->{self.state}, {waypoint}]')

                # Drop off CAN
                  case 8:
                    state = self.state
                    resp = self.clawCmd(0, 1000)
                    self.state = 9
                    self.get_logger().info(f'[{state}->{self.state}, {waypoint}]')

                # Backup 10cm from can dropped off in the Goal area
                  case 9:
                    state = self.state
                    if self.state_last != self.state :
                        self.navTimerStart = time.time()

                    if (time.time() - self.navTimerStart) <  1.0 :
                        msg.linear.x = -0.1 # reverse at 10 cm/sec for 1 sec
                    else :
                        msg.linear.x = 0.0 # stop
                        self.state = 10
                        self.get_logger().info(f'[{state}->{self.state}, {waypoint}]')

                 # Complete exiting Goal area, then start search for another CAN
                  case 10:
                    state = self.state
                    self.findCanStartTime = self.get_clock().now()
                    retVal = self.gotoWaypointStates(now, "goalAlignWaypoint", msg, 0, False)
                    #self.gotoWaypointStates("goalAlignWaypoint", msg, False, False)
                    if retVal != 0 :
                        # self.wpstate = 0
                        self.state = 0
                        self.get_logger().info(f'[{state}->{self.state}, {waypoint}]')

            ######## END STATES ############

            #msg.angular.z = 0.0
            #msg.linear.x = 0.0
            self.cmd_vel_publisher.publish(msg)

        # if     self.gotoWaypoints_last        != self.gotoWaypoints       \
        #     or self.gotoCan_last              != self.gotoCan             \
        #     or self.gotoQtWaypoints_last      != self.gotoQtWaypoints     \
        #     or self.goto4CornerWaypoints_last != self.goto4CornerWaypoints:
        if nav_ctrl_mode!=nav_ctrl_mode_last :
            # stop motion when button released
            # single twist /cmd_vel published so that joy can be used
            # without the fancy twist mux node
            msg = Twist()
            self.cmd_vel_publisher.publish(msg)


        self.nav_ctrl_last = deepcopy(self.nav_ctrl)

        self.gotoWaypoints_last = self.gotoWaypoints
        self.gotoCan_last = self.gotoCan
        self.gotoQtWaypoints_last = self.gotoQtWaypoints
        self.goto4CornerWaypoints_last = self.goto4CornerWaypoints

        self.state_last = state

    # send a message to claw to open/close
    #TODO: use custom msg (2 ints) instead of string
    def clawCmd(self, pct: int, msec: int) -> None:
        """
        sends (publish) a message to claw to open/close
        pct is percent claw closed (0 = 100%o pen)
        msec is how long the claw moves to the new position
        """
        cmd_json = {"claw": {"open": pct, "time": 1000}}
        cmd_str = json.dumps(cmd_json)+"\0"
        self.robo24_json_data_publish(cmd_str)

        # blocking wait for the expected claw movement time
        # blocking is OK since the robot should be stopped
        time.sleep(msec/1000.0)
    


    # go to a waypoint with TOF can options
    # retrns 0:Running 1:Completed 10:Undefined state 100:Timeout scanning for cans
    def gotoWaypointStates(self, now:time, waypoint:str, msg:Twist, obsOff:int = 0, obsEna:bool=False,
                          canMode:bool=False, closeCanDet:bool=False) -> int:
        """
        go to a waypoint with TOF can options
        """
        retVal:int = 0 #Running

        waypoint_distance:float = 0.0
        waypoint_theta:float = 0.0
        theta_err:float = 0.0
                                      
        # flags that transform failed 
        tf_OK = False
        TF_Timeout = False
        tf_Tdiff = False
        
        tf_lookupTimeout = rclpy.duration.Duration(seconds=0.0)
        
        # get waypoint offsets relative to robot
        try:
            t1 = self.tf_buffer.lookup_transform(
                waypoint,
                'base_link',
                now,
                timeout=tf_lookupTimeout
                )
            
            # Get angle of waypoint relative to base_link
            e = euler_from_quaternion(
                t1.transform.rotation.x,
                t1.transform.rotation.y,
                t1.transform.rotation.z,
                t1.transform.rotation.w)
            robot_wp_angle = -e[2] #yaw

            #robot_wp_angle-=(math.pi/2)
                    
            # XY from robot to waypoint ie distance from waypoint to source is negative
            # negate xy so distance is positive and gets smaller positive
            robot_wp_x = -t1.transform.translation.x
            robot_wp_y = -t1.transform.translation.y

            tf_nanosec = t1.header.stamp.nanosec
            if tf_nanosec != self.wptf_nanosec_last : 
                tf_Tdiff = True
            else : 
                tf_Tdiff = False
                self.get_logger().info(f'{tf_Tdiff=} {tf_nanosec=} {self.wptf_nanosec_last=}')

            tf_OK = tf_Tdiff

            #self.get_logger().info(f'{robot_wp_angle = } {robot_wp_x = } {robot_wp_y = } {tf_OK = } {tf_nanosec = } {self.wptf_nanosec_last = } {t1 = }'.encode())
            
            self.wptf_nanosec_last = tf_nanosec
            
        except (LookupException, ConnectivityException, ExtrapolationException) as ex:
            self.get_logger().info(f'Could not transform base_link to {waypoint} - will try again: {ex}')
            tf_OK = False
            #return
                
        # Get XY for can relative to map, compare to goal entrance location to disqualify can in goal
        if tf_OK==True and waypoint=="can" :
            # if self.gotoCan == 1:
            if self.nav_ctrl["mode"]=="6-can" :
                try:
                    t2 = self.tf_buffer.lookup_transform (
                        'map',
                        'base_link',
                        now,
                        timeout=tf_lookupTimeout
                        )
                    # Get angle of can relative to base_link
                    e = euler_from_quaternion(
                        t2.transform.rotation.x,
                        t2.transform.rotation.y,
                        t2.transform.rotation.z,
                        t2.transform.rotation.w)
                    map_robot_angle = -e[2] #yaw
                            
                    # self.get_logger().info(f'{map_robot_angle = } {t2 = }'.encode())


                    # changed direction to goal!
                    goalEntranceX = (self.can6_goalDropWaypoint[0] + self.can6_goalAlignWaypoint[0])/2.0
                    robotCanDist = math.sqrt(t1.transform.translation.x**2 + t1.transform.translation.y**2)

                    robotCanX = robotCanDist * math.cos(map_robot_angle)

                    mapRobotX = t2.transform.translation.x
                    mapCanX = robotCanX + mapRobotX
                    # self.get_logger().info(f'{robotCanDist = } {mapRobotX = } {robotCanX = } {mapCanX = } {goalEntranceX = }'.encode())

                    if mapCanX > goalEntranceX :
                        # force TF invalid
                        tf_OK = False
                        pass
                    
                except (LookupException, ConnectivityException, ExtrapolationException) as ex:
                    self.get_logger().info(f'Could not transform map to base_link - will try again: {ex}')
                    #tf_OK = False


        if tf_OK==True :
            #self.get_logger().info(f'transform robo24 to {waypoint} {tf_nanosec}')
            x=robot_wp_x
            y=robot_wp_y
            waypoint_distance = math.sqrt((x*x) + (y*y))
            waypoint_theta = -math.atan2(y,x)

            theta_err =  robot_wp_angle - waypoint_theta
            # minimize rotation
            if theta_err > math.pi :
                theta_err -= 2*math.pi
            if theta_err < -math.pi :
                theta_err += 2*math.pi

        #if tf_Tdiff==True :
        if tf_OK==True :
            self.tf_OK_time_last = self.get_clock().now()

        else :
            # timeout to restart can search
            timeSinceLastOK = self.get_clock().now() - self.tf_OK_time_last
            if timeSinceLastOK.nanoseconds > self.tf_timeout_ns :
                if TF_Timeout == False :
                    self.get_logger().info(f"WP>>>>>>>>>>  TF Timeout  <<<<<<<<<<<")
                TF_Timeout = True


        # Process state transition to 0 with state transition flag 4 or 5
        if self.wpstate == 4 :
            self.wpstate = 0
            self.wpstate0StartTime = self.get_clock().now()

        if self.wpstate == 5 :
            self.wpstate = 0
            
        # Waypoint states
        match self.wpstate:

          case 0:
            state = self.wpstate
            # TODO: need to figure out how to detect a close can w/o TF
            # and go to it

            if waypoint!="can":
                # Not in can mode so wait for TF detection
                if tf_OK==True :
                    self.wpstate = 1
                    self.get_logger().info(f'WP[{state}->{self.wpstate}, {waypoint}] {tf_OK=}')
            else :
                # in can mode detect when can TF is detected and close enough
                # or if TOF sensors detect the can close
                if tf_OK==True :
                    if waypoint_distance<self.canMaxDist: 
                        self.wpstate = 1 
                        self.get_logger().info(f'WP[{state}->{self.wpstate}, {waypoint}]  {tf_OK=} {waypoint_distance=}')
                        
                    else :
                        # locate can by rotating body until can is detected
                        # randomize the direction to reduce accumulated offsets
                        rand = 1 - 2*random.randint(0,1) # returns 1 or -1
                        msg.angular.z = rand * self.scale_rotation_rate

                ##### ignore close detect while scanning untill it can ignore walls
                #elif closeCanDet==True:
                #        self.wpstate = 3
                        
                else :
                    # scan for a can, timeout if none found
                    msg.angular.z = self.scale_rotation_rate

            if (self.get_clock().now() - self.wpstate0StartTime) >= rclpy.time.Duration(seconds=15.0) :
                self.wpstate = 4
                retVal = 100 # timeout looking for can
                self.get_logger().info(f'Scan timeout WP[{state}->{self.wpstate}, {waypoint} {retVal = }]')

            # self.get_logger().info(f'WP[{state}->{self.wpstate}, {waypoint}] d{waypoint_distance} {tf_OK = } {closeCanDet = } fv{msg.linear.x} av{msg.angular.z}')

        # rotate to point toward waypoint/can
          case 1:
            state = self.wpstate
            if TF_Timeout==True : 
                self.wpstate = 5
                self.get_logger().info(f'WP[{state}->{self.wpstate}, {waypoint} {TF_Timeout = }]{tf_OK = }')
            elif tf_OK == True: 
                # point to waypoint/can
                minD = 0.05
                if waypoint=="can" : minD = self.canMinDist
                if waypoint_distance>minD and abs(theta_err) > 0.05 :
                    # limit low end of theta err                
                    if theta_err>0 : err=theta_err+0.2
                    else :           err=theta_err-0.2
                    msg.angular.z = self.scale_rotation_rate * err
                else :
                    self.wpstate = 2
                    self.get_logger().info(f'WP[{state}->{self.wpstate}, {waypoint} {TF_Timeout = } {waypoint_distance = } {theta_err = } {tf_OK = }]')
                    
            else :
                #self.wpstate = 5
                self.get_logger().info(f'WP[{state}->{self.wpstate}, {waypoint}]{TF_Timeout = } {tf_OK = }')

            # self.get_logger().info(f'WP[{state}->{self.wpstate}, {waypoint}] d{waypoint_distance} a{theta_err} {TF_Timeout = } {tf_OK = } fv{msg.linear.x} av{msg.angular.z}')

        # drive to waypoint/can, stop at limit of blob distance detection capability
          case 2:
            state = self.wpstate
            if TF_Timeout==True : 
                self.wpstate = 4
                self.get_logger().info(f'WP[{state}->{self.wpstate}, {waypoint}]{TF_Timeout = } {tf_OK = }')

            elif tf_OK == True: 
                minD = 0.05 # distance to stop before waypoint
                #obsOff = 0 # obstacle avoidance sensor center offset about 8 deg each
                if waypoint=="can": minD = self.canMinDist
                #if canMode==True : obsOff = self.obstacleZoff
                    
                if waypoint_distance>minD :
                    # correct for angular offset as it drives forward
                    msg.angular.z = 8*self.scale_rotation_rate * theta_err #/self.pi

                    # avoid obstacles
                    if obsEna : self.calcObstacleAvoidance(obsOff)
                    else : self.obstacleOffR = self.obstacleOffL = 0.0

                    msg.angular.z += self.obstacleOffR - self.obstacleOffL

                    #start full max speed then slow down, HW deceleration is 1M/sec
                    #if waypoint_distance > (0.66 * self.scale_forward_speed) : 
                    if (self.obstacleOffR + self.obstacleOffL) == 0 : fwdSpeed = self.scale_forward_speed
                    else : fwdSpeed = 0.5 * self.scale_forward_speed
                    if waypoint_distance > 1.0 : 
                        msg.linear.x = fwdSpeed
                    else : 
                        msg.linear.x = (waypoint_distance+0.2) * fwdSpeed
                else :
                    self.wpstate3StartTime = self.get_clock().now()
                    self.wpstate = 3
                    self.get_logger().info(f'WP[{state}->{self.wpstate}, {waypoint}] {waypoint_distance=}')

            # pause with no movement to allow diyslam to make correction
          case 3:
            state = self.wpstate
            # wait for time
            if (self.get_clock().now() - self.wpstate3StartTime) >= rclpy.time.Duration(seconds=3.0) :
                self.wpstate = 4
                retVal = 1 # finished
                self.get_logger().info(f'WP[{state}->{self.wpstate}, {waypoint} {retVal = }]')

            # self.get_logger().info(f'WP[{state}->{self.wpstate}, {waypoint}] {self.obstacleOffL = } {TF_Timeout = } {tf_OK = } d{waypoint_distance} a{theta_err} fv{msg.linear.x} av{msg.angular.z}')
    
          case _:
            self.wpstate = 4
            retVal = 10 # undefined state
            self.get_logger().info(f'WP[{state}->{self.wpstate}, {waypoint} {retVal = }]')
        
        return retVal

    #END def gotoWayPoints()

    # get TOF8x8x3 sensors [[[12,5],[13,5]], [[12,6],[13,6]], [[12,7],[13,7]]]
    def tof8x8x3_callback(self, msg: String) -> None:

        try:
            #split msg string into 193 seperate strings 1 for each element
            tofStrArray = msg.data.split(" ")

            # parse messsage of "name" then 192 integers (8 rows of 24 distances)
            if tofStrArray[0]!="TOF8x8x3" or len(tofStrArray)!=193:
                self.get_logger().error(f"TOF8x8x3 type message error: {msg.data}")
                return
    
            # select center 8 sensor in row 2,3 from bottom of 8x8x3 sensor array
            self.tof8CanHor8x2 = []
            for i in range(0,8) :
                h2:list = []
                for j in range(0,2) :
                    ########## dont understand why (9+i)gets center instaed of (8+i)
                    h2.append(int(tofStrArray[(9+i)+((5+j)*24)])) # from rows 5 or 6
                self.tof8CanHor8x2.append(h2)

            # select sensors on 3rd row from bottom (5) for obstacle avoidance
            for i in range(0,24):
                self.tof8obstacle[i] = int(tofStrArray[i+(5*24)+1]) #[i,5]

            self.tof8x8x3Ready = True
        except:
            self.get_logger().error(f"TOF8x8x3 parse message error: {msg.data}")
            return


        # self.get_logger().info(f"TOF8x8x3 Horiz  data of interest = {self.tof8CanHor}")
        # self.get_logger().info(f"TOF8x8x3 Center data of interest = {self.tof8CanCtr}")
        # self.get_logger().info(f"TOF8x8x3 Horiz data of interest = {self.tof8CanHor8x2}")


    # This should be replaced with an action command from teleop_robo24
    def joy_callback(self, msg: Joy) -> None:
        if self.XYLatched==False :
            self.gotoWaypoints        = msg.buttons[3] # 1 = Y button pushed
            self.gotoCan              = msg.buttons[2] # 1 = X button pushed
            self.gotoQtWaypoints      = msg.buttons[1] # 1 = B button pushed
            self.goto4CornerWaypoints = msg.buttons[0] # 1 = A button pushed

            if   msg.buttons[3] : self.nav_ctrl["mode"] = "Waypoints"
            elif msg.buttons[2] : self.nav_ctrl["mode"] = "6-can"
            elif msg.buttons[1] : self.nav_ctrl["mode"] = "Quick-trip"
            elif msg.buttons[0] : self.nav_ctrl["mode"] = "4-corner"
            else : self.nav_ctrl["mode"] = "none"

        resetAxes = msg.buttons[6] # 1 = select button pushed
        latchButton = msg.buttons[5] # 1 = select button pushed

        if  self.XYLatched==False :
            if (   msg.buttons[2]==1 or  msg.buttons[3]==1  \
                or msg.buttons[0]==1 or  msg.buttons[1]==1) \
                and latchButton==1 : 
                self.XYLatched = True
        else :
            if (    msg.buttons[2]==0 and msg.buttons[3]==0  \
                and msg.buttons[0]==0 and msg.buttons[1]==0) \
                and latchButton==1 :
                self.XYLatched = False

        if resetAxes :
            self.state = 0
            self.waypoint_num = 0
            self.clawCmd(0, 100) #open claw

    # xyt [x,y,theta]
    def make_static_tf(self, tf_static_broadcaster, 
                       parent: str, child: str, xyt: list) -> None:
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child

        t.transform.translation.x = xyt[0]
        t.transform.translation.y = xyt[1]
        t.transform.translation.z = 0.0
        quat = quaternion_from_euler(0.0, 0.0, xyt[2]) #x,y,theta
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        #self.tf_static_broadcaster.sendTransform(t)
        tf_static_broadcaster.sendTransform(t)


    def broadcast_tf(self, parent: str, child: str, xyt: list ) -> None:
        # Create and broadcast the transform message 
        tfs = TransformStamped()
        tfs.header.stamp = self.get_clock().now().to_msg()
        tfs.header.frame_id = parent
        tfs._child_frame_id = child
        tfs.transform.translation.x = xyt[0]
        tfs.transform.translation.y = xyt[1]
        tfs.transform.translation.z = 0.0 #theta # for debug should be 0.0  

        q = quaternion_from_euler(0.0, 0.0, xyt[2]) #x,y,theta

        tfs.transform.rotation.x = q[0]
        tfs.transform.rotation.y = q[1]
        tfs.transform.rotation.z = q[2]
        tfs.transform.rotation.w = q[3]

#        self.get_logger().info(f"Broadcast {parent} {tfs = }".encode())

        self.tf_broadcaster.sendTransform(tfs)    


# simplified code for 2D robot
def quaternion_from_euler(ai: float, aj: float, ak: float) -> np.ndarray:
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

def euler_from_quaternion(x: float, y: float, z: float, w: float) -> tuple:
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

def main(args=None):
    rclpy.init(args=args)

    node = Robo24DiynavNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

# This code is needed to run .py file directly
if __name__ == '__main__':
    main()

