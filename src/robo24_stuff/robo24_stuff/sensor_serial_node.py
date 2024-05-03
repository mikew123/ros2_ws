#!/usr/bin/env python3

import rclpy
#import sys
import serial
#import math
#import time
#import numpy as np

from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Temperature

class SensorSerialNode(Node):
    # parameters?

    timerRateHz = 30.0; # Rate to check serial port for messages

    reflVal:int = 25 # TOF8x8 reflectance default = 25
    sigmVal:int = 40 # TOF8x8 sigma value default = 10

    #serial_port = "/dev/ttyACM2"
    serial_port:str = "/dev/serial/by-id/usb-Waveshare_RP2040_Zero_E6625887D3477130-if00"

    def __init__(self):
        super().__init__('sensor_serial_node')

        self.sensor_serial_port = serial.Serial(self.serial_port, 2000000)
        # configure interface
        self.sensor_serial_port.write(f"MODE ROS2\n".encode()) # extra tfor startup
        self.sensor_serial_port.write(f"MODE ROS2\n".encode())
        self.sensor_serial_port.write(f"REFL {self.reflVal}\n".encode())
        self.sensor_serial_port.write(f"SIGM {self.sigmVal}\n".encode())


        self.tof8x8x3_msg_publisher = self.create_publisher(String, 'tof8x8x3_msg', 10)
        self.battery_status_msg_publisher = self.create_publisher(BatteryState, 'battery_status', 10)
        self.temperature_msg_publisher = self.create_publisher(Temperature, 'temperature', 10)

        self.timer = self.create_timer((1.0/self.timerRateHz), self.timer_callback)

        self.get_logger().info(f"SensorSerialNode Started")

    # check serial port at timerRateHz and parse out messages to publish
    def timer_callback(self):
        # Check if a line has been received on the serial port
        if self.sensor_serial_port.in_waiting > 0:
            received_data = self.sensor_serial_port.readline().decode().strip()
            #self.get_logger().info(f"Received: {received_data}")
            
            strArray = received_data.split(" ")
            if strArray[0]=="TOF8x8x3" :
                # Publish the received serial line as a String message
                emsg = String()
                emsg.data = received_data
                self.tof8x8x3_msg_publisher.publish(emsg)
            elif strArray[0]=="BT" and len(strArray)==4:
                volts:float = float(strArray[1])/1000
                amps:float = float(strArray[2])/1000
                tempC:float = float(strArray[3])
                # send Batter State message
                bmsg = BatteryState()
                bmsg.header.stamp = self.get_clock().now().to_msg()
                bmsg.header.frame_id = "base_link"
                bmsg.voltage = volts
                bmsg.current = amps
                bmsg.temperature = tempC
                bmsg.present = True
                self.battery_status_msg_publisher.publish(bmsg)
                # send Temperature message
                tmsg = Temperature()
                tmsg.header.stamp = self.get_clock().now().to_msg()
                tmsg.header.frame_id = "base_link"
                tmsg.temperature = tempC
                tmsg.variance = 0.0 # unknown
                self.temperature_msg_publisher.publish(tmsg)
            else :
                self.get_logger().error(f"Invalid serial sensor message {received_data=}")

def main(args=None):
    rclpy.init(args=args)

    node = SensorSerialNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

# This code is needed to run .py file directly
if __name__ == '__main__':
    main()

