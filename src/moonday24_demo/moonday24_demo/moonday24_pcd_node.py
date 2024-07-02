import rclpy
from rclpy.node import Node
import sys

import numpy as np
import math

from std_msgs.msg import String, Header
from sensor_msgs.msg import PointCloud2, PointField

class MoonDay24PcdNode(Node):
    """
    Creates a 8x8x3 point cloud from TOF sensors
    Rviz2 can display point cloud
    """

    # Field of view for TOF 8x8 sensor
    fov8x8:float = 45.0
    mntAngle:float = 45.0

    
    def __init__(self):
        super().__init__('moonday24_pcd_node')

        # TOF sensors to extract point cloud from
        self.tof8x8x3_subscription = self.create_subscription(String, 'tof8x8x3_msg', self.tof8x8x3_callback, 10)
 
        # POINT CLOUD from TOF data
        self.pcd_publisher = self.create_publisher(PointCloud2, 'tof8_pcd', 10)

        self.get_logger().info("moodDay24_pcd Started")

    def tof8x8x3_callback(self, msg: String) -> None:
        """
        get TOF8x8x3 sensors and remove warping for each sensor
        Publish the point cloud for RVIZ to display
        """

        now = rclpy.time.Time() # Gets time=0 (I think simulation time)

        try:
            #split msg string into 193 seperate strings 1 for each element
            tofStrArray = msg.data.split(" ")

            # parse messsage of "name" then 192 integers (8 rows of 24 distances)
            if tofStrArray[0]!="TOF8x8x3" or len(tofStrArray)!=193:
                self.get_logger().error(f"TOF8x8x3 type message error: {msg.data}")
                return
        except:
            self.get_logger().error(f"TOF8x8x3 parse message error: {msg.data}")
            return

        # row 0 is top, row 7 is bottom
        tof8Wall:list[list[float,float]] = []
        for row in range(0,7) : # sensor row
            tof8Row:list[float] = []
            for i in range(0,23) :
                dist = int(tofStrArray[i+(row*24)+1])
                if dist>=0 : tof8Row.append(dist)
                else       : tof8Row.append(0)
            tof8Wall.append(tof8Row)

        # Remove the curve by scaling each sensor with a inverted sin() curve over FOV
        fovPt = self.fov8x8/8 # FOV for each sensor point
        fovPtRad = fovPt*(math.pi/180) #scaled to Radians
        tofCurveCor = []
        for n in range(0,8) :
            theta:float = (n-4+0.5)*fovPtRad + math.pi/2
            s:float = math.sin(theta)
            if n == 0 : s0 = s
            tofCurveCor.append(s0/s)

        #self.get_logger().info(f"{tofCurveCor = }")

        # calc xyz coordinates relative to robot center with 0 deg pointing staight ahead
        # each sensor distance data point has an effective FOV of 60/8 = 7.5 deg
        # there are 8 rows 0 to 7 of 24 data points 0 to 23
        # theta XY is the horizontal angle of the sensors sweeping left to right
        # theta XY = -(45/8)*1/2 + (n-11)*(45/8) -> 0=-64.7 23=-86.25, 11=-2.8, 12=+2.8
        # theta Z is the vertical angle sweeping up to down,
        # theta Z = (45/8)*1/2 + (3-row)*(45/8) -> 0=+19.7, 7=-19.7
        sensorZ = 0.910 # meters above floor when robot is on the demo table, about 1 meter
        xyz:list[np.float32, np.float32, np.float32] = []
        mntAngleRad = self.mntAngle*(math.pi/180) #scaled to Radians
        fovPt = self.fov8x8/8 # FOV for each sensor point
        fovPtRad = fovPt*(math.pi/180) #scaled to Radians

        # calc curve correction for each sensor set of 8
        # Left sensor 0 to 7
        for row in range(0,7) : #Sensor rows
            thetaZ = fovPtRad * (3.5 - row)
            for n in range(0,8) :
                theta = (n-4+0.5)*fovPtRad  - mntAngleRad# scaled to radians
                dist = tof8Wall[row][n]
                Wx =  dist*math.cos(theta)*tofCurveCor[n]/1000
                Wy = -dist*math.sin(theta)*tofCurveCor[n]/1000
                if dist > 0 : Wz =  sensorZ + math.sin(thetaZ)
                else : Wz = sensorZ
                xyz.append([Wx,Wy,Wz])
            # Center sensor 8 to 15
            for n in range(8,16) :
                theta = (n-12+0.5)*fovPtRad# scaled to radians
                dist = tof8Wall[row][n]
                Wx =  dist*math.cos(theta)*tofCurveCor[n-8]/1000
                Wy = -dist*math.sin(theta)*tofCurveCor[n-8]/1000
                if dist > 0 : Wz =  sensorZ + math.sin(thetaZ)
                else : Wz = sensorZ
                xyz.append([Wx,Wy,Wz])
            # Right sensor 16 to 23
            for n in range(16,23) :
                theta = (n-20+0.5)*fovPtRad  + mntAngleRad# scaled to radians
                dist = tof8Wall[row][n]
                Wx =  dist*math.cos(theta)*tofCurveCor[n-16]/1000
                Wy = -dist*math.sin(theta)*tofCurveCor[n-16]/1000
                if dist > 0 : Wz =  sensorZ + math.sin(thetaZ)
                else : Wz = sensorZ
                xyz.append([Wx,Wy,Wz])

        #self.get_logger().info(f"\n{tof8Wall = }\n{xyz = }\n")


        pcd = self.point_cloud(xyz, 'map')
        self.pcd_publisher.publish(pcd)



    def point_cloud(self, points_xy:list[tuple[np.float32]], parent_frame:str="map") -> PointCloud2:
        """
            Input list of tuples (x,y,z) the frame name for xy z is fixed relative offset usually "map"
            Returns a point cloud to publish - Rviz can display it
        """
        points = np.asarray(points_xy)

        ros_dtype = PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.

        data = points.astype(dtype).tobytes() 

        # The fields specify what the bytes represents. The first 4 bytes 
        # represents the x-coordinate, the next 4 the y-coordinate
        fields = [PointField(
            name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate('xyz')]

        #self.get_logger().info(f"{itemsize = } {fields = } {points = } {data = }")

        # The PointCloud2 message also has a header which specifies which 
        # coordinate frame it is represented in. 
        header = Header(
            frame_id=parent_frame,
            #stamp = self.get_clock().now().to_msg(),
            )

        return PointCloud2(
            header=header,
            height=1, 
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False, #Pi4
            fields=fields,
            point_step=(itemsize * 3), # Every point consists of two float32s.
            row_step=(itemsize * 3 * points.shape[0]), 
            data=data
        )


def main(args=None):
    rclpy.init(args=args)

    node = MoonDay24PcdNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

# This code is needed to run .py file directly
if __name__ == '__main__':
    main()
