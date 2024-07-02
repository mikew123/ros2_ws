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

    
    # 24 data points for line of sensors in mm
    tof8Wall:list[int] = [-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1]


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

        ############## WALL MAPPING ##################
        # select sensors on the lower center row for wall mapping
        # row 0 is top, row 7 is bottom
        sensorRow:int = 4
        for i in range(0,24):
            dist = int(tofStrArray[i+(sensorRow*24)+1]) #[i,sensorRow]
            if dist>=0 : self.tof8Wall[i] = dist
            else       : self.tof8Wall[i] = 0

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

        # calc xy coordinates relative to robot center with 0 deg pointing staight ahead
        # each sensor distance data point has an effective FOV of 60/8 = 7.5 deg
        # there are 24 data points n = 0, 1 to 23
        # theta = (60/8)*1/2 + (n-12)*(60/8) -> 0=-86.25 23=+86.25, 11=-3.75, 12=+3.75
        xyW:list = []
        mntAngleRad = self.mntAngle*(math.pi/180) #scaled to Radians
        fovPt = self.fov8x8/8 # FOV for each sensor point
        fovPtRad = fovPt*(math.pi/180) #scaled to Radians

        # calc curve correction for each sensor set of 8
        # Left sensor 0 to 7
        for n in range(0,8) :
            theta = (n-4+0.5)*fovPtRad  - mntAngleRad# scaled to radians
            dist = self.tof8Wall[n]
            Wx =  int(self.tof8Wall[n]*math.cos(theta)*tofCurveCor[n])
            Wy = -int(self.tof8Wall[n]*math.sin(theta)*tofCurveCor[n])
            xyW.append((Wx,Wy))
        # Center sensor 8 to 15
        for n in range(8,16) :
            theta = (n-12+0.5)*fovPtRad# scaled to radians
            dist = self.tof8Wall[n]
            Wx =  int(self.tof8Wall[n]*math.cos(theta)*tofCurveCor[n-8])
            Wy = -int(self.tof8Wall[n]*math.sin(theta)*tofCurveCor[n-8])
            xyW.append((Wx,Wy))
        # Right sensor 16 to 23
        for n in range(16,24) :
            theta = (n-20+0.5)*fovPtRad  + mntAngleRad# scaled to radians
            dist = self.tof8Wall[n]
            Wx =  int(self.tof8Wall[n]*math.cos(theta)*tofCurveCor[n-16])
            Wy = -int(self.tof8Wall[n]*math.sin(theta)*tofCurveCor[n-16])
            xyW.append((Wx,Wy))

        self.get_logger().info(f"\n{self.tof8Wall = }\n{xyW = }\n")

        # TODO: I should not need to rotate anything for moon day demo
        x0:float = 0
        y0:float = 0
        th0:float = 0 # yaw theta
        
        c0:float = math.cos(th0)
        s0:float = math.sin(th0)

        # Create point cloud from TOF8 data
        #Create list of XYZ tupples converting int mm to float meters
        xy_:list[np.float32, np.float32, np.float32] = []
        xy0:list[np.float32, np.float32, np.float32] = []
        for n in range(0,24) :
            # rotate XY with map->base_link angle
            x0y0 = xyW[n] # (x,y)
            zz0 = np.float32(0.130) # height of sensor

            
            xx_ = np.float32(( (x0y0[0]*c0 + x0y0[1]*s0)/1000) )
            yy_ = np.float32((-(x0y0[0]*s0 - x0y0[1]*c0)/1000) )

            # debug
            xy_.append((xx_,yy_,zz0))

            xx0 = xx_ + x0 # - 0.08 # 8mm offset
            yy0 = yy_ + y0
            xy0.append((xx0,yy0,zz0))

        pcd = self.point_cloud(xy0, 'map')
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

        self.get_logger().info(f"{itemsize = } {fields = } {points = } {data = }")

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
