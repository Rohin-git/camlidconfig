import rclpy
import numpy as np
from ctypes import *
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2 , PointField
from std_msgs.msg import Bool

import open3d as o3d

#def execute_functions():
    # Execute functions on PLY file

class LidarFunctionSubscriber(Node):

    def __init__(self):
        super().__init__('lidar_function_subscriber')
        self.subscription = self.create_subscription(Bool,'/bool_set_01',self.lidar_01_function_callback,10)
        self.subscription
        

    def lidar_01_function_callback(self, msg):
        print("Inside lidar call back")

def main(args=None):
    rclpy.init(args=args)
    lidar_function_subscriber = LidarFunctionSubscriber()
    rclpy.spin(lidar_function_subscriber)
    lidar_function_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

