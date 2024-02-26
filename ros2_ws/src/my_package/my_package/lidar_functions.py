import rclpy
import numpy as np
from ctypes import *
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2 , PointField
from std_msgs.msg import Bool

import open3d as o3d

path_01 = "/home/rohin/camlid/camlidconfig/ros2_ws/src/my_package/ply_files/lidar_pointcloud_1.ply"
path_02 = "/home/rohin/camlid/camlidconfig/ros2_ws/src/my_package/ply_files/lidar_pointcloud_2.ply"

def visualize_lidar(path):
    print("Load a ply point cloud, print it, and render it")
    pcd = o3d.io.read_point_cloud(path)
    print(pcd)
    print(np.asarray(pcd.points))
    o3d.visualization.draw_geometries([pcd],
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024])
def crop_car_lidar(path):
    print("Load a ply point cloud, print it, and render it")
    pcd = o3d.io.read_point_cloud(path)
    print(pcd)
    print(np.asarray(pcd.points))

    source = o3d.geometry.PointCloud()
    source.points = o3d.utility.Vector3dVector([[0, 0, 0]])
    
    dists = pcd.compute_point_cloud_distance(source)
    dists = np.asarray(dists)
    ind = np.where(dists > 0.1)[0]
    pcd_without_car = pcd.select_by_index(ind)
    o3d.visualization.draw_geometries([pcd_without_car],
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024])

class LidarFunctionSubscriber(Node):

    def __init__(self):
        super().__init__('lidar_function_subscriber')
        self.subscription = self.create_subscription(Bool,'/bool_set_01',self.lidar_01_function_callback,10)
        self.subscription
        

    def lidar_01_function_callback(self, msg):
        print("Inside lidar call back")

def main(args=None):
    rclpy.init(args=args)
    #visualize_lidar(path_01)
    crop_car_lidar(path_01)
    lidar_function_subscriber = LidarFunctionSubscriber()
    rclpy.spin(lidar_function_subscriber)
    lidar_function_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

