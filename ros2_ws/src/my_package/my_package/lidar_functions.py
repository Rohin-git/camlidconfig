import rclpy
import numpy as np
from ctypes import *
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2 , PointField
from std_msgs.msg import Bool
from camviz import CamViz

import open3d as o3d

path_01 = "/home/rohin/camlid/camlidconfig/ros2_ws/src/my_package/ply_files/point_cloud_1.ply"
path_02 = "/home/rohin/camlid/camlidconfig/ros2_ws/src/my_package/ply_files/point_cloud_2.ply"
path_npz = "/home/rohin/camlid/camlidconfig/ros2_ws/src/my_package/ply_files/lidar_01.npz"

def lidar_camviz(pcd1, pcd2, pcd3):
    # Initialize Camviz
    viz = CamViz()

    # Add point clouds to Camviz
    viz.add_point_cloud(pcd1, color=(1, 0, 0))  # Red color for pc1
    viz.add_point_cloud(pcd2, color=(0, 1, 0))  # Green color for pc2
    viz.add_point_cloud(pcd3, color=(0, 0, 1))  # Blue color for pc3

    # Display the visualization
    viz.show()

def visualize_lidar_npz(npz_path, points_key='data'):
    # Load the .npz file
    data = np.load(npz_path)
    
    
    print("Keys in the npz file:", data.keys())
    points = data[points_key]

    # Debug: Print the shape and type of the points array
    print(f"Type of points: {type(points)}")
    print(f"Shape of points: {points.shape}")
    
     
    points_xyz = points[:, :3]  
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_xyz)

    # Visualize the point cloud
    o3d.visualization.draw_geometries([pcd],
                                      zoom=0.3412,
                                      front=[0.4257, -0.2125, -0.8795],
                                      lookat=[2.6172, 2.0475, 1.532],
                                      up=[-0.0694, -0.9768, 0.2024])
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
    ind = np.where(dists > 1.3)[0]
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
    #visualize_lidar_npz(path_npz)
    #visualize_lidar(path_02)
    #crop_car_lidar(path_01)
    ################################## Prepre lidar pcd3
    # Load the .npz file
    points_key='data'
    data = np.load(path_npz)  
    print("Keys in the npz file:", data.keys())
    points = data[points_key]
    # Debug: Print the shape and type of the points array
    print(f"Type of points: {type(points)}")
    print(f"Shape of points: {points.shape}")
    points_xyz = points[:, :3]  
    pcd3 = o3d.geometry.PointCloud()
    pcd3.points = o3d.utility.Vector3dVector(points_xyz)
    ################################### Prepare pcd 1
    pcd1 = o3d.io.read_point_cloud(path_01)
    ################################## Prepare pcd 2
    pcd2 = o3d.io.read_point_cloud(path_01)
    ########################## Function Call
    lidar_camviz(pcd1, pcd2, pcd3)
    lidar_function_subscriber = LidarFunctionSubscriber()
    rclpy.spin(lidar_function_subscriber)
    lidar_function_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

