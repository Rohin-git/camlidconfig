import rclpy
import numpy as np
from ctypes import *
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2 , PointField
from std_msgs.msg import Bool

import open3d as o3d


class LidarSubscriber(Node):

    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(PointCloud2,'/lidar_left/points_raw',self.lidar_01_callback,10)
        self.subscription = self.create_subscription(PointCloud2,'/lidar_right/points_raw',self.lidar_02_callback,10)
        self.publisher = self.create_publisher(Bool, '/bool_set_01', 10)
        self.subscription
        self.bool_msg = Bool()

    def lidar_01_callback(self, msg):
        # Convert PointCloud2 message to NumPy array
        cloud_data_1 = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, msg.point_step // 4)

        # Extract XYZ coordinates
        xyz = cloud_data_1[:, :3]

        # Create Open3D point cloud
        pcd_1 = o3d.geometry.PointCloud()
        pcd_1.points = o3d.utility.Vector3dVector(xyz)

        # Save point cloud as PLY file
        o3d.io.write_point_cloud('/home/rohin/camlid/camlidconfig/ros2_ws/src/my_package/ply_files/lidar_pointcloud_1.ply', pcd_1)
        self.bool_msg.data = True
        self.publisher.publish(self.bool_msg)
        



    def lidar_02_callback(self, msg):
        # Convert PointCloud2 message to NumPy array
        cloud_data_2 = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, msg.point_step // 4)

        # Extract XYZ coordinates
        xyz = cloud_data_2[:, :3]

        # Create Open3D point cloud
        pcd_2 = o3d.geometry.PointCloud()
        pcd_2.points = o3d.utility.Vector3dVector(xyz)

        # Save point cloud as PLY file
        o3d.io.write_point_cloud('/home/rohin/camlid/camlidconfig/ros2_ws/src/my_package/ply_files/lidar_pointcloud_2.ply', pcd_2)
 

def main(args=None):
    rclpy.init(args=args)
    lidar_subscriber = LidarSubscriber()
    rclpy.spin(lidar_subscriber)
    lidar_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

