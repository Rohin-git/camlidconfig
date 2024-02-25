import rclpy
import numpy as np
from ctypes import *
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2 , PointField

import open3d as o3d

##################################################################################################
FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
    [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

# Bit operations
BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8
convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff)
)
convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)
##################################################################################################
class LidarSubscriber(Node):

    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(PointCloud2,'/lidar_left/points_raw',self.lidar_01_callback,10)
        self.subscription = self.create_subscription(PointCloud2,'/lidar_right/points_raw',self.lidar_02_callback,10)
        self.subscription

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


    # def convertCloudFromRosToOpen3d(self , ros_cloud):
    
    #     # Get cloud data from ros_cloud
    #     field_names=[field.name for field in ros_cloud.fields]
    #     cloud_data = list(pc2.PointField(ros_cloud, skip_nans=True, field_names = field_names))

    #     # Check empty
    #     open3d_cloud = open3d.PointCloud()
    #     if len(cloud_data)==0:
    #         print("Converting an empty cloud")
    #         return None

    #     # Set open3d_cloud
    #     if "rgb" in field_names:
    #         IDX_RGB_IN_FIELD=3 # x, y, z, rgb
            
    #         # Get xyz
    #         xyz = [(x,y,z) for x,y,z,rgb in cloud_data ] # (why cannot put this line below rgb?)

    #         # Get rgb
    #         # Check whether int or float
    #         if type(cloud_data[0][IDX_RGB_IN_FIELD])==float: # if float (from pcl::toROSMsg)
    #             rgb = [convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
    #         else:
    #             rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

    #         # combine
    #         open3d_cloud.points = o3d.Vector3dVector(np.array(xyz))
    #         open3d_cloud.colors = o3d.Vector3dVector(np.array(rgb)/255.0)
    #     else:
    #         xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
    #         open3d_cloud.points = o3d.Vector3dVector(np.array(xyz))

    #     # return
    #     return open3d_cloud

def main(args=None):
    rclpy.init(args=args)
    lidar_subscriber = LidarSubscriber()
    rclpy.spin(lidar_subscriber)
    lidar_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

