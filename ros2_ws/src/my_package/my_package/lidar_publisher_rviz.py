import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import open3d as o3d
import struct
from scipy.spatial.transform import Rotation as R



class PCDPublisher(Node):

    def __init__(self):
        super().__init__('pcd_publisher')
        self.path_01 = "/home/rohin/camlid/camlidconfig/ros2_ws/src/my_package/ply_files/point_cloud_1_DA.ply"
        self.path_02 = "/home/rohin/camlid/camlidconfig/ros2_ws/src/my_package/ply_files/point_cloud_2_DA.ply"
        self.path_npz = "/home/rohin/camlid/camlidconfig/ros2_ws/src/my_package/ply_files/lidar_01.npz"

        # Publishers
        self.pcd_pub1 = self.create_publisher(PointCloud2, '/pcd1_visualization', 10)
        self.pcd_pub2 = self.create_publisher(PointCloud2, '/pcd2_visualization', 10)
        self.pcd_pub3 = self.create_publisher(PointCloud2, '/pcd3_visualization', 10)
        ################################## Prepre lidar pcd3
        # Load the .npz file
        self.points_key='data'
        self.data = np.load(self.path_npz)  
        print("Keys in the npz file:", self.data.keys())
        self.points = self.data[self.points_key]
        # Debug: Print the shape and type of the points array
        print(f"Type of points: {type(self.points)}")
        print(f"Shape of points: {self.points.shape}")
        self.points_xyz = self.points[:, :3]  
        self.pcd3 = o3d.geometry.PointCloud()
        self.pcd3.points = o3d.utility.Vector3dVector(self.points_xyz)
        ################################### Prepare pcd 1
        self.pcd1 = o3d.io.read_point_cloud(self.path_01)
        ################################## Prepare pcd 2
        self.pcd2 = o3d.io.read_point_cloud(self.path_02)
        
        # Extract z-values
        self.z_values_lidar = np.asarray(self.pcd3.points)[:, 2]
        self.z_values_pcd1 = np.asarray(self.pcd1.points)[:, 2]
        self.z_values_pcd2 = np.asarray(self.pcd2.points)[:, 2]

        # Calculate mean or minimum z-value for alignment
        self.reference_z = np.mean(self.z_values_lidar)  # or np.min(z_values_lidar)
        self.pcd1_z = np.mean(self.z_values_pcd1)  # or np.min(z_values_pcd1)
        self.pcd2_z = np.mean(self.z_values_pcd2)  # or np.min(z_values_pcd2)

        # Calculate the offset
        self.offset_pcd1 =self.reference_z - self.pcd1_z
        self.offset_pcd2 = self.reference_z - self.pcd2_z

        # Adjust the z-values of PCD1 and PCD2
        self.points_pcd1 = np.asarray(self.pcd1.points)
        self.points_pcd2 = np.asarray(self.pcd2.points)

        # self.points_pcd1[:, 2] += self.offset_pcd1
        # self.points_pcd2[:, 2] += self.offset_pcd2
        # Update the point clouds with new z-values
        self.pcd1.points = o3d.utility.Vector3dVector(self.points_pcd1)
        self.pcd2.points = o3d.utility.Vector3dVector(self.points_pcd2)
        scale_factor = 0.1  # Example scale factor if your depth data is in millimeters use 0.08
        self.pcd1.points = o3d.utility.Vector3dVector(np.asarray(self.pcd1.points)* scale_factor)
        self.pcd2.points = o3d.utility.Vector3dVector(np.asarray(self.pcd2.points) * scale_factor)
        # Apply transformations
        self.apply_transformations()
        self.timer = self.create_timer(1.0, self.publish_pcds)  # 1 Hz
        # self.visualize_point_cloud(self.pcd1,self.pcd2)
    def visualize_point_cloud(self,pcd1 ,pcd2):
        print("Load a ply point cloud, print it, and render it")
        o3d.visualization.draw_geometries([pcd1],
                                    zoom=0.3412,
                                    front=[0.4257, -0.2125, -0.8795],
                                    lookat=[2.6172, 2.0475, 1.532],
                                    up=[-0.0694, -0.9768, 0.2024])
        o3d.visualization.draw_geometries([pcd2],
                                    zoom=0.3412,
                                    front=[0.4257, -0.2125, -0.8795],
                                    lookat=[2.6172, 2.0475, 1.532],
                                    up=[-0.0694, -0.9768, 0.2024])
    def convert_to_pointcloud2(self, pcd):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'  # Replace 'map' with the appropriate frame

        points = np.asarray(pcd.points)

        
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        cloud_data = []
        for point in points:
            cloud_data.append(struct.pack('fff', point[0], point[1], point[2]))

        cloud_msg = PointCloud2(
            header=header,
            height=1,
            width=len(points),
            is_dense=True,
            is_bigendian=False,
            fields=fields,
            point_step=12,
            row_step=12 * len(points),
            data=b''.join(cloud_data)
        )

        return cloud_msg

    def publish_pcds(self):
        cloud_msg1 = self.convert_to_pointcloud2(self.pcd1)
        cloud_msg2 = self.convert_to_pointcloud2(self.pcd2)
        cloud_msg3 = self.convert_to_pointcloud2(self.pcd3)

        self.pcd_pub1.publish(cloud_msg1)
        self.pcd_pub2.publish(cloud_msg2)
        self.pcd_pub3.publish(cloud_msg3)
    def quaternion_to_rotation_matrix(self, qx, qy, qz, qw):
        # Create a Rotation object from the quaternion
        r = R.from_quat([qx, qy, qz, qw])
        # Convert to a 3x3 rotation matrix
        rotation_matrix = r.as_matrix()
        return rotation_matrix

    def apply_transformations(self):
        # Extrinsics for Camera 1 (relative to LiDAR)
        rotation_matrix1 = self.quaternion_to_rotation_matrix(-0.5204131359731259, 0.483559134543592, -0.48212418510268484, 0.512734825660536)
        translation1 = np.array([1.4855427639599839, 0.28616353316692766, 1.5617304615771417])

        # Extrinsics for Camera 2 (relative to LiDAR)
        rotation_matrix2 = self.quaternion_to_rotation_matrix(-0.6751131476133265, 0.23003455787756122, -0.2338379349739517, 0.6607769368469352)
        translation2 = np.array([1.52093975696107, 0.4574157637366625, 1.5753242209317193])

        # Apply transformations to PCD1 (Camera 1)
        points_pcd1 = np.asarray(self.pcd1.points)
        points_pcd1 = np.dot(points_pcd1, rotation_matrix1.T) + translation1
        self.pcd1.points = o3d.utility.Vector3dVector(points_pcd1)

        # Apply transformations to PCD2 (Camera 2)
        points_pcd2 = np.asarray(self.pcd2.points)
        points_pcd2 = np.dot(points_pcd2, rotation_matrix2.T) + translation2
        self.pcd2.points = o3d.utility.Vector3dVector(points_pcd2)

def main(args=None):
    rclpy.init(args=args)
    pcd_publisher = PCDPublisher()
    rclpy.spin(pcd_publisher)

    pcd_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()