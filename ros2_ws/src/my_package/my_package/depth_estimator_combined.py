import rclpy
import numpy as np
from ctypes import *
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2 , PointField
from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge
import cv2
import yaml
import time

import open3d as o3d
#Following class will store the image and lidar data as a batch file
class Depth_Subscriber(Node):

    def __init__(self):
        super().__init__('depth_subscriber')
        self.cv_bridge = CvBridge()
        self.dictionary = {}  # Dictionary to store images , lidar data and timestamps for the same
        self.lidar_sub_left = Subscriber(self, PointCloud2, '/lidar_left/points_raw') # we are considering storing only the front wide, front left and left lidar data to the batch
        self.camera_sub_front_left = Subscriber(self, CompressedImage, '/camera_front_left/image/compressed')
        self.camera_sub_front_wide = Subscriber(self, CompressedImage, '/camera_front_wide/image/compressed')
        #Approximate time synchroniser
        print("Before synchromiser")
        self.sync = ApproximateTimeSynchronizer(
            [self.lidar_sub_left, self.camera_sub_front_left, self.camera_sub_front_wide],
            queue_size=10, slop=0.05) # slope adjusted to 50ms to prevent boundary condition for lidar cycle 
            #self.lidar_sub_left, self.lidar_sub_right, self.camera_sub_back, self.camera_sub_back_left, self.camera_sub_back_right, self.camera_sub_front_left, self.camera_sub_front_right, self.camera_sub_front_narrow, self.camera_sub_front_wide
        self.sync.registerCallback(self.callback_all_topics)
        print("After sync synchromiser")
        #self.publisher = self.create_publisher(Bool, '/bool_set_01', 10)
        self.i = 0
        self.n = 0
        #self.bool_msg = Bool()
        
        
        #self.timer = self.create_timer(60.0, self.save_point_cloud) # set a timer to determine the duration of each batch
        self.camera_ctr = [0] *7                         # to accept only every third image for storing to point cloud
        self.counter = 0
        self.n = 30

    def save_point_cloud(self):
    
        print(f"Saving batch {self.i}") # to confirm entry to save batch function
        file_path = f'/home/rohin/camlid/camlidconfig/ros2_ws/src/my_package/batch_files/concatenated_point_cloud_lid_cam_{self.i}.npz'
        # for timestamp, data in self.dictionary.items():
        #     print("Timestamp:", timestamp)
        #     # Iterate over the keys and values in the inner dictionary
        #     if self.n ==1:
        #         for item in data:
        #             camera_name = item["camera_name"]
        #             image = item["image"]
        #             print(camera_name ,":", image)
        #     else:
        #         for key, value in data.items():
        #             print(key, ":", value)
        # Convert keys to strings if they are not already strings
        string_keys_dict = {str(key): value for key, value in self.dictionary.items()}

        # Save the dictionary using np.savez
        np.savez(file_path, **string_keys_dict)
        print("Batch saving is completed")
        self.i = 1        
        self.dictionary.clear()   

      
    def callback_all_topics(self, lidarleft_msg ,camera_frontwide_msg, camera_frontleft_msg):
        if self.counter % self.n == 0:
            print("Inside call back")
            # self, lidar_01_msg, lidar_02_msg, camera_back_msg, camera_backleft_msg , camera_backright_msg, camera_frontleft_msg, camera_frontright_msg, camera_frontnarrow_msg, camera_frontwide_msg
            ############################# Lidar point cloud #################################################
            ####################For Lidar left

            # Convert PointCloud2 message to NumPy array
            cloud_data_lidar_left = np.frombuffer(lidarleft_msg.data, dtype=np.float32).reshape(-1, lidarleft_msg.point_step // 4)
            # # Extract XYZ coordinates
            # xyz = cloud_data[:, :3]
            # # Create Open3D point cloud
            # pcd = o3d.geometry.PointCloud()
            # pcd.points = o3d.utility.Vector3dVector(xyz)
            # self.lidar_left.append(pcd)                       # Data appended to the list for Lidar left
            # #For Lidar right

            # # Convert PointCloud2 message to NumPy array
            # cloud_data = np.frombuffer(lidar_02_msg.data, dtype=np.float32).reshape(-1, lidar_02_msg.point_step // 4)
            # # Extract XYZ coordinates
            # xyz = cloud_data[:, :3]
            # # Create Open3D point cloud
            # pcd = o3d.geometry.PointCloud()
            # pcd.points = o3d.utility.Vector3dVector(xyz)
            # self.lidar_right.append(pcd)                     # data appended to list for Lidar right
            ################################### Camera point cloud ###################################################################

            # Camera Front Wide
            print("Camera front wide begin ")
            cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(camera_frontwide_msg)   # Convert the image to cv2
            with open('/home/rohin/camlid/camlidconfig/ros2_ws/src/my_package/intrinsic/camera_front_wide.yaml', 'r') as file:
                camera_data = yaml.safe_load(file)            
                camera_matrix = np.array(camera_data['camera_matrix']['data']).reshape(3, 3)       # Extract camera matrix and distortion coefficients from the YAML file
                dist_coeffs = np.array(camera_data['distortion_coefficients']['data'])
            rectified_image_front_wide = cv2.undistort(cv_image, camera_matrix, dist_coeffs)   
            

            
            ############################################ Camera Front Left
            print("Camera front left begin")
            cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(camera_frontleft_msg)   # Convert the image to cv2
            with open('/home/rohin/camlid/camlidconfig/ros2_ws/src/my_package/intrinsic/camera_front_left.yaml', 'r') as file:
                camera_data = yaml.safe_load(file)            
                camera_matrix = np.array(camera_data['camera_matrix']['data']).reshape(3, 3)       # Extract camera matrix and distortion coefficients from the YAML file
                dist_coeffs = np.array(camera_data['distortion_coefficients']['data'])
            rectified_image_front_left = cv2.undistort(cv_image, camera_matrix, dist_coeffs)     
            
            # Get the timestamps of the images and the lidar
            timestamp_front_left = camera_frontleft_msg.header.stamp.sec *1000
            timestamp_front_wide = camera_frontwide_msg.header.stamp.sec * 1000
            timestamp_lidar_left = lidarleft_msg.header.stamp.sec * 1000

            # Store the images and timestamps in the dictionary
            
            if timestamp_front_left == timestamp_front_wide and timestamp_front_left == timestamp_lidar_left:
                self.dictionary[timestamp_front_left] = [
                    {"camera_name": "front_wide", "image": rectified_image_front_wide},
                    {"camera_name": "front_left", "image": rectified_image_front_left},
                    {"lidar_name": "left", "image": cloud_data_lidar_left}
                ]
            elif timestamp_front_left == timestamp_front_wide:
                self.dictionary[timestamp_front_left] = [
                    {"camera_name": "front_wide", "image": rectified_image_front_wide},
                    {"camera_name": "front_left", "image": rectified_image_front_left}
                ]
                self.dictionary[timestamp_lidar_left] = {"lidar_name": "left", "image": cloud_data_lidar_left}
            else:
    
                # Otherwise, store them separately
                self.dictionary[timestamp_front_wide] = {"camera_name": "front_wide", "image": rectified_image_front_wide}
                self.dictionary[timestamp_front_left] = {"camera_name": "front_left", "image": rectified_image_front_left}
                self.dictionary[timestamp_lidar_left] = {"lidar_name": "left", "image": cloud_data_lidar_left}
                

            ############################################ camera_backright
            # cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(camera_backright_msg)   # Convert the image to cv2
            # with open('/home/rohin/camlid/camlidconfig/ros2_ws/src/my_package/intrinsic/camera_back_right.yaml', 'r') as file:
            #     camera_data = yaml.safe_load(file)            
            #     camera_matrix = np.array(camera_data['camera_matrix']['data']).reshape(3, 3)       # Extract camera matrix and distortion coefficients from the YAML file
            #     dist_coeffs = np.array(camera_data['distortion_coefficients']['data'])
            # rectified_image = cv2.undistort(cv_image, camera_matrix, dist_coeffs)     
            # self.camera_back_right.append(rectified_image) 

            # ############################################ camera_front left
            # cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(camera_frontleft_msg)   # Convert the image to cv2
            # with open('/home/rohin/camlid/camlidconfig/ros2_ws/src/my_package/intrinsic/camera_front_left.yaml', 'r') as file:
            #     camera_data = yaml.safe_load(file)            
            #     camera_matrix = np.array(camera_data['camera_matrix']['data']).reshape(3, 3)       # Extract camera matrix and distortion coefficients from the YAML file
            #     dist_coeffs = np.array(camera_data['distortion_coefficients']['data'])
            # rectified_image = cv2.undistort(cv_image, camera_matrix, dist_coeffs)     
            # self.camera_front_left.append(rectified_image) 

            # ############################################ camera_front right
            # cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(camera_frontright_msg)   # Convert the image to cv2
            # with open('/home/rohin/camlid/camlidconfig/ros2_ws/src/my_package/intrinsic/camera_front_right.yaml', 'r') as file:
            #     camera_data = yaml.safe_load(file)            
            #     camera_matrix = np.array(camera_data['camera_matrix']['data']).reshape(3, 3)       # Extract camera matrix and distortion coefficients from the YAML file
            #     dist_coeffs = np.array(camera_data['distortion_coefficients']['data'])
            # rectified_image = cv2.undistort(cv_image, camera_matrix, dist_coeffs)     
            # self.camera_front_right.append(rectified_image) 

            # ############################################ camera_front narrow
            # cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(camera_frontnarrow_msg)   # Convert the image to cv2
            # with open('/home/rohin/camlid/camlidconfig/ros2_ws/src/my_package/intrinsic/camera_front_narrow.yaml', 'r') as file:
            #     camera_data = yaml.safe_load(file)            
            #     camera_matrix = np.array(camera_data['camera_matrix']['data']).reshape(3, 3)       # Extract camera matrix and distortion coefficients from the YAML file
            #     dist_coeffs = np.array(camera_data['distortion_coefficients']['data'])
            # rectified_image = cv2.undistort(cv_image, camera_matrix, dist_coeffs)     
            # self.camera_front_narrow.append(rectified_image) 

            # ############################################ camera_front wide
            # cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(camera_frontwide_msg)   # Convert the image to cv2
            # with open('/home/rohin/camlid/camlidconfig/ros2_ws/src/my_package/intrinsic/camera_front_wide.yaml', 'r') as file:
            #     camera_data = yaml.safe_load(file)            
            #     camera_matrix = np.array(camera_data['camera_matrix']['data']).reshape(3, 3)       # Extract camera matrix and distortion coefficients from the YAML file
            #     dist_coeffs = np.array(camera_data['distortion_coefficients']['data'])
            # rectified_image = cv2.undistort(cv_image, camera_matrix, dist_coeffs)     
            # self.camera_front_wide.append(rectified_image)  
        self.counter += 1 
        if self.i == 0:

            self.save_point_cloud()

        print("One cycle completed")             

        

 




def main(args=None):
    rclpy.init(args=args)
    lidar_subscriber = Depth_Subscriber()
    rclpy.spin(lidar_subscriber)
    lidar_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

