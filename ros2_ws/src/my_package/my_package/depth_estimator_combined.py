import rclpy
#import torch
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
        self.dictionary = {}  # Dictionary to store images and timestamps
        # self.lidar_sub_left = Subscriber(self, PointCloud2, '/lidar_left/points_raw')
        # self.lidar_sub_right = Subscriber(self, PointCloud2, '/lidar_right/points_raw')
        self.camera_sub_back = Subscriber(self, CompressedImage, '/camera_back/image/compressed')
        self.camera_sub_back_left = Subscriber(self, CompressedImage, '/camera_back_left/image/compressed')
        # self.camera_sub_back_right = Subscriber(self, CompressedImage, '/camera_back_right/image/compressed')
        # self.camera_sub_front_left = Subscriber(self, CompressedImage, '/camera_front_left/image/compressed')
        # self.camera_sub_front_right = Subscriber(self, CompressedImage, '/camera_front_right/image/compressed')
        # self.camera_sub_front_narrow = Subscriber(self, CompressedImage, '/camera_front_narrow/image/compressed')
        # self.camera_sub_front_wide = Subscriber(self, CompressedImage, '/camera_front_wide/image/compressed')
        #Approximate time synchroniser
        print("Before synchromiser")
        self.sync = ApproximateTimeSynchronizer(
            [self.camera_sub_back, self.camera_sub_back_left],
            queue_size=10, slop=0.05) # slope adjusted to 50ms to prevent boundary condition for lidar cycle 
            #self.lidar_sub_left, self.lidar_sub_right, self.camera_sub_back, self.camera_sub_back_left, self.camera_sub_back_right, self.camera_sub_front_left, self.camera_sub_front_right, self.camera_sub_front_narrow, self.camera_sub_front_wide
        self.sync.registerCallback(self.callback_all_topics)
        print("After sync synchromiser")
        #self.publisher = self.create_publisher(Bool, '/bool_set_01', 10)
        self.i = 0
        self.n = 0
        #self.bool_msg = Bool()
        ####################################### Lidar Data
        self.lidar_left = []
        self.lidar_right = []
        ####################################### Camera Data
        self.camera_back = []
        self.camera_back_left = []
        self.camera_back_right = []
        self.camera_front_left = []
        self.camera_front_right = []
        self.camera_front_narrow = []
        self.camera_front_wide = []
        #self.timer = self.create_timer(60.0, self.save_point_cloud) # set a timer to determine the duration of each batch
        self.camera_ctr = [0] *7                         # to accept only every third image for storing to point cloud
        self.counter = 0
        self.n = 30

    def save_point_cloud(self):
    
        print(f"Saving batch {self.i}")
        # Concatenate point cloud data from LIdar 1 , Lidar 2 , Camera 1-7
        #concat_data = np.concatenate([self.camera_back, self.camera_back_left], axis=0)
        # self.lidar_left, self.lidar_right, self.camera_back, self.camera_back_left, self.camera_back_right, self.camera_front_left, self.camera_front_right, self.camera_front_narrow, self.camera_front_wide
        file_path = f'/home/rohin/camlid/camlidconfig/ros2_ws/src/my_package/batch_files/concatenated_point_cloud_{self.i}.npz'
        for timestamp, data in self.dictionary.items():
            print("Timestamp:", timestamp)
            # Iterate over the keys and values in the inner dictionary
            if self.n ==1:
                for item in data:
                    camera_name = item["camera_name"]
                    image = item["image"]
                    print(camera_name ,":", image)
            else:
                for key, value in data.items():
                    print(key, ":", value)
        # Convert keys to strings if they are not already strings
        string_keys_dict = {str(key): value for key, value in self.dictionary.items()}

        # Save the dictionary using np.savez
        np.savez(file_path, **string_keys_dict)
        print("Batch saving is completed")
        self.i = 1
        ############################## Clear Lidar data for the next batch
        # self.lidar_left.clear()
        # self.lidar_right.clear()
        # ####################################### Clear Camera Data for the next batch
        # self.camera_back.clear()
        # self.camera_back_left.clear()
        # self.camera_back_right.clear()
        # self.camera_front_left.clear()
        # self.camera_front_right.clear()
        # self.camera_front_narrow.clear()
        # self.camera_front_wide.clear()
        self.dictionary.clear()   

      
    def callback_all_topics(self, camera_back_msg, camera_backleft_msg):
        if self.counter % self.n == 0:
            print("Inside call back")
            # self, lidar_01_msg, lidar_02_msg, camera_back_msg, camera_backleft_msg , camera_backright_msg, camera_frontleft_msg, camera_frontright_msg, camera_frontnarrow_msg, camera_frontwide_msg
            ############################# Lidar point cloud #################################################
            #For Lidar left

            # # Convert PointCloud2 message to NumPy array
            # cloud_data = np.frombuffer(lidar_01_msg.data, dtype=np.float32).reshape(-1, lidar_01_msg.point_step // 4)
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

            # Camera Back
            print("Camera back negin ")
            cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(camera_back_msg)   # Convert the image to cv2
            with open('/home/rohin/camlid/camlidconfig/ros2_ws/src/my_package/intrinsic/camera_back.yaml', 'r') as file:
                camera_data = yaml.safe_load(file)            
                camera_matrix = np.array(camera_data['camera_matrix']['data']).reshape(3, 3)       # Extract camera matrix and distortion coefficients from the YAML file
                dist_coeffs = np.array(camera_data['distortion_coefficients']['data'])
            rectified_image_back = cv2.undistort(cv_image, camera_matrix, dist_coeffs)   
            # transform = transforms.Compose([
            # transforms.Resize((192, 640)),  # Resize image to match pretrained model input size for the depth model
            # transforms.ToTensor(),
            # transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),  # Normalize image , could be wrong need recheck also reference the original
            # ])    
            # input_image = transform(rectified_image).unsqueeze(0)     

            # rgb = torch.tensor(input_image).permute(2,0,1).unsqueeze(0)/255.

            # depth_pred = packnet_model(rgb) #depth prediction output
            # depth_pred_np = depth_pred.detach().cpu().numpy()  # Convert to NumPy array

            # # Generate pixel coordinates
            # h, w = depth_pred_np.shape[:2]
            # y, x = np.mgrid[0:h, 0:w]

            # # Apply camera intrinsics to obtain 3D coordinates
            # K_inv = np.linalg.inv(camera_matrix)  # Inverse of camera matrix
            # xyz_camera = np.dot(K_inv, np.vstack((x.flatten(), y.flatten(), np.ones((1, h*w))))) * depth_pred_np.flatten()

            
            # pcd = o3d.geometry.PointCloud()
            # pcd.points = o3d.utility.Vector3dVector(xyz_camera.T)
            ############################################ camera_backleft
            print("Camera back left begin")
            cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(camera_backleft_msg)   # Convert the image to cv2
            with open('/home/rohin/camlid/camlidconfig/ros2_ws/src/my_package/intrinsic/camera_back_left.yaml', 'r') as file:
                camera_data = yaml.safe_load(file)            
                camera_matrix = np.array(camera_data['camera_matrix']['data']).reshape(3, 3)       # Extract camera matrix and distortion coefficients from the YAML file
                dist_coeffs = np.array(camera_data['distortion_coefficients']['data'])
            rectified_image_back_left = cv2.undistort(cv_image, camera_matrix, dist_coeffs)     
            
            # Get the timestamps of the images
            timestamp_back = camera_back_msg.header.stamp.sec *1000
            timestamp_back_left = camera_backleft_msg.header.stamp.sec * 1000

            # Store the images and timestamps in the dictionary
            
            if timestamp_back == timestamp_back_left:
                
                self.dictionary[timestamp_back] = [
                    {"camera_name": "back", "image": rectified_image_back},
                    {"camera_name": "back_left", "image": rectified_image_back_left}
                ]
                self.n = 1
            else:
                # Otherwise, store them separately
                self.dictionary[timestamp_back] = {"camera_name": "back", "image": rectified_image_back}
                self.dictionary[timestamp_back_left] = {"camera_name": "back_left", "image": rectified_image_back_left}
                self.n = 0


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

