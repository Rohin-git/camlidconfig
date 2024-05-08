import rclpy
import numpy as np
from ctypes import *
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import yaml

class CameraVisualizeSubscriber(Node):

    def __init__(self):
        super().__init__('camera_visualize_subscriber')
        #self.subscription = self.create_subscription(CompressedImage,'/camera_back/image/compressed',self.camera_back_callback,10)
        # self.subscription = self.create_subscription(CompressedImage,'/camera_back_left/image/compressed',self.camera_back_left_callback,10)
        # self.subscription = self.create_subscription(CompressedImage,'/camera_back_right/image/compressed',self.camera_back_right_callback,10)
        # self.subscription = self.create_subscription(CompressedImage,'/camera_front_left/image/compressed',self.camera_front_left_callback,10)
        # self.subscription = self.create_subscription(CompressedImage,'/camera_front_right/image/compressed',self.camera_front_right_callback,10)
        # self.subscription = self.create_subscription(CompressedImage,'/camera_front_narrow/image/compressed',self.camera_front_narrow_callback,10)
        self.subscription = self.create_subscription(CompressedImage,'/camera_front_wide/image/compressed',self.camera_front_wide_callback,10)
        self.subscription
        self.cv_bridge = CvBridge()
        
        

    # def camera_back_callback(self, msg):
    #     print("Inside camera_back_callback")
    #     cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(msg)
    #     with open('/home/rohin/camlid/camlidconfig/ros2_ws/src/my_package/intrinsic/camera_back.yaml', 'r') as file:
    #         camera_data = yaml.safe_load(file)            
    #         camera_matrix = np.array(camera_data['camera_matrix']['data']).reshape(3, 3)       # Extract camera matrix and distortion coefficients from the YAML file
    #         dist_coeffs = np.array(camera_data['distortion_coefficients']['data'])
    #     rectified_image = cv2.undistort(cv_image, camera_matrix, dist_coeffs)
    #     custom_resolution = (320, 240)  
    #     cv_image_back = cv2.resize(cv_image, custom_resolution)
    #     cv_image_back_rectified = cv2.resize(rectified_image, custom_resolution)

        
    #     concatenated_image = np.concatenate((cv_image_back, cv_image_back_rectified), axis=1)

    #     # Display the concatenated image 
    #     cv2.imshow('OG Image vs Rectified Image [ Camera Back ]', concatenated_image)
    #     cv2.waitKey(200) 

    # def camera_back_left_callback(self, msg):
    #     print("Inside camera_back_left_callback")
    #     cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(msg)

    #     custom_resolution = (320, 240)  
    #     cv_image_back_left = cv2.resize(cv_image, custom_resolution)

    #     # 
    #     cv2.imshow('Image_back_left', cv_image_back_left)
    #     cv2.waitKey(200) 

    # def camera_back_right_callback(self, msg):
    #     print("Inside camera_back_right_callback")
    #     cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(msg)  

    #     custom_resolution = (320, 240)  
    #     cv_image_back_right = cv2.resize(cv_image, custom_resolution)

    #     # 
    #     cv2.imshow('Image_back_right', cv_image_back_right)
    #     cv2.waitKey(200)

    # def camera_front_left_callback(self, msg):
    #     print("Inside camera_front_left_callback")
    #     cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(msg)

    #     custom_resolution = (320, 240)  
    #     cv_image_front_left = cv2.resize(cv_image, custom_resolution)

    #     # 
    #     cv2.imshow('Image_front_left', cv_image_front_left)
    #     cv2.waitKey(200)

    # def camera_front_right_callback(self, msg):
    #     print("Inside camera_front_right_callback")
    #     cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(msg)

    #     custom_resolution = (320, 240)  
    #     cv_image_front_right = cv2.resize(cv_image, custom_resolution)
    #     # 
    #     cv2.imshow('Image_front_right', cv_image_front_right)
    #     cv2.waitKey(200)

    # def camera_front_narrow_callback(self, msg):
    #     print("Inside camera_front_narrow_callback")
    #     cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(msg)

    #     custom_resolution = (320, 240)  
    #     cv_image_front_narrow = cv2.resize(cv_image, custom_resolution)

    #     # 
    #     cv2.imshow('Image_front_narrow', cv_image_front_narrow)
    #     cv2.waitKey(200)

    def camera_front_wide_callback(self, msg):
        print("Inside camera_front_wide_callback")
        cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(msg)
        with open('/home/rohin/camlid/camlidconfig/ros2_ws/src/my_package/intrinsic/camera_front_wide.yaml', 'r') as file:
            camera_data = yaml.safe_load(file)            
            camera_matrix = np.array(camera_data['camera_matrix']['data']).reshape(3, 3)       # Extract camera matrix and distortion coefficients from the YAML file
            dist_coeffs = np.array(camera_data['distortion_coefficients']['data'])
        rectified_image = cv2.undistort(cv_image, camera_matrix, dist_coeffs)
        custom_resolution = (320, 240)  
        cv_image_front_wide = cv2.resize(cv_image, custom_resolution)
        cv_image_front_wide_rectified = cv2.resize(rectified_image, custom_resolution)

        
        concatenated_image = np.concatenate((cv_image_front_wide, cv_image_front_wide_rectified), axis=1)

        # Display the concatenated image 
        cv2.imshow('OG Image vs Rectified Image [ Camera Front Wide ]', concatenated_image)
        cv2.waitKey(200) 
        


def main(args=None):
    rclpy.init(args=args)
    #visualize_lidar(path_01)
    #crop_car_lidar(path_01)
    camera_visualize_subscriber = CameraVisualizeSubscriber()
    rclpy.spin(camera_visualize_subscriber)
    camera_visualize_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

