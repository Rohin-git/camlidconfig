import rclpy
import numpy as np
#from transformers import pipeline
from PIL import Image
import requests
import cv2



def main(args=None):
    print("In Main")
    i = 0
    file_path = '/home/rohin/camlid/camlidconfig/ros2_ws/src/my_package/batch_files/concatenated_point_cloud_lid_cam_0.npz'
    saved_data = np.load(file_path, allow_pickle=True)
    saved_dict = dict(saved_data)
    for timestamp ,  data in saved_dict.items():
        print(data)
        print(i)
        i += 1

if __name__ == '__main__':
    main()

