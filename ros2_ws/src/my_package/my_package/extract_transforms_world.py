from math import sin, cos, pi
import urdf_parser_py.urdf as urdf
import numpy as np
import os
import tempfile


# Create Transform function
def create_transformation_matrix(xyz, rpy):
    """Create a 4x4 transformation matrix from xyz and rpy."""
    tx, ty, tz = xyz
    roll, pitch, yaw = rpy
    # Compute the rotation matrix from roll, pitch, yaw
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])
    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])
    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])
    R = np.dot(R_z, np.dot(R_y, R_x))

    # Create the 4x4 transformation matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [tx, ty, tz]
    return T



def main():
    print("Inside Main")
    # Load the URDF file
    urdf_model = urdf.URDF.from_xml_file("/home/rohin/camlid/camlidconfig/ros2_ws/src/my_package/urdf/vehicle.urdf")
    #Extract the base to world transform first
    base_link_name = 'base_link'
    base_to_world_transform = np.eye(4)  # Identity matrix

    # Collect transformations from the root link to the base link
    for joint in urdf_model.joints:
        if joint.parent == base_link_name:
            T = create_transformation_matrix(joint.origin.xyz, joint.origin.rpy)
            base_to_world_transform = np.dot(base_to_world_transform, T)

    # Now base_to_world_transform holds the transformation from base_link to world
    print("Base to world transformation matrix:")
    print(base_to_world_transform)

    T_base_to_world = base_to_world_transform



    # Extract camera transformations relative to the base frame
    camera_transforms_base = {}
    camera_joints = [
        "base_link_to_camera_front_wide_joint",
        "camera_front_wide_to_camera_front_left_joint",
        "camera_front_wide_to_camera_front_narrow_joint",
        "camera_front_wide_to_camera_front_right_joint",
        "camera_front_wide_to_camera_back_right_joint",
        "camera_front_wide_to_camera_back_joint",
        "camera_front_wide_to_camera_back_left_joint"
    ]

    for joint in urdf_model.joints:
        if joint.name in camera_joints:
            child_link = joint.child
            T = create_transformation_matrix(joint.origin.xyz, joint.origin.rpy)
            camera_transforms_base[child_link] = T

    # Convert camera transformations to the world frame
    camera_transforms_world = {}
    for camera, transform_base in camera_transforms_base.items():
        transform_world = np.dot(T_base_to_world, transform_base)
        camera_transforms_world[camera] = transform_world

    # Print the camera transformations in the world frame
    for camera, transform in camera_transforms_world.items():
        print(f"{camera} transformation in world frame:")
        print(transform)

if __name__ == '__main__':
    main()

