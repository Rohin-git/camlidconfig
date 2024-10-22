from setuptools import setup

package_name = 'my_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rohin',
    maintainer_email='rohinjohnmathews@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_package.my_node:main',
            'lidar_functions_node = my_package.lidar_functions:main',
            'depth_estimator_node = my_package.depth_estimator:main',
            'depth_estimator_lid_cam_node = my_package.depth_estimator_combined:main',
            'view_lid_cam_node = my_package.view_lidar_camera_batch_data:main',
            'extract_world_transform_node = my_package.extract_transforms_world:main',
            'lidar_publisher_node = my_package.lidar_publisher_rviz:main',
            'camera_functions_node = my_package.camera_functions:main'
        ],
    },
)

