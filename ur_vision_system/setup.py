from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ur_vision_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # Include URDF files
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf.xacro')),
        # Include model files
        ('share/' + package_name + '/models/test_cylinder', glob('models/test_cylinder/*')),
        ('share/' + package_name + '/models/test_box', glob('models/test_box/*')),
        ('share/' + package_name + '/models/test_prism', glob('models/test_prism/*')),
        ('share/' + package_name + '/models/world_camera', glob('models/world_camera/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xichen',
    maintainer_email='x93hu@uwaterloo.ca',
    description='Environment configuration package for adding fixed camera to UR Gazebo simulation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # No Python nodes - this is pure environment configuration package
        ],
    },
)
