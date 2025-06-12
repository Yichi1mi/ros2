from setuptools import find_packages, setup

package_name = 'move_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'robot_common'],
    zip_safe=True,
    maintainer='xichen',
    maintainer_email='x93hu@uwaterloo.ca',
    description='UR_control_pack',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'joint_controller = move_node.joint_controller:main',
            'position_controller = move_node.position_controller:main',
            'gripper_controller = move_node.gripper_controller:main',
            'robot_arm_api = move_node.robot_arm_controller:main',
        ],
    },
)