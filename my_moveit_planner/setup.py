from setuptools import find_packages, setup

package_name = 'my_moveit_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xichen',
    maintainer_email='x93hu@uwaterloo.ca',
    description='机械臂控制包',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'joint_controller = my_moveit_planner.joint_controller:main',
        ],
    },
)