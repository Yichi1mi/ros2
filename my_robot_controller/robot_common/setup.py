from setuptools import find_packages, setup

package_name = 'robot_common'

setup(
    name=package_name,
    version='0.0.0',
    packages=['robot_common'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xichen',
    maintainer_email='x93hu@uwaterloo.ca',
    description='Common utilities for robot control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # No executable scripts in this package
        ],
    },
)