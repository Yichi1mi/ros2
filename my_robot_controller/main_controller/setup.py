from setuptools import setup

package_name = 'main_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xichen',
    maintainer_email='x93hu@uwaterloo.ca',
    description='Main controller for robot arm',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_controller = main_controller.main_controller:main',
        ],
    },
)
