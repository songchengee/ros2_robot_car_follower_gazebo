from setuptools import setup
import os
from glob import glob

package_name = 'follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch/'),glob('launch/*launch.[pxy][yma]*')),
        #('share/' + package_name, ['ros2_controller.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A depth camera follower node in ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follower = follower.follow_node:main',
            'wheels_control = follower.wheels_control_node:main',
        ],
    },
)
