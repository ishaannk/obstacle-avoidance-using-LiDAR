from setuptools import setup
import os
from glob import glob

package_name = 'rplidar_obstacle_avoidance'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Obstacle avoidance package for RPLIDAR C1',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_avoidance_node = rplidar_obstacle_avoidance.obstacle_avoidance_node:main',
            'obstacle_avoidance_visual = rplidar_obstacle_avoidance.obstacle_avoidance_visual:main'
        ],
    },
)