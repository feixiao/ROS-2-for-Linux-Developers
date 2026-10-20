from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_simulation_demo'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='dev@example.com',
    description='Zero-Hardware Robot 2D/3D Physics, Raycasting LiDAR, and Kinematics Simulation Demo',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'simulated_robot_node = robot_simulation_demo.simulated_robot_node:main',
        ],
    },
)
