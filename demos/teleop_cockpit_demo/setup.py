from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'teleop_cockpit_demo'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'web'), glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='dev@example.com',
    description='Browser-based Teleoperation Cockpit Demo with Real-time HUD Video Stream and Low-Latency Control',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'cockpit_bridge_node = teleop_cockpit_demo.cockpit_bridge_node:main',
        ],
    },
)
