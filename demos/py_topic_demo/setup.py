from setuptools import find_packages, setup

package_name = 'py_topic_demo'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='dev@example.com',
    description='Python Topic Publisher and Subscriber Demo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'py_talker = py_topic_demo.talker:main',
            'py_listener = py_topic_demo.listener:main',
            'py_service_server = py_topic_demo.service_server:main',
            'py_action_server = py_topic_demo.action_server:main',
        ],
    },
)
