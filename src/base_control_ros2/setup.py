from setuptools import setup
import os
from glob import glob

package_name = 'base_control_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files. This is the most important line here!
        (os.path.join('share', package_name,'launch'), glob('launch/*launch.py')),   
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='ROS 2 serial base controller for Bingda/Nano controller chassis.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_node = base_control_ros2.test_node:main',
            'base_control_node = base_control_ros2.base_control_ros2:main'
        ],
    },
)
