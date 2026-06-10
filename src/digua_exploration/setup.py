from setuptools import setup
from glob import glob
import os

package_name = 'digua_exploration'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='sunrise@example.com',
    description='Frontier exploration for Digua robot autonomous mapping.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'frontier_explorer_node = digua_exploration.frontier_explorer_node:main',
        ],
    },
)
