from setuptools import setup
import os
from glob import glob

package_name = 'v2i_gridcalib_rasterizer'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@todo.com',
    description='ROS2 node for distance-adaptive point cloud rasterization from the V2I-GridCalib paper.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rasterizer_node = v2i_gridcalib_rasterizer.rasterizer_node:main'
        ],
    },
)