from setuptools import setup
from glob import glob
import os

package_name = 'my_nav2_pointcloud_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Nav2 bringup with direct PointCloud2 obstacle source',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'send_goal = my_nav2_pointcloud_pkg.send_goal:main',
            'cmd_to_aimavel = my_nav2_pointcloud_pkg.cmd_to_aimavel:main',
        ],
    },
)
