from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'perception_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='todo@todo.com',
    description='YOLO Detection과 LiDAR 데이터를 융합하여 객체 정보를 발행하는 노드',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'perception_bridge_node = perception_bridge.perception_bridge_node:main',
        ],
    },
)
