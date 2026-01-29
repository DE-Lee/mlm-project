import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'camera_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # config 폴더의 모든 yaml 파일을 share 폴더로 복사
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # launch 폴더의 모든 launch 파일을 share 폴더로 복사
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='11306260+liangfuyuan@user.noreply.gitee.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'color_sender = camera_utils.color_sender:main',
        ],
    },
)
