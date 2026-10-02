from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'ma_rrt_path_plan'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'waypoints'), glob('waypoints/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maxim Yastremsky',
    maintainer_email='maxim.yastremsky@egnition-hamburg.de',
    description='The ma_rrt_path_plan package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = ma_rrt_path_plan.main:main',
            'MaRRTPathPlanNode = ma_rrt_path_plan.MaRRTPathPlanNode:main',
            'ma_rrt = ma_rrt_path_plan.ma_rrt:main',
            'track = ma_rrt_path_plan.track:main',
            'main_oa = ma_rrt_path_plan.main_oa:main',
            'ma_rrt_oa = ma_rrt_path_plan.ma_rrt_oa:main',
            'MaRRTPathPlanNode_oa = ma_rrt_path_plan.MaRRTPathPlanNode_oa:main',
            'track_oa = ma_rrt_path_plan.track_oa:main',
            'ERP42_status1 = ma_rrt_path_plan.ERP42_status1:main',
        ],
    },
)
