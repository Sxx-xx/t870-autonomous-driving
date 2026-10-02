from setuptools import setup, find_packages
import os
from glob import glob

package_name = 't870_track'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maxim Yastremsky',
    maintainer_email='maxim.yastremsky@egnition-hamburg.de',
    description='The t870_track package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            't870_tracker = t870_track.t870_track:main',
            't870_tracker_oa = t870_track.t870_track_oa:main',
            't870_status = t870_track.t870_status1:main',
        ],
    },
)