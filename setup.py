import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'cc02_autodrive'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, [
            'package.xml',
            'cc02_autodrive/wp_position_basic.csv',
            'cc02_autodrive/wp_position_advance.csv',
        ]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryutakawamoto',
    maintainer_email='longtaichuanben@gmail.com',
    description='研究活動:PID制御による自律走行',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pid_node = cc02_autodrive.pid_controller:main',
            'stanley_node = cc02_autodrive.stanley_controller:main',
            'pure_pursuit_node = cc02_autodrive.pure_pursuit_controller:main',
            'gnss_logger_node = cc02_autodrive.gnss_logger:main',
        ],
    },
)
