import os
from glob import glob
from setuptools import find_packages, setup

package_name = '650610856_final'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*launch.py")),
        (os.path.join("share", package_name, "myword_650610856"), glob("")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pw-wq',
    maintainer_email='jaroenwachiirasak@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_node = 650610856_final.lidar_scan_node:main',
            'cmd_node = 650610856_final.cmd_msg_node:main',
            'turtle_node = 650610856_final.turtlebot_scan_service:main',
        ],
    },
)
