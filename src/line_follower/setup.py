from setuptools import setup
import os
from glob import glob

package_name = 'line_follower'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jordan Palafox',
    maintainer_email='a00835705@tec.mx',
    description='Nodo de seguimiento de línea para PuzzleBot usando OpenCV y ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_follower = line_follower.line_follower:main',
        ],
    },
) 