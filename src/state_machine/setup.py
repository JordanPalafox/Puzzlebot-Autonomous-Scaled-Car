from setuptools import setup
import os
from glob import glob

package_name = 'state_machine'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f'{package_name}.behaviors'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jordan Palafox',
    maintainer_email='a00835705@tec.mx',
    description='Máquina de estados para el Puzzlebot usando py_trees',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_machine = state_machine.state_machine:main',
            'send_state_command = state_machine.send_state_command:main',
        ],
    },
)
