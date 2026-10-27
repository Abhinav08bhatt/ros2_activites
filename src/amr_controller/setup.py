import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'amr_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Install Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # Install Config YAML files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='avi',
    maintainer_email='placeholder@gmail.com',
    description='project question 3 controller',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'amr = amr_controller.amr_node:main',
            'supervisor = amr_controller.supervisor_node:main'
        ],
    },
)
