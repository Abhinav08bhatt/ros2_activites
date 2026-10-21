from setuptools import find_packages, setup

package_name = 'robot_py_app'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='avi',
    maintainer_email='placeholder@gmail.com',
    description='the practice set for beginner level ros',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot = robot_py_app.robot_node:main',
            'monitor = robot_py_app.monitor_node:main'
        ],
    },
)
