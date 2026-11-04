from setuptools import find_packages, setup

package_name = 'pantilt_tracker'

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
    description='pantilt tracker package',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "target_simulator = pantilt_tracker.target_simulator:main",
            "pan_tilt_broadcaster = pantilt_tracker.pan_tilt_broadcaster:main",
            "track_controller_node = pantilt_tracker.track_controller_node:main"
        ],
    },
)
