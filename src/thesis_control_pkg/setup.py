from setuptools import setup
from glob import glob
import os

package_name = 'thesis_control_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[
        package_name,
        package_name + '.sensor_fusion',
        package_name + '.controller',
        package_name + '.safety',
        package_name + '.utils',
        package_name + '.scripts',  # ← Added for data_logger
    ],
    data_files=[
        # Resource index (FIXED PATH)
        ('share/ament_index/resource_index/packages',
            [os.path.join('resource', package_name)]),
        
        # Package manifest
        ('share/' + package_name, ['package.xml']),
        
        # Launch files (explicit list - more reliable than glob)
        ('share/' + package_name + '/launch', [
            'thesis_control_pkg/launch/warehouse_asmc_launch.py',
            'thesis_control_pkg/launch/warehouse_pid_launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Sensor Fusion Control Thesis Package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'adaptive_ekf_node = thesis_control_pkg.sensor_fusion.adaptive_ekf_node:main',
            'asmc_controller_node = thesis_control_pkg.controller.asmc_controller_node:main',
            'pid_controller_node = thesis_control_pkg.controller.pid_controller_node:main',
            'safety_layer_node = thesis_control_pkg.safety.safety_layer_node:main',
            'trajectory_generator_node = thesis_control_pkg.utils.trajectory_generator:main',
            'data_logger = thesis_control_pkg.scripts.data_logger_node:main',
        ],
    },
)
