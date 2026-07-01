from setuptools import setup
from glob import glob
import os

package_name = 'fanuc_crx_xy_demo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Brian Spence',
    maintainer_email='brian.spence@example.com',
    description='ROS 2 demo package for Raspberry Pi 5 dual AS5600 encoder feedback to FANUC CRX target pose generation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dual_as5600_mux_node = fanuc_crx_xy_demo.dual_as5600_mux_node:main',
            'xy_target_node = fanuc_crx_xy_demo.xy_target_node:main',
            'fanuc_moveit_target_bridge_example = fanuc_crx_xy_demo.fanuc_moveit_target_bridge_example:main',
            'encoder_test = fanuc_crx_xy_demo.encoder_test:main'
        ],
    },
)
