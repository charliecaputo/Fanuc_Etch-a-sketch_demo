from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'test_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=[
        'setuptools',
        'pynput',
        'smbus2'
        
        ],

    zip_safe=True,
    maintainer='fanuc',
    maintainer_email='fanuc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'keyboard_tool_teleop = test_py.keyboard_tool_teleop:main',
            'test = test_py.test:main',
            'manual_init = test_py.manual_init:main',
            'cartesian_kb_teleop = test_py.cartesian_kb_teleop:main',
            'servo_control = test_py.servo_control:main',
            'jog_listener_node = test_py.jog_listener_node:main',
            'encoder_read = test_py.encoder_read:main',
        ],
    },
)
